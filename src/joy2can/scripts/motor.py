from cmd_id_enums import *
from odrive.enums import *
import can
import time
import cantools


class Motor:
	def __init__(self, axisID, bus, db, encoder_cpr, kp, ki, kd):
		self.axisID = axisID
		self.axisID_shifted = self.axisID << 5
		self.bus = bus
		self.db = db
		self.sp = 0
		self.encoder_cpr = encoder_cpr
		self.kp = kp
		self.ki = ki
		self.kd = kd

	def init(self):
		# Calibration sequence for motors
		print("\nRequesting AXIS_STATE_FULL_CALIBRATION_SEQUENCE (0x03) on axisID's: " + str(self.axisID))
		# Import the message that should be sent
		msg = self.db.get_message_by_name('Set_Axis_State')
		# Encode message, using format 'Command': ENUM (from odrive.enums)
		data = msg.encode({'Axis_Requested_State': AXIS_STATE_FULL_CALIBRATION_SEQUENCE})
		# Update the msg variable to be a CAN message
		msg = can.Message(arbitration_id=msg.frame_id | self.axisID_shifted, is_extended_id=False, data=data)

		# Try to send the CAN message to the bus
		try:
			self.bus.send(msg)
			print("Message sent on {}".format(self.bus.channel_info))
		except can.CanError:
			print("Message NOT sent!  Please verify can0 is working first")

		print("Waiting for calibration to finish...")

		# Read messages infinitely and wait for the right ID to show up
		while True:
			msg = self.bus.recv()
			if msg.arbitration_id == ((self.axisID_shifted) | self.db.get_message_by_name('Heartbeat').frame_id):
				current_state = self.db.decode_message('Heartbeat', msg.data)['Axis_State']
				if current_state == AXIS_STATE_IDLE:
					print("\nAxis has returned to Idle state.")
					break

		# Check if any errors were received
		for msg in self.bus:
			if msg.arbitration_id == ((self.axisID_shifted) | self.db.get_message_by_name('Heartbeat').frame_id):
				errorCode = self.db.decode_message('Heartbeat', msg.data)['Axis_Error']
				if errorCode == AXIS_ERROR_NONE:
					print("No errors")
				else:
					print("Axis error!  Error code: " + str(hex(errorCode)))
				break

	def setAxisState(self, axisState):
		# Set axis state
		print("\nPutting axis", self.axisID, "into axis state ", axisState)
		data = self.db.encode_message('Set_Axis_State', {'Axis_Requested_State': axisState})
		msg = can.Message(arbitration_id=self.axisID_shifted | SET_AXIS_REQUESTED_STATE, is_extended_id=False,
						  data=data)
		# print(msg)

		# Try to send the CAN message to the bus
		try:
			self.bus.send(msg)
			print("Message successfully sent on {}".format(self.bus.channel_info))
		except can.CanError:
			print("Message NOT sent!")

		# Wait for reply
		for msg in self.bus:
			if msg.arbitration_id == ODRIVE_HEARTBEAT_MESSAGE | self.axisID_shifted:
				print("\nReceived Axis heartbeat message:")
				msg = self.db.decode_message('Heartbeat', msg.data)
				# print(msg)
				if msg['Axis_State'] == axisState:
					print("Axis", self.axisID, "has entered state ", axisState)
				else:
					print("Axis", self.axisID, "has failed to enter axis state ", axisState)
				break

	def setControlMode(self, inputMode, controlMode):
		# Set control mode
		print("\nPutting axis", self.axisID, "into control mode", controlMode)
		data = self.db.encode_message('Set_Controller_Mode', {'Input_Mode': inputMode, 'Control_Mode': controlMode})
		msg = can.Message(arbitration_id=self.axisID_shifted | SET_CONTROLLER_MODES, is_extended_id=False, data=data)

		# Try to send the CAN message to the bus
		try:
			self.bus.send(msg)
			print("Message successfully sent on {}".format(self.bus.channel_info))
		except can.CanError:
			print("Message NOT sent!")

	def setInputMode(self, inputMode, controlMode):
		# Set closed loop control loop
		print("\nSetting input mode for axis ", self.axisID, " to ", inputMode)
		data = self.db.encode_message('Set_Controller_Mode', {'Input_Mode': inputMode, 'Control_Mode': controlMode})
		msg = can.Message(arbitration_id=self.axisID_shifted | SET_CONTROLLER_MODES, is_extended_id=False, data=data)
		# print(msg)

		# Try to send the CAN message to the bus
		try:
			self.bus.send(msg)
			print("Message successfully sent on {}".format(self.bus.channel_info))
		except can.CanError:
			print("Message NOT sent!")

	def configureTrapTraj(self, trajVelLimit, trajAccelLimit, trajDecelLimit, trajInertia):
		print("Setting trajectory velocity limit...")
		data = self.db.encode_message('Set_Traj_Vel_Limit', {'Traj_Vel_Limit': trajVelLimit})
		msg = can.Message(arbitration_id=self.axisID_shifted | SET_TRAJ_VEL_LIMIT, is_extended_id=False, data=data)
		# Try to send the CAN message to the bus
		try:
			self.bus.send(msg)
			print("Message successfully sent on {}".format(self.bus.channel_info))
		except can.CanError:
			print("Message NOT sent!")

		print("Setting trajectory acceleration limits...")
		data = self.db.encode_message('Set_Traj_Accel_Limits',
									  {'Traj_Accel_Limit': trajAccelLimit, 'Traj_Decel_Limit': trajDecelLimit})
		msg = can.Message(arbitration_id=self.axisID_shifted | SET_TRAJ_ACCEL_LIMIT, is_extended_id=False, data=data)
		# Try to send the CAN message to the bus
		try:
			self.bus.send(msg)
			print("Message successfully sent on {}".format(self.bus.channel_info))
		except can.CanError:
			print("Message NOT sent!")

		print("Setting trajectory inertia...")
		data = self.db.encode_message('Set_Traj_Inertia', {'Traj_Inertia': trajInertia})
		msg = can.Message(arbitration_id=self.axisID_shifted | SET_TRAJ_INERTIA, is_extended_id=False, data=data)
		# Try to send the CAN message to the bus
		try:
			self.bus.send(msg)
			print("Message successfully sent on {}".format(self.bus.channel_info))
		except can.CanError:
			print("Message NOT sent!")

	def setLimits(self, vel_limit, current_limit):
		print("Setting velocity and current limits...")
		data = self.db.encode_message('Set_Limits', {'Velocity_Limit': vel_limit, 'Current_Limit': current_limit})
		msg = can.Message(arbitration_id=self.axisID_shifted | SET_LIMITS, is_extended_id=False, data=data)
		self.bus.send(msg)

	def sendSetpoint(self, setpoint, vel_ff, torque_ff):
		data = self.db.encode_message('Set_Input_Pos',
									  {'Input_Pos': setpoint, 'Vel_FF': vel_ff, 'Torque_FF': torque_ff})
		msg = can.Message(arbitration_id=self.axisID_shifted | SET_INPUT_POS, data=data, is_extended_id=False)
		self.bus.send(msg)

	def setVelocity(self, velocity, torque_ff):
		data = self.db.encode_message('Set_Input_Vel', {'Input_Torque_FF': torque_ff, 'Input_Vel': velocity})
		msg = can.Message(arbitration_id=self.axisID_shifted | SET_INPUT_VEL, data=data, is_extended_id=False)
		self.bus.send(msg)

	def setTorque(self, torque):
		print("Setting torque to:", torque)
		data = self.db.encode_message('Set_Input_Torque', {'Input_Torque': torque})
		msg = can.Message(arbitration_id=self.axisID_shifted | SET_INPUT_VEL, data=data, is_extended_id=False)
		self.bus.send(msg)

	def getEstimates(self):
		"""
		This function will get the position estimate, velocity estimate from ODrive via CAN bus.
		It will then calculate the encoder position based on the position estimate.

		:return posEstimate, velEstimate, encEstimate

		"""
		while True:
			msg = self.bus.recv()
			if msg.arbitration_id == (
					(self.axisID_shifted) | self.db.get_message_by_name('Get_Encoder_Estimates').frame_id):
				posEstimate = self.db.decode_message('Get_Encoder_Estimates', msg.data)['Pos_Estimate']
				velEstimate = self.db.decode_message('Get_Encoder_Estimates', msg.data)['Vel_Estimate']
				encEstimate = posEstimate*self.encoder_cpr
				return posEstimate, velEstimate, encEstimate

	def getEncoderCount(self):
		msg = self.db.get_message_by_name('Get_Encoder_Count')
		data = [0, 0, 0, 0, 0, 0, 0, 0]
		msg = can.Message(arbitration_id=msg.frame_id | self.axisID_shifted, is_extended_id=False, data=data, is_remote_frame=True)
		self.bus.send(msg)
		while True:
			msg = self.bus.recv()
			if msg.arbitration_id == ((self.axisID_shifted) | self.db.get_message_by_name('Get_Encoder_Count').frame_id):
				CPR = self.db.decode_message('Get_Encoder_Count', msg.data)['Count_in_CPR']
				shadowCount = self.db.decode_message('Get_Encoder_Count', msg.data)['Shadow_Count']
				return CPR, shadowCount


