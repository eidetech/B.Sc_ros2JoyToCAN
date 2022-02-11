from cmd_id_enums import *
from odrive.enums import *
import can
import time
import cantools

class Motor:
  def __init__(self, axisID, bus, db):
    self.axisID = axisID
    self.axisID_shifted = self.axisID << 5
    self.bus = bus
    self.db = db

  def init(self):
    # Calibration sequence for motors
    print("\nRequesting AXIS_STATE_FULL_CALIBRATION_SEQUENCE (0x03) on axisID's: " + str(self.axisID))
    # Import the message that should be sent
    msg = self.db.get_message_by_name('Set_Axis_State')
    # Encode message, using format 'Command': ENUM (from odrive.enums)
    data = msg.encode({'Axis_Requested_State': AXIS_STATE_FULL_CALIBRATION_SEQUENCE})
    # Update the msg variable to be a CAN message with
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

  def setControlMode(self, controlMode):
    # Set closed loop control loop
    print("\nPutting axis", self.axisID, "into control mode", controlMode)
    data = self.db.encode_message('Set_Axis_State', {'Axis_Requested_State': controlMode})
    msg = can.Message(arbitration_id=self.axisID_shifted | SET_AXIS_REQUESTED_STATE, is_extended_id=False, data=data)
    #print(msg)

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
        #print(msg)
        if msg['Axis_State'] == controlMode:
          print("Axis has entered control mode ", controlMode)
        else:
          print("Axis has failed to enter control mode ", controlMode)
        break

  def setLimits(self, vel_limit, current_limit):
    data = self.db.encode_message('Set_Limits', {'Velocity_Limit': vel_limit, 'Current_Limit': current_limit})
    msg = can.Message(arbitration_id=self.axisID_shifted | SET_LIMITS, is_extended_id=False, data=data)
    self.bus.send(msg)

  def sendSetpoint(self, setpoint, vel_ff, torque_ff):
      print("Going to " + str(setpoint))
      data = self.db.encode_message('Set_Input_Pos', {'Input_Pos': setpoint, 'Vel_FF': vel_ff, 'Torque_FF': torque_ff})
      msg = can.Message(arbitration_id=self.axisID_shifted | SET_INPUT_POS, data=data, is_extended_id=False)
      self.bus.send(msg)