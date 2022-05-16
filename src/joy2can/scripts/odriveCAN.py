#!/usr/bin/env python3
import can
import cantools
import time
import math
from odrive.enums import *
from cmd_id_enums import *
from motor import *
from motorSetpointSub import *
from std_msgs.msg import Float32MultiArray

class odriveCAN_data(Node):

	def __init__(self):
		super().__init__('encoder_pub')
		self.publisher_ = self.create_publisher(Float32MultiArray, 'odriveCAN_data', 10)
		timer_period = 0.01  # seconds
		self.timer = self.create_timer(timer_period, self.timer_callback)

		self.q1_pos = 0
		self.q2_pos = 0
		self.q1_pos_est = 0
		self.q2_pos_est = 0

		self.q1_vel = 0
		self.q2_vel = 0
		self.q1_vel_est = 0
		self.q2_vel_est = 0

		self.q1_e = 0
		self.q2_e = 0

	def timer_callback(self):
		msg = Float32MultiArray()
		msg.data = [float(self.q1_pos), float(self.q2_pos), float(self.q1_pos_est), float(self.q2_pos_est),
					float(self.q1_vel), float(self.q2_vel), float(self.q1_vel_est), float(self.q2_vel_est),
					float(self.q1_e), float(self.q2_e)]
		self.publisher_.publish(msg)


def main(args=None):
	rclpy.init(args=args)
	motorSP = MotorSetpointSub()
	odrive_data_pub = odriveCAN_data()

	# Import database containing can messages (ROS2 environment could not find the file. Put absolute path to avoid error)
	db = cantools.database.load_file("src/joy2can/scripts/odrive-cansimple.dbc")

	#bus = can.Bus("vcan0", bustype="virtual")
	bus = can.Bus("can0", bustype="socketcan")
	q1 = Motor(0x01, bus, db, 8192, 20, 10, 0) # axisID, bus, db, encoder_cpr, kp, ki, kd
	q2 = Motor(0x00, bus, db, 8192, 20, 10, 0) # axisID, bus, db, encoder_cpr, kp, ki, kd

	q1.setAxisState(AXIS_STATE_CLOSED_LOOP_CONTROL)
	q2.setAxisState(AXIS_STATE_CLOSED_LOOP_CONTROL)

	pid = True

	while True:
		rclpy.spin_once(motorSP)
		rclpy.spin_once(odrive_data_pub)

		posEst_q1, velEst_q1, encEst_q1 = q1.getEstimates()
		posEst_q2, velEst_q2, encEst_q2 = q2.getEstimates()
		#posEst_q1 = -posEst_q1
		#posEst_q2 = -posEst_q2

		odrive_data_pub.q1_pos_est = posEst_q1
		odrive_data_pub.q2_pos_est = posEst_q2
		odrive_data_pub.q1_pos = motorSP.angPos_q1 - motorSP.angPos_q1_initial
		odrive_data_pub.q2_pos = motorSP.angPos_q2 - motorSP.angPos_q2_initial
		odrive_data_pub.q1_vel = motorSP.angVel_q1
		odrive_data_pub.q2_vel = motorSP.angVel_q2
		odrive_data_pub.q1_vel_est = velEst_q1
		odrive_data_pub.q2_vel_est = velEst_q2

		if(motorSP.mode == 0 and motorSP.lastMode == 1): # Set parameteres for velocity control
			print("Setting control mode to velocity control")
			q2.setControlMode(INPUT_MODE_PASSTHROUGH, CONTROL_MODE_VELOCITY_CONTROL)
			q1.setControlMode(INPUT_MODE_PASSTHROUGH, CONTROL_MODE_VELOCITY_CONTROL)
			q2.setLimits(40, 25)
			q1.setLimits(40, 25)
			motorSP.lastMode = 0
			prevT = motorSP.get_clock().now().nanoseconds / 1.0e9

			eprev_q1 = 0
			eprev_q2 = 0

			eintegral_q1 = 0
			eintegral_q2 = 0

		elif(motorSP.mode == 0 and motorSP.lastMode == 0): # Run system in velocity control mode

			#print("angVel_q1:", motorSP.angVel_q1, "estVel:", velEst_q1)

			currT = motorSP.get_clock().now().nanoseconds / 1.0e9

			deltaT = float(currT - prevT)
			prevT = currT

			# Closed Loop Control for q1:
			# error
			e_q1 = (motorSP.angPos_q1 - motorSP.angPos_q1_initial) - posEst_q1

			# integral
			eintegral_q1 = eintegral_q1 + e_q1 * deltaT
			I_term_q1 = eintegral_q1 * q1.ki
			if I_term_q1 >= 30:
				I_term_q1 = 30
			elif I_term_q1 <= -30:
				I_term_q1 = -30

			# derivative
			dedt_q1 = (e_q1 - eprev_q1) / (deltaT)

			# control signal
			u_q1 = motorSP.angVel_q1 + q1.kp * e_q1 + I_term_q1 + q1.kd * dedt_q1
			uu_q1 = q1.kp * e_q1 + I_term_q1 + q1.kd * dedt_q1
			eprev_q1 = e_q1

			# Closed Loop Control for q2:
			# error
			e_q2 = (motorSP.angPos_q2 - motorSP.angPos_q2_initial) - posEst_q2

			# integral
			eintegral_q2 = eintegral_q2 + e_q2 * deltaT
			I_term_q2 = eintegral_q2 * q2.ki
			if I_term_q2 >= 30:
				I_term_q2 = 30
			elif I_term_q2 <= -30:
				I_term_q2 = -30

			# derivative
			dedt_q2 = (e_q2 - eprev_q2) / (deltaT)

			# control signal
			u_q2 = motorSP.angVel_q2 + q2.kp * e_q2 + I_term_q2 + q2.kd * dedt_q2
			uu_q2 = q2.kp * e_q2 + I_term_q2 + q2.kd * dedt_q2

			eprev_q2 = e_q2

			odrive_data_pub.q1_e = uu_q1
			odrive_data_pub.q2_e = uu_q2


			if pid: # Set velocities for q1 and q2 from control loops:
				q1.setVelocity(u_q1, 0)
				q2.setVelocity(u_q2, 0)
			else: # Set velocities directly from velocity reference:
				q1.setVelocity(motorSP.angVel_q1, 0)
				q2.setVelocity(motorSP.angVel_q2, 0)

			#print("q1 initial pos:", motorSP.angPos_q1_initial, "q2 initial pos:", motorSP.angPos_q2_initial)
			#print("q1:", posEst_q1, "q2:", posEst_q2)

		elif(motorSP.mode == 1 and motorSP.lastMode == 0): # Set parameters for position control
			print("Setting control mode to position control")
			q2.setControlMode(INPUT_MODE_PASSTHROUGH, CONTROL_MODE_POSITION_CONTROL)
			q1.setControlMode(INPUT_MODE_PASSTHROUGH, CONTROL_MODE_POSITION_CONTROL)
			q2.setLimits(4, 25)
			q1.setLimits(6, 25)

			motorSP.lastMode = 1
		elif(motorSP.mode == 1 and motorSP.lastMode == 1): # Run system in position control mode back to origo (with offsets)
			print("Going home...")
			print("q1 initial pos:", motorSP.angPos_q1_initial, "q2 initial pos:", motorSP.angPos_q2_initial)
			q2.sendSetpoint(0, 0, 0)
			q1.sendSetpoint(0, 0, 0)

	# Destroy the node explicitly
	# (optional - otherwise it will be done automatically
	# when the garbage collector destroys the node object)
	motorSP.destroy_node()
	rclpy.shutdown()

if __name__ == '__main__':
	main()
