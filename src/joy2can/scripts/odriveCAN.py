#!/usr/bin/env python3
import can
import cantools
import time
import math
from odrive.enums import *
from cmd_id_enums import *
from motor import *
from motorSetpointSub import *

def main(args=None):
	rclpy.init(args=args)
	motorSP = MotorSetpointSub()

	# Import database containing can messages (ROS2 environment could not find the file. Put absolute path to avoid error)
	db = cantools.database.load_file("src/joy2can/scripts/odrive-cansimple.dbc")

	#bus = can.Bus("vcan0", bustype="virtual")
	bus = can.Bus("can0", bustype="socketcan")
	M0 = Motor(0x00, bus, db, 8192)
	M1 = Motor(0x01, bus, db, 8192)

	M0.setAxisState(AXIS_STATE_CLOSED_LOOP_CONTROL)
	M1.setAxisState(AXIS_STATE_CLOSED_LOOP_CONTROL)

	while True:
		rclpy.spin_once(motorSP)
		print("Mode: ", motorSP.mode)
		if(motorSP.mode == 0 and motorSP.lastMode == 1): # Set paramteres for velocity control
			print("Setting control mode to velocity control")
			M0.setControlMode(INPUT_MODE_PASSTHROUGH, CONTROL_MODE_VELOCITY_CONTROL)
			M1.setControlMode(INPUT_MODE_PASSTHROUGH, CONTROL_MODE_VELOCITY_CONTROL)
			M0.setLimits(15, 10)
			M1.setLimits(15, 10)
			motorSP.lastMode = 0
		elif(motorSP.mode == 0 and motorSP.lastMode == 0): # Run system in velocity control mode
			M0.setVelocity(motorSP.angVel_q2, 0)
			M1.setVelocity(motorSP.angVel_q1, 0)
			motorSP.angPos_q1_final = motorSP.angPos_q1
			motorSP.angPos_q2_final = motorSP.angPos_q2
			posEst_M1, velEst_M1, encEst_M1 = M1.getEstimates()
			print("Position estimate q1:", posEst_M1)
		elif(motorSP.mode == 1 and motorSP.lastMode == 0): # Set parameters for position control
			print("Setting control mode to position control")
			M0.setControlMode(INPUT_MODE_PASSTHROUGH, CONTROL_MODE_POSITION_CONTROL)
			M1.setControlMode(INPUT_MODE_PASSTHROUGH, CONTROL_MODE_POSITION_CONTROL)
			M0.setLimits(4, 10)
			M1.setLimits(6, 10)
			motorSP.lastMode = 1
		elif(motorSP.mode == 1 and motorSP.lastMode == 1): # Run system in position control mode back to origo (with offsets)
			M0.sendSetpoint(0, 0, 0)
			M1.sendSetpoint(0, 0, 0)

#		posEst_M0, velEst_M0, encEst_M0 = M0.getEstimates()
#		posEst_M1, velEst_M1, encEst_M1 = M1.getEstimates()
#		print(motorSP.angPos_q1-motorSP.angPos_q1_initial, posEst_M1, (motorSP.angPos_q1-motorSP.angPos_q1_initial)-posEst_M1)

	# Destroy the node explicitly
	# (optional - otherwise it will be done automatically
	# when the garbage collector destroys the node object)
	motorSP.destroy_node()
	rclpy.shutdown()

	def runSin():
		t0 = time.monotonic()
		while True:
			M0.sendSetpoint(4.0 * math.sin((time.monotonic() - t0) * 2), 0, 0)
			M1.sendSetpoint(4.0 * math.sin((time.monotonic() - t0) * 2), 0, 0)
			time.sleep(0.01)

	def runSinVelocity():
		t0 = time.monotonic()
		while True:
			#startTime = time.time()
			sinF =4*math.sin((time.monotonic() - t0))
			M0.setVelocity(sinF, 0)
			M1.setVelocity(sinF, 0)

			#M0.getEncoderCount()
			x, y , z = M1.getEstimates()
			print(z)

			#print("sinF: ", sinF)
			#time.sleep(0.01)

if __name__ == '__main__':
	main()
