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
	db = cantools.database.load_file("/home/marcus/github/B.Sc_ros2JoyToCAN/src/joy2can/scripts/odrive-cansimple.dbc")

	#bus = can.Bus("vcan0", bustype="virtual")
	bus = can.Bus("can0", bustype="socketcan")
	M0 = Motor(0x00, bus, db, 8192)
	M1 = Motor(0x01, bus, db, 8192)

	#M0.init() # Do a full calibration sequence
	#time.sleep(2)
	#M1.init() # Do a full calibration sequence

	M0.setAxisState(AXIS_STATE_CLOSED_LOOP_CONTROL)
	M1.setAxisState(AXIS_STATE_CLOSED_LOOP_CONTROL)

	M0.setControlMode(INPUT_MODE_PASSTHROUGH, CONTROL_MODE_VELOCITY_CONTROL)
	M1.setControlMode(INPUT_MODE_PASSTHROUGH, CONTROL_MODE_VELOCITY_CONTROL)

	#M0.setInputMode(INPUT_MODE_TRAP_TRAJ, AXIS_STATE_CLOSED_LOOP_CONTROL)
	#M1.setInputMode(INPUT_MODE_TRAP_TRAJ, AXIS_STATE_CLOSED_LOOP_CONTROL)

	#M0.configureTrapTraj(10, 10, 10, 0)
	#M1.configureTrapTraj(10, 10, 10, 0)

	M0.setLimits(10, 10)
	M1.setLimits(10, 10)

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

	while True:
		rclpy.spin_once(motorSP)

		M0.setVelocity(motorSP.spR, 0)
		M1.setVelocity(motorSP.spL, 0)


		#runSin()
		#M0.sp = motorSP.spL
		#M0.sendSetpoint(M0.sp, 0, 0)

		#M1.sp = motorSP.spR
		#M1.sendSetpoint(M1.sp, 0, 0)

		#runSinVelocity()


		#M0.setVelocity(2, 0)
		#M1.setVelocity(2, 0)

		#print("x: " + str(motorSP.px) + ", z: " + str(motorSP.pz))
		#print("M0: " + str(M0.sp) + ", M1: " + str(M1.sp))

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



if __name__ == '__main__':
	main()
