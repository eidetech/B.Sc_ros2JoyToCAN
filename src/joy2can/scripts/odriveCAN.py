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
    M0 = Motor(0x00, bus, db)
    M1 = Motor(0x01, bus, db)

    #M0.init() # Do a full calibration sequence
    #time.sleep(2)
    #M1.init() # Do a full calibration sequence

    M0.setControlMode(AXIS_STATE_CLOSED_LOOP_CONTROL)
    M1.setControlMode(AXIS_STATE_CLOSED_LOOP_CONTROL)

    M0.setInputMode(INPUT_MODE_TRAP_TRAJ, AXIS_STATE_CLOSED_LOOP_CONTROL)
    M1.setInputMode(INPUT_MODE_TRAP_TRAJ, AXIS_STATE_CLOSED_LOOP_CONTROL)

    M0.configureTrapTraj(10, 10, 10, 0)
    M1.configureTrapTraj(10, 10, 10, 0)

    M0.setLimits(10, 10)
    M1.setLimits(10, 10)

    while True:
        rclpy.spin_once(motorSP)
        #runSin()
        M0.sp = motorSP.spL
        M0.sendSetpoint(M0.sp, 0, 0)

        M1.sp = motorSP.spR
        M1.sendSetpoint(M1.sp, 0, 0)

        print("x: " + str(motorSP.px) + ", z: " + str(motorSP.pz))
        print("M0: " + str(M0.sp) + ", M1: " + str(M1.sp))

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
