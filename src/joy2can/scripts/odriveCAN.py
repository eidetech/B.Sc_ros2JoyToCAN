#!/usr/bin/env python3
import can
import cantools
import time
import math
from odrive.enums import *
from cmd_id_enums import *
from motor import *
from joySubscriber import *


def main(args=None):
    rclpy.init(args=args)
    joySubscriber = JoySubscriber()

    # Import database containing can messages (ROS2 environment could not find the file. Put absolute path to avoid error)
    db = cantools.database.load_file("/home/marcus/github/B.Sc_ros2JoyToCAN/src/joy2can/scripts/odrive-cansimple.dbc")

    #bus = can.Bus("vcan0", bustype="virtual")
    bus = can.Bus("can0", bustype="socketcan")
    M0 = Motor(0x00, bus, db)
    M1 = Motor(0x01, bus, db)

    # M0.init() # Do a full calibration sequence
    # time.sleep(2)
    # M1.init() # Do a full calibration sequence

    M0.setControlMode(AXIS_STATE_CLOSED_LOOP_CONTROL)
    M1.setControlMode(AXIS_STATE_CLOSED_LOOP_CONTROL)

    M0.setLimits(10, 10)
    M1.setLimits(10, 10)

    while True:
        rclpy.spin_once(joySubscriber)
        #runSin()
        M0.x = M0.x + (joySubscriber.joyZ / 2)
        M0.sendSetpoint(M0.x, 0, 0)

        M1.x = M1.x + (joySubscriber.joyX / 2)
        M1.sendSetpoint(M1.x, 0, 0)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    joySubscriber.destroy_node()
    rclpy.shutdown()

    def runSin():
        t0 = time.monotonic()
        while True:
            M0.sendSetpoint(4.0 * math.sin((time.monotonic() - t0) * 2), 0, 0)
            M1.sendSetpoint(4.0 * math.sin((time.monotonic() - t0) * 2), 0, 0)
            time.sleep(0.01)

if __name__ == '__main__':
    main()
