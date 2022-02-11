import can
import cantools
import time
import math
from odrive.enums import *
from cmd_id_enums import *
from motor import *

# Import database containing can messages
db = cantools.database.load_file("odrive-cansimple.dbc")

# bus = can.Bus("vcan0", bustype="virtual")
bus = can.Bus("can0", bustype="socketcan")

M0 = Motor(0x00, bus, db)
M1 = Motor(0x01, bus, db)

#M0.init() # Do a full calibration sequence
#time.sleep(2)
#M1.init() # Do a full calibration sequence

M0.setControlMode(AXIS_STATE_CLOSED_LOOP_CONTROL)
M1.setControlMode(AXIS_STATE_CLOSED_LOOP_CONTROL)

M0.setLimits(4,10)
M1.setLimits(4,10)

def runSin():
    t0 = time.monotonic()
    while True:
        M0.sendSetpoint(4.0 * math.sin((time.monotonic() - t0) * 2), 0,0)
        M1.sendSetpoint(4.0 * math.sin((time.monotonic() - t0) * 2), 0, 0)
        time.sleep(0.01)

#runSin()