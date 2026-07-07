"""Read EEPROM angle limits from all 6 servos (no movement needed)"""
import sys, os
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from robot_arm import SerialProtocol, config

ADDR_CCW = 6   # EEPROM: counter-clockwise angle limit (min)
ADDR_CW  = 8   # EEPROM: clockwise angle limit (max)

p = SerialProtocol()
p.open()
print("\n=== Servo EEPROM Angle Limits ===\n")
print("%6s  %10s  %10s  %10s" % ("Joint", "CCW(min)", "CW(max)", "Range"))
print("-" * 46)

for sid in config.SERVO_IDS:
    ccw = p.read_register(sid, ADDR_CCW, 2)
    cw  = p.read_register(sid, ADDR_CW, 2)
    if ccw is not None and cw is not None:
        print("   J%d    %6d      %6d      %6d" % (sid, ccw, cw, cw - ccw))
    else:
        print("   J%d    READ FAILED" % sid)
print()
p.close()
