"""Quick zero calibration - reads and saves zero offsets immediately"""
import sys, os, json
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from robot_arm import SerialProtocol, Servo, config

OUTPUT_FILE = os.path.join(os.path.dirname(__file__), "robot_arm", "zero_offset.json")

proto = SerialProtocol()
proto.open()
print("\n=== Zero Calibration ===\n")

offsets = {}
for sid in config.SERVO_IDS:
    servo = Servo(sid, proto)
    raw = servo.read_position_raw()
    angle = servo.read_angle()
    if raw is not None:
        offsets[str(sid)] = {"raw_position": raw, "angle": round(angle, 2) if angle is not None else None}
        if angle is not None:
            print(f"  J{sid}: pos={raw:>5}, angle={angle:>+7.2f} deg")
        else:
            print(f"  J{sid}: pos={raw:>5}")
    else:
        print(f"  J{sid}: READ FAILED")
        offsets[str(sid)] = {"raw_position": None, "angle": None}

with open(OUTPUT_FILE, "w", encoding="utf-8") as f:
    json.dump(offsets, f, indent=2, ensure_ascii=False)

print(f"\nSaved to: {OUTPUT_FILE}")
proto.close()
