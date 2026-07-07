# Interactive joint limit finder - manual jog with auto load detection
# Usage: python 07_find_limits.py [joint_number]
#   Arrow keys or +/- to jog, Enter to mark limit, Q to quit
import sys, os, time, json, select
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from robot_arm import ArmRobot, config

OUTFILE = os.path.join(os.path.dirname(os.path.dirname(__file__)),
                       "robot_arm", "joint_limits.json")

# Load existing limits if any
if os.path.exists(OUTFILE):
    with open(OUTFILE) as f:
        limits = json.load(f)
    limits = {int(k): v for k, v in limits.items()}
else:
    limits = {}

# Which joint to calibrate
target_joint = int(sys.argv[1]) if len(sys.argv) > 1 else None
joints_to_do = [target_joint] if target_joint else config.SERVO_IDS

arm = ArmRobot()
arm.enable_all()
time.sleep(0.3)


def read_joint(sid):
    a = arm.servos[sid].read_angle()
    l = arm.servos[sid].read_load()
    return a, l


def jog(sid, delta):
    current, _ = read_joint(sid)
    if current is not None:
        arm.servos[sid].write_angle(current + delta)


def print_status(sid):
    angle, load = read_joint(sid)
    lim = limits.get(sid, {})
    a_str = "%+7.2f" % angle if angle is not None else "   ERR"
    l_str = "%5d" % load if load is not None else "  ERR"
    min_s = "%+7.1f" % lim.get("min", float("nan")) if lim.get("min") is not None else "   ?"
    max_s = "%+7.1f" % lim.get("max", float("nan")) if lim.get("max") is not None else "   ?"
    print("\r  J%d: angle=%s  load=%s  limits=[%s, %s]  " % (sid, a_str, l_str, min_s, max_s), end="")


print("=" * 55)
print("Interactive Joint Limit Calibration")
print("=" * 55)
print("Commands:  + / -   jog 5 deg")
print("           [ / ]   jog 1 deg (fine)")
print("           < / >   jog 10 deg (fast)")
print("           m = mark as MIN limit")
print("           M = mark as MAX limit")
print("           0 = go to zero")
print("           Enter = next joint")
print("           q = save & quit")
print()

sid = joints_to_do[0]
idx = 0
print("Now calibrating: Joint %d" % sid)
print_status(sid)

try:
    while True:
        cmd = input().strip()

        if cmd in ("q", "Q"):
            break
        elif cmd == "":
            # Next joint
            arm.servos[sid].write_angle(0.0)
            idx += 1
            if idx >= len(joints_to_do):
                print("\nAll joints done!")
                break
            sid = joints_to_do[idx]
            print("\nNow calibrating: Joint %d" % sid)
            print_status(sid)
            continue
        elif cmd == "+":
            jog(sid, 5)
        elif cmd == "-":
            jog(sid, -5)
        elif cmd == "[":
            jog(sid, 1)
        elif cmd == "]":
            jog(sid, -1)
        elif cmd == ">":
            jog(sid, 10)
        elif cmd == "<":
            jog(sid, -10)
        elif cmd == "0":
            arm.servos[sid].write_angle(0.0)
        elif cmd == "m":
            a, _ = read_joint(sid)
            if a is not None:
                limits[sid] = limits.get(sid, {})
                limits[sid]["min"] = round(a, 1)
                print("\n  -> MIN limit set to %+.1f deg" % a)
        elif cmd == "M":
            a, _ = read_joint(sid)
            if a is not None:
                limits[sid] = limits.get(sid, {})
                limits[sid]["max"] = round(a, 1)
                print("\n  -> MAX limit set to %+.1f deg" % a)

        time.sleep(0.15)
        print_status(sid)

except KeyboardInterrupt:
    print("\n")

finally:
    # Save
    with open(OUTFILE, "w") as f:
        json.dump({str(k): v for k, v in limits.items()}, f, indent=2)
    print("\nLimits saved: %s" % OUTFILE)
    for k, v in sorted(limits.items()):
        print("  J%s: [%s, %s]" % (k, v.get("min", "?"), v.get("max", "?")))
    arm.close()
