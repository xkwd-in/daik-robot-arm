"""Re-calibrate J1 zero to current manually-centered position"""
import sys, os, json
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from robot_arm import SerialProtocol

p = SerialProtocol()
p.open()
raw = p.read_register(1, 56, 2)
print("J1 raw position: %d" % raw)

zf = os.path.join("robot_arm", "zero_offset.json")
with open(zf) as f:
    data = json.load(f)
old = data["1"]["raw_position"]
data["1"]["raw_position"] = raw
data["1"]["angle"] = 0.0
with open(zf, "w") as f:
    json.dump(data, f, indent=2)
print("J1 zero: %d -> %d" % (old, raw))

jf = os.path.join("robot_arm", "joint_limits.json")
if os.path.exists(jf):
    with open(jf) as f:
        lims = json.load(f)
    if "1" in lims:
        del lims["1"]
    with open(jf, "w") as f:
        json.dump(lims, f, indent=2)
    print("J1 limits cleared -- recalibrate with: python examples/07_find_limits.py 1")

p.close()
