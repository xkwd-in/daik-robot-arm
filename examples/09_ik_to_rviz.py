# IK validation: compute angles, verify via FK, print commands for RViz
import sys, os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from robot_arm.ik_solver import IKSolver

print("=== IK Solver + URDF Validation ===\n")

ik = IKSolver()

tests = [
    (0.12, 0.00, 0.20, "front-mid"),
    (0.15, -0.08, 0.15, "left-reach"),
    (0.08, 0.05, 0.25, "right-up"),
    (0.00, -0.10, 0.18, "left-low"),
]

print("Computing IK and verifying via FK:\n")
print("%-12s  %-6s %-6s %-6s  %-36s  %s" % (
    "Label", "X", "Y", "Z", "Joint Angles (deg)", "FK Err"))
print("-" * 105)

for x, y, z, label in tests:
    angles = ik.solve_position(x, y, z)
    if angles:
        fk = ik.fk_xyz(angles)
        err = sum(abs(fk[i] - [x, y, z][i]) * 1000 for i in range(3))
        a_str = "[" + " ".join("%+6.1f" % a for a in angles) + "]"
        print("%-12s  %-6.3f %-6.3f %-6.3f  %-36s  %.1f mm" % (
            label, x, y, z, a_str, err))
    else:
        print("%-12s  %-6.3f %-6.3f %-6.3f  NO SOLUTION" % (label, x, y, z))

print("\n=== RViz commands ===")
print("Terminal 1 (start RViz):")
print("  wsl -d Ubuntu-22.04")
print("  cd /mnt/d/daik/ros_ws")
print("  source /opt/ros/humble/setup.bash")
print("  source install/setup.bash")
print("  ros2 launch robot_arm_description display.launch.py")
print()
print("Terminal 2 (publish IK angles - after RViz is open):")
for x, y, z, label in tests:
    angles = ik.solve_position(x, y, z)
    if angles:
        pos_str = ",".join("%.4f" % a for a in angles)
        names = "joint1,joint2,joint3,joint4,joint5,joint6"
        print("# %s" % label)
        print("ros2 topic pub /joint_states sensor_msgs/msg/JointState "
              "\"{name: [%s], position: [%s]}\" -1" % (names, pos_str))

print("\n=== Done ===")
