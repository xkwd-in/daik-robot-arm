# IK -> Real hardware test with trajectory interpolation + limit checks
import sys, os, time
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from robot_arm import ArmRobot, config
from robot_arm.ik_solver import IKSolver

print("=== IK Real Hardware Test ===\n")

ik = IKSolver()
arm = ArmRobot()
arm.enable_all()
time.sleep(0.3)

current = arm.read_all_angles_list()
print("Current: %s" % str([("%+.1f" % a) for a in current]))

targets = [
    (0.12, 0.00, 0.20, "front-mid"),
    (0.15, -0.08, 0.15, "left-reach"),
]

for x, y, z, label in targets:
    print("\n--- %s (%.2f, %.2f, %.2f) ---" % (label, x, y, z))

    angles = ik.solve_position(x, y, z, seed_angles_deg=current)
    if angles is None:
        print("  IK FAILED - skipping")
        continue

    # Check limits
    safe = True
    for i, sid in enumerate(config.SERVO_IDS):
        lo, hi = config.get_joint_limit(sid)
        if not (lo <= angles[i] <= hi):
            print("  J%d=%.1f OUT OF LIMITS [%.1f,%.1f] - SKIPPING" % (
                sid, angles[i], lo, hi))
            safe = False
            break
    if not safe:
        continue

    print("  Target: %s" % str([("%+.1f" % a) for a in angles]))

    # Move via interpolation
    print("  Moving (2s trajectory)...")
    arm.sync.execute_trajectory([current, angles], duration_ms=2000, steps=80)
    time.sleep(0.5)

    actual = arm.read_all_angles_list()
    fk_actual = ik.fk_xyz(actual)
    print("  Reached: %s -> FK=(%.3f,%.3f,%.3f)" % (
        str([("%+.1f" % a) for a in actual]),
        fk_actual[0], fk_actual[1], fk_actual[2]))
    err = sum(abs(fk_actual[i] - [x, y, z][i]) * 1000 for i in range(3))
    print("  FK error: %.1f mm" % err)

    current = actual

# Home at the end
print("\n  Final homing...")
arm.sync.execute_trajectory([current, [0]*6], duration_ms=1500, steps=60)
time.sleep(0.5)

print("\n=== Done ===")
arm.close()
