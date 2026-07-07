# Pick-and-place demo: grasp at point A, release at point B
import sys, os, time
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from robot_arm import ArmRobot, config
from robot_arm.ik_solver import IKSolver

# Config (adjust XYZ for your setup)
PICK_XYZ   = (0.12, 0.00, 0.02)
PLACE_XYZ  = (0.12, -0.08, 0.02)
Z_APPROACH = 0.08
GRIPPER_OPEN  = 40.0
GRIPPER_CLOSE = 0.0
SPEED = 1500

ik = IKSolver()
arm = ArmRobot()
arm.enable_all()
time.sleep(0.3)


def move_to(x, y, z, gripper, label, seed):
    angles = ik.solve_position(x, y, z, seed_angles_deg=seed)
    if angles is None:
        print("  IK FAILED for %s" % label)
        return seed
    for i, sid in enumerate(config.SERVO_IDS):
        lo, hi = config.get_joint_limit(sid)
        if not (lo <= angles[i] <= hi):
            print("  LIMIT VIOLATION J%d=%.1f" % (sid, angles[i]))
            return seed
    angles[5] = gripper
    print("  -> %s" % str([("%+.1f" % a) for a in angles]))
    arm.sync.execute_trajectory([seed, angles], duration_ms=SPEED, steps=60)
    time.sleep(0.3)
    return angles


seed = arm.read_all_angles_list()
print("=== Pick & Place Demo ===\n")
print("Pick:  (%.2f, %.2f, %.2f)" % PICK_XYZ)
print("Place: (%.2f, %.2f, %.2f)\n" % PLACE_XYZ)

print("1. Approach above pick...")
seed = move_to(PICK_XYZ[0], PICK_XYZ[1], PICK_XYZ[2] + Z_APPROACH, GRIPPER_OPEN, "above-pick", seed)

print("2. Descend to pick...")
seed = move_to(PICK_XYZ[0], PICK_XYZ[1], PICK_XYZ[2], GRIPPER_OPEN, "pick", seed)

print("3. Grasp (close gripper)...")
seed[5] = GRIPPER_CLOSE
arm.sync.execute_trajectory([seed, seed], duration_ms=500, steps=10)
time.sleep(0.5)

print("4. Lift...")
seed = move_to(PICK_XYZ[0], PICK_XYZ[1], PICK_XYZ[2] + Z_APPROACH, GRIPPER_CLOSE, "lift", seed)

print("5. Move above place...")
seed = move_to(PLACE_XYZ[0], PLACE_XYZ[1], PLACE_XYZ[2] + Z_APPROACH, GRIPPER_CLOSE, "above-place", seed)

print("6. Descend to place...")
seed = move_to(PLACE_XYZ[0], PLACE_XYZ[1], PLACE_XYZ[2], GRIPPER_CLOSE, "place", seed)

print("7. Release (open gripper)...")
seed[5] = GRIPPER_OPEN
arm.sync.execute_trajectory([seed, seed], duration_ms=500, steps=10)
time.sleep(0.5)

print("8. Retreat...")
seed = move_to(PLACE_XYZ[0], PLACE_XYZ[1], PLACE_XYZ[2] + Z_APPROACH, GRIPPER_OPEN, "retreat", seed)

print("\nHoming...")
arm.go_home()
time.sleep(1.5)
arm.close()
print("=== Done ===")
