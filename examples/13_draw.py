# Robot arm drawing: pen in gripper, draw shapes on paper
import sys, os, time, math
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from robot_arm import ArmRobot, config
from robot_arm.ik_solver import IKSolver

# Adjust these to match your paper position
CX, CY = 0.10, 0.00       # center of drawing area (m)
Z_UP   = 0.06             # pen lifted
Z_DRAW = 0.01             # pen on paper
SIZE   = 0.04             # shape size (4cm)

ik = IKSolver()
arm = ArmRobot()
arm.enable_all()
time.sleep(0.3)
seed = arm.read_all_angles_list()

print("=== Robot Drawing ===\n")
print("Paper center: (%.2f, %.2f) size=%.2f" % (CX, CY, SIZE))


def move_to(x, y, z, seed):
    angles = ik.solve_position(x, y, z, seed_angles_deg=seed)
    if angles is None: return seed
    for i, sid in enumerate(config.SERVO_IDS):
        lo, hi = config.get_joint_limit(sid)
        angles[i] = max(lo, min(hi, angles[i]))
    angles[5] = 0
    arm.sync.execute_trajectory([seed, angles], duration_ms=600, steps=20)
    time.sleep(0.15)
    return angles


def line(x1, y1, x2, y2, seed, n=30):
    for i in range(n + 1):
        t = i / n
        seed = move_to(x1 + (x2-x1)*t, y1 + (y2-y1)*t, Z_DRAW, seed)
    return seed


# ---- Square ----
s = SIZE / 2
corners = [(CX-s, CY-s), (CX+s, CY-s), (CX+s, CY+s), (CX-s, CY+s), (CX-s, CY-s)]

print("--- Square ---")
print("Move to start...")
seed = move_to(corners[0][0], corners[0][1], Z_UP, seed)
seed = move_to(corners[0][0], corners[0][1], Z_DRAW, seed)

for i in range(4):
    x1, y1 = corners[i]
    x2, y2 = corners[i+1]
    print("  edge %d" % (i+1))
    seed = line(x1, y1, x2, y2, seed)

print("Pen up...")
seed = move_to(corners[0][0], corners[0][1], Z_UP, seed)

# ---- Circle ----
print("--- Circle ---")
seed = move_to(CX + SIZE, CY, Z_UP, seed)
seed = move_to(CX + SIZE, CY, Z_DRAW, seed)

for i in range(80):
    a = 2 * math.pi * (i / 80)
    seed = move_to(CX + SIZE * math.cos(a), CY + SIZE * math.sin(a), Z_DRAW, seed)

print("Pen up, homing...")
move_to(CX, CY, Z_UP, seed)
arm.go_home()
time.sleep(1)
arm.close()
print("\n=== Done ===")
