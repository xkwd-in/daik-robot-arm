# IK validation - numerical tests, no hardware
import sys, os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from robot_arm.ik_solver import IKSolver
from robot_arm import config

print("=== IK Solver Validation ===\n")

ik = IKSolver()
ik.get_chain_info()

# Test 1: FK at zero
print("\n--- Test 1: FK at zero ---")
xyz = ik.fk_xyz([0, 0, 0, 0, 0, 0])
print("FK([0,0,0,0,0,0]) -> x=%.3f y=%.3f z=%.3f m" % tuple(xyz))

# Test 2: FK -> IK -> FK round trip (with original angles as seed)
print("\n--- Test 2: Round-trip (perfect seed) ---")
orig = [0, 10, -15, 5, -5, 20]
target = ik.fk_xyz(orig)
print("FK(%s) -> (%.3f, %.3f, %.3f)" % (
    str([("%+.0f" % a) for a in orig]), *target))
solved = ik.solve_position(target[0], target[1], target[2], seed_angles_deg=orig)
if solved:
    check = ik.fk_xyz(solved)
    err_mm = [abs(check[i] - target[i]) * 1000 for i in range(3)]
    print("IK -> %s" % str([("%+.1f" % a) for a in solved]))
    print("FK -> (%.3f, %.3f, %.3f)  err: (%.1f, %.1f, %.1f) mm" % (
        check[0], check[1], check[2], err_mm[0], err_mm[1], err_mm[2]))
else:
    print("FAILED")

# Test 3: IK with default seed (realistic scenario)
print("\n--- Test 3: Realistic IK (default seed) ---")
targets = [
    (0.12, 0.00, 0.25),
    (0.15, -0.05, 0.20),
    (0.10, 0.03, 0.30),
]
for x, y, z in targets:
    r = ik.solve_position(x, y, z)
    if r:
        fk = ik.fk_xyz(r)
        err = sum(abs(fk[i] - [x, y, z][i]) * 1000 for i in range(3))
        j1_lo, j1_hi = config.get_joint_limit(1)
        status = "OK" if j1_lo <= r[0] <= j1_hi else "J1 LIMIT!"
        print("  target (%5.2f,%5.2f,%5.2f) -> %s -> FK(%5.3f,%5.3f,%5.3f) err=%.1fmm %s" % (
            x, y, z, str([("%+.0f" % a) for a in r]),
            fk[0], fk[1], fk[2], err, status))
    else:
        print("  target (%5.2f,%5.2f,%5.2f) -> NO SOLUTION" % (x, y, z))

# Test 4: J6 always >= 0
print("\n--- Test 4: J6 constraint (must be >= 0) ---")
for _ in range(5):
    r = ik.solve_position(0.10, 0.0, 0.20)
    if r and r[5] < 0:
        print("  FAIL: J6 = %.1f" % r[5])
        break
else:
    print("  OK: J6 always >= 0")

print("\n=== Done ===")
