# Workspace exploration: sample XYZ grid, solve IK, find reachable volume
import sys, os, json, time
import numpy as np
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from robot_arm.ik_solver import IKSolver
from robot_arm import config

print("=== Workspace Explorer ===\n")
ik = IKSolver()

X = np.linspace(-0.25, 0.25, 30)
Y = np.linspace(-0.25, 0.10, 20)
Z = np.linspace(0.01, 0.35, 20)

print("Grid: X[%.2f,%.2f] Y[%.2f,%.2f] Z[%.2f,%.2f]" % (
    X[0], X[-1], Y[0], Y[-1], Z[0], Z[-1]))
total = len(X) * len(Y) * len(Z)
print("Points: %d\n" % total)

reachable = []
blocked_j1 = []
no_ik = []
count = 0; t0 = time.time()

for x in X:
    for y in Y:
        for z in Z:
            count += 1
            if count % 3000 == 0:
                print("\r  %d/%d (%.0f%%) %.0fs..." % (
                    count, total, 100*count/total, time.time()-t0), end="")
            angles = ik.solve_position(x, y, z)
            if angles is None:
                no_ik.append((x, y, z)); continue
            j1_lo, j1_hi = config.get_joint_limit(1)
            if not (j1_lo <= angles[0] <= j1_hi):
                blocked_j1.append((x, y, z)); continue
            ok = True
            for i, sid in enumerate(config.SERVO_IDS):
                lo, hi = config.get_joint_limit(sid)
                if not (lo <= angles[i] <= hi):
                    ok = False; break
            if not ok:
                continue
            # Must actually reach the target (FK error < 3cm)
            fk = ik.fk_xyz(angles)
            err = sum(abs(fk[i] - [x, y, z][i]) for i in range(3))
            if err < 0.03:
                reachable.append((x, y, z))

print("\r  %d/%d done in %.1fs" % (total, total, time.time()-t0))

print("\n=== Results ===")
print("Reachable:       %d" % len(reachable))
print("Blocked by J1:   %d" % len(blocked_j1))
print("No IK solution:  %d" % len(no_ik))

if reachable:
    pts = np.array(reachable)
    b = blocked_j1 and np.array(blocked_j1)
    print("\nReachable bounds:")
    print("  X: [%.3f, %.3f]" % (pts[:,0].min(), pts[:,0].max()))
    print("  Y: [%.3f, %.3f]" % (pts[:,1].min(), pts[:,1].max()))
    print("  Z: [%.3f, %.3f]" % (pts[:,2].min(), pts[:,2].max()))
    r_max = np.sqrt((pts[:,:2]**2).sum(axis=1)).max()
    print("  Max radius: %.3f m" % r_max)
    # XZ slice at Y~0
    xz = [(x,z) for x,y,z in reachable if abs(y) < 0.02]
    if xz:
        xs, zs = zip(*xz)
        print("  XZ slice (Y~0): X[%.2f,%.2f] Z[%.2f,%.2f]" % (
            min(xs), max(xs), min(zs), max(zs)))

out = {
    "reachable": [(round(x,4),round(y,4),round(z,4)) for x,y,z in reachable],
    "blocked_j1": [(round(x,4),round(y,4),round(z,4)) for x,y,z in blocked_j1],
}
outfile = os.path.join(os.path.dirname(os.path.dirname(__file__)),
                       "robot_arm", "workspace.json")
with open(outfile, "w") as f:
    json.dump(out, f)
print("\nSaved: %s" % outfile)
print("=== Done ===")
