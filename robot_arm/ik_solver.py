"""IK solver using ikpy v4 + URDF kinematic chain"""
import os
import numpy as np
from ikpy.chain import Chain
from . import config

URDF_PATH = os.path.join(os.path.dirname(os.path.dirname(__file__)),
                         "urdf", "robot_arm.urdf")


class IKSolver:
    """
    IK for 6-DOF arm using ikpy 4.0.
    Chain: [base_link, joint1..joint6, tool_fixed] = 8 links
    Active joints: indices 1-6 (6 revolute joints)
    """

    def __init__(self, urdf_path=None):
        self.chain = Chain.from_urdf_file(
            urdf_path or URDF_PATH,
            active_links_mask=[False, True, True, True, True, True, True, False]
        )
        self.n_links = len(self.chain.links)  # 8

    # ---- FK ----
    def _angles_to_full(self, angles_deg):
        """[j1..j6] deg -> radians vector of length 8"""
        rad = [np.deg2rad(a) for a in angles_deg]
        return [0.0] + rad + [0.0]

    def forward_kinematics(self, angles_deg):
        """6 joint angles (deg) -> 4x4 end-effector pose"""
        return self.chain.forward_kinematics(self._angles_to_full(angles_deg))

    def fk_xyz(self, angles_deg):
        """6 joint angles (deg) -> (x, y, z)"""
        T = self.forward_kinematics(angles_deg)
        return T[:3, 3]

    # ---- IK ----
    # Default seed: slightly bent forward to escape the straight-up singularity
    _DEFAULT_SEED_DEG = [0.0, 15.0, -30.0, 10.0, -5.0, 15.0]

    def _seed_full(self, seed_angles_deg):
        if seed_angles_deg is None:
            seed_angles_deg = self._DEFAULT_SEED_DEG
        rad = [np.deg2rad(a) for a in seed_angles_deg]
        return [0.0] + rad + [0.0]

    def solve_position(self, x, y, z, seed_angles_deg=None):
        """
        IK for [x,y,z] target (position only).
        Returns 6 joint angles in degrees, or None if no solution.
        """
        seed = self._seed_full(seed_angles_deg)
        result = self.chain.inverse_kinematics(
            target_position=[x, y, z],
            initial_position=seed,
        )
        if result is None:
            return None
        return self._clamp_result(result)

    def solve_full_pose(self, x, y, z, rx, ry, rz, seed_angles_deg=None):
        """
        IK for full pose [x,y,z,rx,ry,rz] (position + ZYX Euler orientation).
        Returns 6 joint angles in degrees, or None.
        """
        seed = self._seed_full(seed_angles_deg)
        result = self.chain.inverse_kinematics(
            target_position=[x, y, z],
            target_orientation=[rx, ry, rz],
            orientation_mode="ZYX",
            initial_position=seed,
        )
        if result is None:
            return None
        return self._clamp_result(result)

    def _clamp_result(self, result):
        angles_deg = [np.rad2deg(result[i]) for i in range(1, 7)]
        for i, sid in enumerate(config.SERVO_IDS):
            lo, hi = config.get_joint_limit(sid)
            angles_deg[i] = max(lo, min(hi, angles_deg[i]))
        return angles_deg

    def get_chain_info(self):
        print("Chain links (%d total):" % self.n_links)
        for i, link in enumerate(self.chain.links):
            tag = ""
            if i == 0: tag = " (base, fixed)"
            elif i == self.n_links - 1: tag = " (tool_tip, fixed)"
            print("  [%d] %s%s" % (i, link.name, tag))
