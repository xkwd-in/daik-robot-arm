# -*- coding: utf-8 -*-
"""
运动学模块 — 正运动学 (FK) / 逆运动学 (IK) / 轨迹规划

当前为骨架代码，DH 参数表需根据机械臂实物测量后填入。
完成后，上层调用方式：
    ik = IKSolver(dh_params)
    joint_angles = ik.solve(x, y, z, roll, pitch, yaw)
    controller.sync_write_angles(joint_angles)
"""

import math
import numpy as np


# ============================================================
# DH 参数工具
# ============================================================

def dh_matrix(a, alpha, d, theta):
    """
    标准 DH 参数 → 4x4 齐次变换矩阵
    参数单位：a(mm), alpha(rad), d(mm), theta(rad)
    """
    ct = math.cos(theta)
    st = math.sin(theta)
    ca = math.cos(alpha)
    sa = math.sin(alpha)

    return np.array([
        [ct, -st * ca,  st * sa, a * ct],
        [st,  ct * ca, -ct * sa, a * st],
        [0,   sa,       ca,      d     ],
        [0,   0,        0,       1     ],
    ])


def forward_kinematics(dh_params, joint_angles):
    """
    正运动学：给定关节角 → 返回末端 4x4 位姿矩阵

    Args:
        dh_params: list of (a, alpha, d, theta_offset) — theta_offset 是零位偏置
        joint_angles: list of 6 floats (radians)

    Returns:
        4x4 numpy array (末端法兰坐标系在基座坐标系下的位姿)
    """
    T = np.eye(4)
    for i, (a, alpha, d, theta_off) in enumerate(dh_params):
        theta = joint_angles[i] + theta_off
        T = T @ dh_matrix(a, alpha, d, theta)
    return T


# ============================================================
# DH 参数表 — 待实物测量后填入
# ============================================================

# 格式: [(a, alpha(rad), d(mm), theta_offset(rad)), ...]  共 6 行
# 以下为占位示例（不准确的猜测值，仅示意结构）

DH_PARAMS_PLACEHOLDER = [
    (0,    math.pi/2,  100,  0),           # 关节1 → 关节2
    (120,  0,           0,  -math.pi/2),   # 关节2 → 关节3
    (0,    math.pi/2,    0,   0),           # 关节3 → 关节4
    (0,   -math.pi/2,  100,  0),           # 关节4 → 关节5
    (0,    math.pi/2,    0,   0),           # 关节5 → 关节6
    (0,    0,           60,   0),           # 关节6 → 法兰
]

# ============================================================
# IK 求解器（数值法 / 解析法，待实物标定后确定方案）
# ============================================================

class IKSolver:
    """
    逆运动学求解器

    推荐方案（按优先级）：
    1. ikpy 库 — 数值解，依赖少，适合原型验证
       pip install ikpy
    2. 解析解 — 自行推导，速度快，但需要机械臂满足 Pieper 准则
    3. 其他路径：trac_ik, pybullet, ...
    """

    def __init__(self, dh_params=None):
        self.dh_params = dh_params or DH_PARAMS_PLACEHOLDER

    def solve(self, target_pose, seed=None):
        """
        占位方法 — 待 ikpy 接入后实现

        Args:
            target_pose: 4x4 矩阵 (末端目标位姿) 或 (x,y,z,roll,pitch,yaw)
            seed: 初始猜测关节角 (迭代求解用)

        Returns:
            list of 6 floats (关节角度, 弧度)
        """
        raise NotImplementedError(
            "IK 求解未实现。"
            "请用 pip install ikpy 安装后，在 robot_arm/ik_solver.py 中实现。"
            "或手工推导解析解填入此类。"
        )

    def solve_cartesian(self, x, y, z, roll=0, pitch=0, yaw=0, seed=None):
        """
        笛卡尔位姿 → 关节角（接口方法）
        """
        cr, sr = math.cos(roll), math.sin(roll)
        cp, sp = math.cos(pitch), math.sin(pitch)
        cy, sy = math.cos(yaw), math.sin(yaw)

        R = np.array([
            [cy*cp, cy*sp*sr - sy*cr, cy*sp*cr + sy*sr],
            [sy*cp, sy*sp*sr + cy*cr, sy*sp*cr - cy*sr],
            [-sp,   cp*sr,            cp*cr           ],
        ])
        T = np.eye(4)
        T[:3, :3] = R
        T[:3, 3] = [x, y, z]

        return self.solve(T, seed=seed)


# ============================================================
# 轨迹规划器
# ============================================================

class TrajectoryPlanner:
    """关节空间轨迹插值"""

    @staticmethod
    def linear_interpolation(start_angles, end_angles, steps):
        """线性插值"""
        result = []
        for i in range(steps):
            t = i / (steps - 1) if steps > 1 else 1.0
            angles = [
                s + (e - s) * t
                for s, e in zip(start_angles, end_angles)
            ]
            result.append(angles)
        return result

    @staticmethod
    def trapezoidal_profile(start_angles, end_angles, steps,
                             accel_ratio=0.3, decel_ratio=0.3):
        """
        梯形速度曲线插值（加速 → 匀速 → 减速）

        Args:
            accel_ratio: 加速段占比
            decel_ratio: 减速段占比
        """
        result = []
        for i in range(steps):
            t = i / (steps - 1) if steps > 1 else 1.0

            if t < accel_ratio:
                s_t = (t ** 2) / (2 * accel_ratio * (1 - accel_ratio))
            elif t < 1 - decel_ratio:
                s_t = (t - accel_ratio / 2) / (1 - accel_ratio / 2 - decel_ratio / 2)
            else:
                s_t = 1 - ((1 - t) ** 2) / (2 * decel_ratio * (1 - decel_ratio))

            s_t = max(0, min(1, s_t))
            angles = [
                s + (e - s) * s_t
                for s, e in zip(start_angles, end_angles)
            ]
            result.append(angles)
        return result

    @staticmethod
    def cartesian_line(start_xyz, end_xyz, steps, ik_solver):
        """
        笛卡尔空间直线轨迹。
        每步调用 IK 求解关节角，确保末端走直线。

        Args:
            start_xyz: (x, y, z) 起点
            end_xyz: (x, y, z) 终点
            steps: 插值步数
            ik_solver: IKSolver 实例

        Returns:
            list of joint angle lists
        """
        result = []
        for i in range(steps):
            t = i / (steps - 1) if steps > 1 else 1.0
            x = start_xyz[0] + (end_xyz[0] - start_xyz[0]) * t
            y = start_xyz[1] + (end_xyz[1] - start_xyz[1]) * t
            z = start_xyz[2] + (end_xyz[2] - start_xyz[2]) * t
            angles = ik_solver.solve_cartesian(x, y, z)
            result.append(angles)
        return result
