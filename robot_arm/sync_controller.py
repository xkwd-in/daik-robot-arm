# -*- coding: utf-8 -*-
"""
六轴同步控制器 — 使用 SYNC_WRITE 指令实现多舵机同时动作

关键点：SYNC_WRITE (0x83) 将多个舵机的目标位置打包在一帧里，
各舵机收到后同时开始运动，避免逐个写入导致的"波浪效应"。
"""

import time
from . import config


class SyncController:
    """六轴同步位置控制"""

    def __init__(self, protocol):
        """
        Args:
            protocol: SerialProtocol 实例
        """
        self._proto = protocol
        self._ids = config.SERVO_IDS

    # ---- 原始位置值同步写入 ----

    def sync_write_positions(self, positions):
        """
        Sync write raw position values to all servos.

        Args:
            positions: dict {servo_id: position_value}
        """
        addr = config.ADDR_GOAL_POSITION
        data_len = 2  # 2 bytes per servo (position value)

        # SCS SYNC_WRITE format: [ADDR, DATA_LEN, ID1 LO1 HI1, ID2 LO2 HI2, ...]
        params = [addr, data_len]
        for sid in self._ids:
            pos = positions.get(sid, config.get_zero_position(sid))
            pos = max(0, min(4095, pos))
            params.append(sid)               # servo ID
            params.append(pos & 0xFF)        # position low byte
            params.append((pos >> 8) & 0xFF) # position high byte

        return self._proto.send(0xFE, config.INST_SYNC_WRITE, params)

    # ---- 角度同步写入 ----

    def _angle_to_position(self, angle, servo_id):
        lo, hi = config.get_joint_limit(servo_id)
        clamped = max(lo, min(hi, angle))
        zero = config.get_zero_position(servo_id)
        return int(zero + clamped * config.POS_PER_DEG)

    def sync_write_angles(self, angles):
        """
        同步写入角度到所有舵机。

        Args:
            angles: dict {servo_id: angle (-90° ~ +90°)} 或
                    list [angle1, angle2, ..., angle6] (按 ID 顺序)
        """
        if isinstance(angles, list):
            angles = dict(zip(config.SERVO_IDS, angles))

        positions = {
            sid: self._angle_to_position(angle, sid)
            for sid, angle in angles.items()
        }
        return self.sync_write_positions(positions)

    # ---- 同步写入 + 速度控制 ----

    def sync_write_angles_with_speed(self, angles, speeds=None):
        """
        同步写入角度 + 速度。

        Args:
            angles: dict {servo_id: angle} 或 list
            speeds:  dict {servo_id: speed} 或 None（使用默认速度）
        """
        if speeds is None:
            speeds = {sid: config.DEFAULT_SPEED for sid in config.SERVO_IDS}
        elif isinstance(speeds, list):
            speeds = dict(zip(config.SERVO_IDS, speeds))

        for sid in config.SERVO_IDS:
            if sid in speeds:
                self._proto.write_register(
                    sid, config.ADDR_GOAL_SPEED, speeds[sid], size=2
                )

        return self.sync_write_angles(angles)

    # ---- 轨迹执行（插值） ----

    def execute_trajectory(self, waypoints, duration_ms=1000, steps=50):
        """
        执行一条轨迹（线性插值）。

        Args:
            waypoints: list of list，例如 [[0,0,0,0,0,0], [30,-20,45,10,-15,60]]
            duration_ms: 总时长（毫秒）
            steps: 插值步数
        """
        if len(waypoints) < 2:
            print("需要至少 2 个路径点")
            return

        step_delay = duration_ms / 1000.0 / steps

        for i in range(steps):
            t = i / (steps - 1) if steps > 1 else 1.0
            angles = {}
            for j, sid in enumerate(config.SERVO_IDS):
                start = waypoints[0][j]
                end = waypoints[1][j]
                angles[sid] = start + (end - start) * t

            self.sync_write_angles(angles)
            time.sleep(step_delay)

    # ---- 回零 ----

    def go_home(self, speed=None):
        """所有轴回到 0°（中位）"""
        if speed is not None:
            return self.sync_write_angles_with_speed(
                {sid: 0.0 for sid in config.SERVO_IDS},
                {sid: speed for sid in config.SERVO_IDS},
            )
        return self.sync_write_angles({sid: 0.0 for sid in config.SERVO_IDS})

    # ---- 等待全部停止 ----

    def wait_all_stopped(self, timeout=5.0):
        """阻塞等待所有舵机停止运动"""
        start = time.time()
        while time.time() - start < timeout:
            moving = False
            for sid in config.SERVO_IDS:
                val = self._proto.read_register(sid, config.ADDR_MOVING, length=1)
                if val == 1:
                    moving = True
                    break
            if not moving:
                return True
            time.sleep(0.02)
        return False
