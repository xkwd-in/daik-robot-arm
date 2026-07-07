# -*- coding: utf-8 -*-
"""
单舵机控制 — 角度读写、力矩、速度控制
"""

from . import config


def _angle_to_position(angle, servo_id):
    """Angle (relative to zero) -> raw servo position, respecting joint limits"""
    lo, hi = config.get_joint_limit(servo_id)
    clamped = max(lo, min(hi, angle))
    zero = config.get_zero_position(servo_id)
    return int(zero + clamped * config.POS_PER_DEG)


def _position_to_angle(position, servo_id):
    """Raw servo position -> angle (relative to zero), respecting joint limits"""
    lo, hi = config.get_joint_limit(servo_id)
    zero = config.get_zero_position(servo_id)
    angle = (position - zero) / config.POS_PER_DEG
    return max(lo, min(hi, angle))


class Servo:
    """单个舵机的操作封装"""

    def __init__(self, servo_id, protocol):
        """
        Args:
            servo_id: 舵机 ID (1~6)
            protocol: SerialProtocol 实例
        """
        self.id = servo_id
        self._proto = protocol

    # ---- 基本信息 ----

    def ping(self):
        """Ping 舵机，在线返回 (model, fw_version)，离线返回 None"""
        return self._proto.ping(self.id)

    # ---- 力矩 ----

    def enable_torque(self):
        """使能力矩（上电）"""
        return self._proto.write_register(
            self.id, config.ADDR_TORQUE_ENABLE, 1, size=1
        )

    def disable_torque(self):
        """解除力矩（卸力，可自由转动）"""
        return self._proto.write_register(
            self.id, config.ADDR_TORQUE_ENABLE, 0, size=1
        )

    # ---- 角度读写 ----

    def read_angle(self):
        """读取当前角度 (相对零位, -90° ~ +90°)，读取失败返回 None"""
        pos = self._proto.read_register(
            self.id, config.ADDR_PRESENT_POSITION, length=2
        )
        if pos is None:
            return None
        return _position_to_angle(pos, self.id)

    def read_position_raw(self):
        """读取原始位置值 (0~4095)"""
        return self._proto.read_register(
            self.id, config.ADDR_PRESENT_POSITION, length=2
        )

    def write_angle(self, angle, speed=None):
        """设置目标角度 (相对零位, -90° ~ +90°)，可选速度"""
        if speed is not None:
            self._proto.write_register(
                self.id, config.ADDR_GOAL_SPEED, speed, size=2
            )
        pos = _angle_to_position(angle, self.id)
        return self._proto.write_register(
            self.id, config.ADDR_GOAL_POSITION, pos, size=2
        )

    # ---- 状态 ----

    def is_moving(self):
        """是否正在运动"""
        val = self._proto.read_register(self.id, config.ADDR_MOVING, length=1)
        return val == 1 if val is not None else None

    def read_speed(self):
        """读取当前转速"""
        return self._proto.read_register(
            self.id, config.ADDR_PRESENT_SPEED, length=2
        )

    def read_load(self):
        """读取当前负载"""
        return self._proto.read_register(
            self.id, config.ADDR_PRESENT_LOAD, length=2
        )
