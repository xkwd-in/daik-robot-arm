# -*- coding: utf-8 -*-
"""
机械臂全局配置
"""

import json
import os

# --- 串口 ---
PORT = "COM6"
BAUDRATE = 1000000
TIMEOUT = 0.1

# --- 舵机 ---
SERVO_IDS = [1, 2, 3, 4, 5, 6]  # 6 个关节舵机 ID

# STS3215 舵机参数
POSITION_MIN = 0       # 0° (原始值)
POSITION_MAX = 4095    # 360° (原始值)
POSITION_CENTER = 2048 # 180° — 中位

# Default software limits (degrees, relative to zero)
ANGLE_MIN = -90.0
ANGLE_MAX = 90.0

# Default per-joint angle limits (overridden by joint_limits.json if exists)
_DEFAULT_JOINT_LIMITS = {
    1: (-90.0, 90.0),
    2: (-90.0, 90.0),
    3: (-90.0, 90.0),
    4: (-90.0, 90.0),
    5: (-90.0, 90.0),
    6: (0.0, 90.0),     # gripper: 0=closed
}

# Load calibrated limits from file
_limits_file = os.path.join(os.path.dirname(__file__), "joint_limits.json")
_joint_limits = dict(_DEFAULT_JOINT_LIMITS)
if os.path.exists(_limits_file):
    with open(_limits_file, "r", encoding="utf-8") as _f:
        _data = json.load(_f)
    for _k, _v in _data.items():
        _sid = int(_k)
        _lo = _v.get("min")
        _hi = _v.get("max")
        if _lo is not None and _hi is not None:
            _joint_limits[_sid] = (_lo, _hi)


def get_joint_limit(servo_id):
    """Return (min_angle, max_angle) for a given servo"""
    return _joint_limits.get(servo_id, (ANGLE_MIN, ANGLE_MAX))

# 角度 → 位置值映射参数（当前舵机工作区间）
POS_LOW = 1024   # 对应 -90°
POS_HIGH = 3072  # 对应 +90°

# POS_STEP: 每度对应多少个位置值
POS_PER_DEG = (POS_HIGH - POS_LOW) / (ANGLE_MAX - ANGLE_MIN)

# --- 零位标定数据 ---
_zero_file = os.path.join(os.path.dirname(__file__), "zero_offset.json")
_zero_raw = {}
if os.path.exists(_zero_file):
    with open(_zero_file, "r", encoding="utf-8") as _f:
        _data = json.load(_f)
    for _sid, _v in _data.items():
        if _v.get("raw_position") is not None:
            _zero_raw[int(_sid)] = _v["raw_position"]


def get_zero_position(servo_id):
    """返回舵机的零位原始位置值，未标定时默认 2048"""
    return _zero_raw.get(servo_id, 2048)


def is_calibrated():
    """是否已完成零位标定"""
    return len(_zero_raw) > 0

# --- 控制参数 ---
DEFAULT_SPEED = 500   # 默认转速 (0~1023, 约 0.1rpm/单位)
DEFAULT_ACC = 50      # 默认加速度

# --- 寄存器地址 (SCS Protocol) ---
ADDR_TORQUE_ENABLE = 40
ADDR_GOAL_POSITION = 42
ADDR_GOAL_SPEED = 46      # 目标转速（非零时启用速度闭环）
ADDR_PRESENT_POSITION = 56
ADDR_PRESENT_SPEED = 58
ADDR_PRESENT_LOAD = 60
ADDR_MOVING = 66          # 是否运动中：0=停止, 1=运动

# --- 指令 ---
INST_PING = 1
INST_READ = 2
INST_WRITE = 3
INST_SYNC_WRITE = 0x83    # 同步写入（多舵机同时生效）

# --- 通信状态码 ---
COMM_SUCCESS = 0
COMM_RX_TIMEOUT = -6
COMM_RX_CORRUPT = -7
