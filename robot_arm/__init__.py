# -*- coding: utf-8 -*-
"""
robot_arm — 六轴机械臂控制库

用法:
    from robot_arm import SerialProtocol, Servo, SyncController, ArmRobot

    # 1. 建立通信
    proto = SerialProtocol()
    proto.open()

    # 2. 控制单轴
    s1 = Servo(1, proto)
    s1.enable_torque()
    s1.write_angle(30.0)

    # 3. 六轴同步
    sync = SyncController(proto)
    sync.sync_write_angles([10, -15, 30, 0, 45, -20])

    # 4. 完整机械臂
    arm = ArmRobot(proto)
    arm.enable_all()
    arm.go_home()
    arm.move_joints([10, 20, -15, 30, 0, 45])
    arm.close()
"""

from .serial_protocol import SerialProtocol
from .servo import Servo
from .sync_controller import SyncController
from .kinematics import (
    forward_kinematics,
    dh_matrix,
    IKSolver,
    TrajectoryPlanner,
    DH_PARAMS_PLACEHOLDER,
)
from . import config


class ArmRobot:
    """完整机械臂 — 封装初始化和常用组合动作"""

    def __init__(self, protocol=None, port=None):
        if protocol is not None:
            self._proto = protocol
            self._own_proto = False
        else:
            self._proto = SerialProtocol(port=port)
            self._proto.open()
            self._own_proto = True

        self.servos = {sid: Servo(sid, self._proto) for sid in config.SERVO_IDS}
        self.sync = SyncController(self._proto)

    # ---- 生命周期 ----

    def close(self):
        """Disable torque then close serial port"""
        try:
            self.disable_all()
        except Exception:
            pass
        if self._own_proto:
            self._proto.close()

    # ---- 批量操作 ----

    def enable_all(self):
        """使能所有舵机力矩"""
        for sid in config.SERVO_IDS:
            self.servos[sid].enable_torque()

    def disable_all(self):
        """解除所有舵机力矩"""
        for sid in config.SERVO_IDS:
            self.servos[sid].disable_torque()

    def go_home(self):
        """所有轴回零"""
        return self.sync.go_home()

    def move_joints(self, angles):
        """
        同步移动到目标关节角。

        Args:
            angles: list of 6 floats [j1..j6] (度) 或 dict {id: angle}
        """
        return self.sync.sync_write_angles(angles)

    def read_all_angles(self):
        """读取所有关节当前角度，返回 dict {id: angle}"""
        return {sid: self.servos[sid].read_angle() for sid in config.SERVO_IDS}

    def read_all_angles_list(self):
        """读取所有关节当前角度，返回 list [j1..j6]"""
        return [self.servos[sid].read_angle() for sid in config.SERVO_IDS]

    def scan(self):
        """扫描总线上所有舵机，返回在线舵机列表"""
        online = []
        for sid in config.SERVO_IDS:
            result = self.servos[sid].ping()
            if result:
                model, fw = result
                print(f"  ID {sid}: 型号={model}, 固件={fw}")
                online.append(sid)
            else:
                print(f"  ID {sid}: 无应答")
        return online
