# -*- coding: utf-8 -*-
"""
底层串口协议 — SCS 总线数据包的发送与接收
参考: FeeTech SCServo 通讯协议 (0xFF 0xFF 头 + 校验和)
"""

import serial
import time
from . import config


class SerialProtocol:
    """SCS 串行总线协议处理"""

    def __init__(self, port=None, baudrate=None, timeout=None):
        self.port_name = port or config.PORT
        self.baudrate = baudrate or config.BAUDRATE
        self.timeout = timeout or config.TIMEOUT
        self.serial_port = None

    # ---- 连接管理 ----

    def open(self):
        self.serial_port = serial.Serial(
            self.port_name,
            baudrate=self.baudrate,
            timeout=self.timeout,
        )
        time.sleep(0.1)
        self.serial_port.reset_input_buffer()
        print(f"[SerialProtocol] 已打开 {self.port_name} @ {self.baudrate} bps")
        return True

    def close(self):
        if self.serial_port and self.serial_port.is_open:
            self.serial_port.close()
            print(f"[SerialProtocol] 已关闭 {self.port_name}")

    def is_open(self):
        return self.serial_port is not None and self.serial_port.is_open

    # ---- 数据包构建 ----

    @staticmethod
    def _checksum(data):
        """计算 SCS 校验和：对 byte 列表求和取反的低 8 位"""
        return (~sum(data)) & 0xFF

    def build_packet(self, servo_id, instruction, parameters=None):
        """构建 SCS 数据包，返回 bytes"""
        if parameters is None:
            parameters = []
        length = len(parameters) + 2
        core = [servo_id, length, instruction] + parameters
        chk = self._checksum(core)
        return bytes([0xFF, 0xFF] + core + [chk])

    # ---- 发送与接收 ----

    def send(self, servo_id, instruction, parameters=None):
        """发送指令包（不等待应答）"""
        if not self.is_open():
            return False
        packet = self.build_packet(servo_id, instruction, parameters)
        try:
            self.serial_port.reset_input_buffer()
            self.serial_port.write(packet)
            self.serial_port.flush()
            return True
        except Exception:
            return False

    def send_and_recv(self, servo_id, instruction, parameters=None):
        """发送指令并读取应答，返回 (status, error, data)"""
        if not self.send(servo_id, instruction, parameters):
            return config.COMM_RX_TIMEOUT, 0, []

        start = time.time()
        packet = []
        while (time.time() - start) < self.timeout:
            if self.serial_port.in_waiting > 0:
                b = self.serial_port.read(1)
                if not b:
                    continue
                b = b[0]

                # 跳过起始前的干扰字节
                if not packet and b != 0xFF:
                    continue

                packet.append(b)

                # 遇到连续 0xFF 0xFF 时复位（新的包头）
                if len(packet) >= 2 and packet[-2:] == [0xFF, 0xFF]:
                    if len(packet) > 2:
                        packet = [0xFF, 0xFF]
                    continue

                # 包完整时校验
                if len(packet) > 4:
                    pkt_len = packet[3]  # 长度字段
                    if len(packet) == pkt_len + 4:
                        core = packet[2:-1]  # [ID, LEN, ...]
                        calc = self._checksum(core)
                        if calc == packet[-1]:
                            return config.COMM_SUCCESS, packet[4], packet[5:-1]
                        return config.COMM_RX_CORRUPT, 0, []

        return config.COMM_RX_TIMEOUT, 0, []

    # ---- 寄存器读写便捷方法 ----

    def read_register(self, servo_id, address, length=2):
        """读寄存器，返回 int 值或 None"""
        status, error, data = self.send_and_recv(
            servo_id, config.INST_READ, [address, length]
        )
        if status != config.COMM_SUCCESS:
            return None
        # Note: error byte carries status flags (overload, voltage, etc.)
        # that don't invalidate the data. We return the value regardless.
        if len(data) < length:
            return None
        # 小端序
        value = 0
        for i in range(length):
            value |= data[i] << (8 * i)
        return value

    def write_register(self, servo_id, address, value, size=2):
        """写寄存器，返回 bool"""
        params = [address]
        if size == 1:
            params.append(value & 0xFF)
        elif size == 2:
            params.extend([value & 0xFF, (value >> 8) & 0xFF])
        else:
            return False
        return self.send(servo_id, config.INST_WRITE, params)

    def ping(self, servo_id):
        """Ping 舵机，返回 (model_number, firmware_version) 或 None"""
        status, error, data = self.send_and_recv(servo_id, config.INST_PING)
        if status == config.COMM_SUCCESS and len(data) >= 3:
            model = data[0] | (data[1] << 8)
            fw = data[2]
            return model, fw
        return None
