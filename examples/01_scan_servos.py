# -*- coding: utf-8 -*-
"""
示例 01 — 扫描总线舵机
确认 6 个舵机都在线，打印型号和固件版本。
"""

import sys
import os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from robot_arm import SerialProtocol, Servo, config


def main():
    proto = SerialProtocol()
    try:
        proto.open()
        print("\n=== 扫描舵机总线 ===")
        online = 0
        for sid in config.SERVO_IDS:
            servo = Servo(sid, proto)
            # Try ping first, then fall back to read_position
            result = servo.ping()
            if result:
                print(f"  [OK] ID {sid}: model={result[0]}, fw={result[1]}")
                online += 1
            else:
                # Ping may not be supported; try reading position instead
                pos = servo.read_position_raw()
                if pos is not None:
                    angle = servo.read_angle()
                    print(f"  [OK] ID {sid}: pos={pos}, angle={angle:.1f} deg" if angle else f"  [OK] ID {sid}: pos={pos}")
                    online += 1
                else:
                    print(f"  [--] ID {sid}: no response")
        print(f"\n结果: {online}/{len(config.SERVO_IDS)} 在线")
    finally:
        proto.close()


if __name__ == "__main__":
    main()
