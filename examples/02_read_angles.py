# -*- coding: utf-8 -*-
"""
示例 02 — 读取所有舵机角度
实时刷新 6 个关节的当前角度。
"""

import sys
import os
import time
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from robot_arm import ArmRobot


def main():
    arm = ArmRobot()
    print("\n=== 实时读取关节角度 (Ctrl+C 停止) ===")
    try:
        while True:
            angles = arm.read_all_angles()
            line = "  ".join(
                f"J{sid}: {angles[sid]:+6.1f}°" if angles[sid] is not None
                else f"J{sid}:   ERR"
                for sid in sorted(angles)
            )
            print(f"\r{line}", end="")
            time.sleep(0.1)
    except KeyboardInterrupt:
        print("\n\n已停止。")
    finally:
        arm.close()


if __name__ == "__main__":
    main()
