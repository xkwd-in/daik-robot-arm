# -*- coding: utf-8 -*-
"""
示例 04 — 六轴同步联动
使用 SYNC_WRITE 实现 6 轴同时运动，并演示轨迹插值。
"""

import sys
import os
import time
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from robot_arm import ArmRobot


def demo_sync_move(arm):
    """演示：六轴同步移动到一组目标"""
    print("\n--- 同步移动到目标角度 ---")
    targets = [20, -15, 30, -10, 45, -25]
    print(f"  目标: {[f'{a:+.1f}°' for a in targets]}")
    arm.move_joints(targets)
    time.sleep(1.5)

    print("  回零...")
    arm.go_home()
    time.sleep(1.5)


def demo_trajectory(arm):
    """演示：轨迹插值（100 步平滑过渡）"""
    print("\n--- 轨迹插值 ---")
    start = [0, 0, 0, 0, 0, 0]
    end = [30, -20, 45, 15, -15, 30]
    print(f"  起点: {start}")
    print(f"  终点: {end}")
    print("  执行中...")

    arm.sync.execute_trajectory([start, end], duration_ms=2000, steps=100)
    time.sleep(0.5)

    print("  回零...")
    arm.sync.execute_trajectory([end, start], duration_ms=1000, steps=50)
    time.sleep(0.5)


def demo_wave(arm):
    """演示：波浪动作"""
    print("\n--- 波浪动作 ---")
    poses = [
        [0, 0, 0, 0, 0, 0],
        [0, -30, 0, 30, 0, -30],
        [0, 30, 0, -30, 0, 30],
        [0, 0, 0, 0, 0, 0],
    ]
    for i in range(len(poses) - 1):
        arm.sync.execute_trajectory(
            [poses[i], poses[i + 1]],
            duration_ms=800,
            steps=40,
        )


def main():
    arm = ArmRobot()
    try:
        print("使能力矩...")
        arm.enable_all()
        time.sleep(0.3)

        # 先回零
        print("回零...")
        arm.go_home()
        time.sleep(1)

        demo_sync_move(arm)
        demo_trajectory(arm)
        demo_wave(arm)

        print("\n=== 演示结束 ===")
    except KeyboardInterrupt:
        print("\n已停止。")
    finally:
        print("回零并关闭...")
        arm.go_home()
        time.sleep(1)
        arm.close()


if __name__ == "__main__":
    main()
