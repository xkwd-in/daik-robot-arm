# -*- coding: utf-8 -*-
"""
示例 03 — GUI 滑块控制六轴
每个滑块拖动时实时写入对应舵机（逐个写，适合手动调试）。
"""

import sys
import os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import matplotlib.pyplot as plt
from matplotlib.widgets import Slider
from robot_arm import ArmRobot, config


def main():
    arm = ArmRobot()
    try:
        print("使能力矩...")
        arm.enable_all()

        fig, ax = plt.subplots(figsize=(8, 6))
        plt.subplots_adjust(left=0.25, bottom=0.1, top=0.9, right=0.9)
        fig.suptitle("六轴舵机角度控制 — COM3")

        sliders = []
        for i, sid in enumerate(config.SERVO_IDS):
            ax_slider = plt.axes([0.25, 0.1 + i * 0.12, 0.65, 0.05])
            slider = Slider(
                ax=ax_slider,
                label=f"关节 {sid}",
                valmin=-90,
                valmax=90,
                valinit=0,
                valfmt="%0.1f°",
            )
            sliders.append(slider)

        def make_callback(sid):
            def callback(val):
                arm.servos[sid].write_angle(val)
            return callback

        for i, sid in enumerate(config.SERVO_IDS):
            sliders[i].on_changed(make_callback(sid))

        print("GUI 已就绪。拖动滑块控制舵机，关闭窗口退出。")
        plt.show()

    except KeyboardInterrupt:
        print("\n已停止。")
    finally:
        arm.close()


if __name__ == "__main__":
    main()
