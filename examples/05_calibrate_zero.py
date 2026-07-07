# -*- coding: utf-8 -*-
"""
示例 05 — 零位标定
机械臂摆好目标零位姿态后运行此脚本，
读取 6 个舵机的原始位置值 + 当前角度，存入 robot_arm/zero_offset.json。
"""

import sys
import os
import json
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from robot_arm import SerialProtocol, Servo, config

OUTPUT_FILE = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
    "robot_arm", "zero_offset.json"
)


def main():
    proto = SerialProtocol()
    try:
        proto.open()
        print("\n=== 零位标定 ===\n")
        print("请确认机械臂已摆好零位姿态，然后按 Enter 继续...")
        input()

        offsets = {}
        print(f"{'关节':>6}  {'原始位置':>10}  {'当前角度':>10}")
        print("-" * 38)

        for sid in config.SERVO_IDS:
            servo = Servo(sid, proto)
            raw = servo.read_position_raw()
            angle = servo.read_angle()

            if raw is not None:
                offsets[str(sid)] = {
                    "raw_position": raw,
                    "angle": round(angle, 2) if angle is not None else None,
                }
                print(f"   J{sid}   {raw:>8}     {angle:>+8.2f}°" if angle else f"   J{sid}   {raw:>8}     {'ERR':>8}")
            else:
                print(f"   J{sid}   {'读取失败':>10}")
                offsets[str(sid)] = {"raw_position": None, "angle": None}

        with open(OUTPUT_FILE, "w", encoding="utf-8") as f:
            json.dump(offsets, f, indent=2, ensure_ascii=False)

        print(f"\n零位数据已保存到: {OUTPUT_FILE}")
        print("\n内容预览:")
        for sid in sorted(offsets, key=int):
            d = offsets[sid]
            print(f"  关节 {sid}: pos={d['raw_position']}, angle={d['angle']}°")

        print("\n标定完成！后续运动学将以这些值为零位基准。")

    finally:
        proto.close()


if __name__ == "__main__":
    main()
