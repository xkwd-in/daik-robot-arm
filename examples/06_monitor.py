# Real-time 6-axis servo monitor - terminal dashboard
import sys, os, time
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from robot_arm import ArmRobot, config


def clear_screen():
    os.system("cls" if os.name == "nt" else "clear")


def main():
    arm = ArmRobot()
    print("Starting monitor...")
    time.sleep(0.5)

    try:
        while True:
            clear_screen()
            print("=== 6-Axis Servo Monitor ===\n")
            print(f"{'Joint':>6}  {'Angle':>9}  {'RawPos':>7}  {'Moving':>7}  {'Load':>7}")
            print("-" * 50)

            for sid in config.SERVO_IDS:
                s = arm.servos[sid]
                angle = s.read_angle()
                raw = s.read_position_raw()
                moving = s.is_moving()
                load = s.read_load()

                a_str = f"{angle:+7.2f} deg" if angle is not None else "   ERR"
                r_str = f"{raw:>5}" if raw is not None else "  ERR"
                m_str = "YES" if moving else "no"
                l_str = f"{load}" if load is not None else "ERR"

                print(f"   J{sid}   {a_str}   {r_str}    {m_str:>5}    {l_str:>5}")

            calib = "Calibrated" if config.is_calibrated() else "UNCALIBRATED"
            print(f"\n{calib} | Ctrl+C to exit")
            time.sleep(0.2)

    except KeyboardInterrupt:
        print("\n\nMonitor stopped.")
    finally:
        arm.close()


if __name__ == "__main__":
    main()
