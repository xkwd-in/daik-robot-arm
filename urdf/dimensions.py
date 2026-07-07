"""
Robot Arm Dimensions — ALL PLACEHOLDER VALUES
=============================================
Replace each value with YOUR actual measurement (in METERS).
When you get a caliper or ruler, update these and regenerate URDF.
"""

# ── Link lengths (joint-axis to joint-axis) ──
DIMS = {
    # "x" = forward along link, "z" = up along link axis
    "base_height": 0.080,
    "upper_arm":   0.120,
    "forearm":     0.110,
    "wrist_offset": 0.040,
    "wrist_pitch_to_yaw": 0.040,
    "tool_length": 0.060,
}

# ── Joint limits (radians) ──
JOINT_LIMITS = {
    1: (-1.57, 1.57),   # base yaw     +-90 deg
    2: (-1.57, 1.57),   # shoulder     +-90 deg
    3: (-1.57, 1.57),   # elbow        +-90 deg
    4: (-1.57, 1.57),   # wrist roll   +-90 deg
    5: (-1.57, 1.57),   # wrist pitch  +-90 deg
    6: (-1.57, 1.57),   # wrist yaw    +-90 deg
}

# ── How to measure without calipers ──
MEASURE_HINTS = """
Quick measurement tricks:
  1. Ruler + phone camera: put ruler next to arm, take straight side photo.
     Measure pixel distances in any image viewer.
  2. String method: wrap string around joint centers, mark, measure with ruler.
  3. Count servo bracket holes: standard spacing ~4-5mm per hole.
  4. Look for model numbers on brackets/U-brackets — standard sizes exist.
"""
