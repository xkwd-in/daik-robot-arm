# daik-robot-arm

六轴机械臂 Python 控制库，PC 直连 STS3215 舵机总线，SCS 协议自实现。

## 硬件

| 项目 | 规格 |
|------|------|
| 舵机 | STS3215 × 6（FeeTech 总线串行舵机） |
| 通信 | USB 转 TTL（COM6, 1Mbps） |
| 协议 | SCS（0xFF 0xFF 帧头 + 校验和） |
| 角度 | ±90°（软件限位可标定） |

## 目录

```
├── robot_arm/          # Python 控制库
│   ├── __init__.py     # ArmRobot 高层 API
│   ├── config.py       # 全局配置 + 标定加载
│   ├── serial_protocol.py  # SCS 串口协议
│   ├── servo.py        # 单舵机操作
│   ├── sync_controller.py  # 六轴同步（SYNC_WRITE）
│   ├── kinematics.py   # FK/DH/轨迹规划
│   └── ik_solver.py    # IK 求解（ikpy v4 + URDF）
├── examples/           # 使用示例（寻找限位→IK→工作空间→绘画）
├── urdf/               # URDF + 尺寸测量
├── ros_ws/             # ROS 2 仿真（Gazebo + RViz）
└── rezero_j1.py        # 零位重标定工具
```

## 快速开始

```python
from robot_arm import ArmRobot

arm = ArmRobot()
arm.enable_all()
arm.move_joints([10, -15, 30, 0, 45, 20])  # 六轴同步运动
arm.close()
```

```bash
# 工作空间探索（无硬件，纯 IK）
python examples/12_workspace_map.py

# 机械臂绘画
python examples/13_draw.py
```

## 功能

- SCS 协议自实现（帧构建/收发/校验/超时）
- SYNC_WRITE 六轴同步（去波浪效应）
- ikpy v4 逆运动学（位置/位姿）
- 关节空间 线性+梯形 轨迹插值
- 零位/限位标定系统
- ROS 2 Gazebo 仿真
- 工作空间可达性分析

## 踩坑记录

### 握笔姿势策略
AI 生成绘画代码时倾向模拟人类书写姿态（倾斜握笔），但夹爪只能垂直夹持。机械臂绘画就是"横平竖直"——笔竖直向下，末端只做 XYZ 平移，用 `solve_position()` 别碰 `solve_full_pose()`。

### 夹爪抓取
金属平面夹爪抓圆形笔杆打滑，测试时用平板替代纸面导致摩擦系数完全不同。夹爪为块状物体设计，夹笔需要橡胶套/V 型槽适配器。**先验证硬件再写代码。**

### 通用教训
- 硬件先行验证：手动测夹爪能不能抓住目标，不要等代码写完才发现物理上不可行
- 约束条件前置：joint6 末端姿态是锁死的，在约束内设计策略
- 测试条件诚实：没有纸就别测绘画，平板替代 = 换了个物理环境

## Wiki

完整架构文档见 [Darcy's Wiki](https://github.com/xkwd-in/darcy-wiki)：
- `entities/daik-robot-arm.md` — 项目总览
- `concepts/robot-arm-codebase.md` — 代码架构详解
- `concepts/sts3215-scs-servo-control.md` — 舵机协议
- `concepts/urdf-ros-simulation.md` — URDF 与仿真
