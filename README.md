# ros_test_8 — 机械臂 ROS 2 上位机工作空间

六轴机械臂的上位机工程：URDF 模型 → MoveIt2 规划 → ros2_control 控制器 → 三选一的执行后端（真机 USB 串口 / MuJoCo 物理仿真 / 模拟硬件）。

> **这是 `ros/ros_test_8` 分支。** 本仓库另一条线是 `main` 分支的 STM32H723 电控工程（CAN 电机、丝杆夹爪、遥控、图传），两边是独立历史、互不依赖。两条线的关系见 `main` 分支根目录的 `ROS2.md`。

**当前状态**：离线协议解析、状态机和模型测试已在 Windows 上跑通；完整的 Ubuntu 构建、ROS 启动和实机动作，需要按 [实机操作指南](REAL_HARDWARE_STEP_BY_STEP.zh-CN.md) 逐阶段验证。本文的启动命令是设计意图，不代表已实机验收。

## 包结构

```
src/
├── test_robot/                 机器人本体：URDF + 34 个 STL 网格（17 视觉 + 17 碰撞）+ RViz 配置
├── test_robot_interfaces/      自定义消息（PoseCommand.msg）
├── test_robot_moveit_config/   MoveIt2 配置：SRDF、运动学、OMPL、Pilz、控制器
├── test_robot_commander_cpp/   C++ 命令节点，调用 MoveIt 发目标位姿
├── test_robot_bringup/         模拟硬件演示的启动包
├── test_robot_mujoco/          MuJoCo 物理仿真后端（ros2_control 插件）
└── dm_mc02_hardware/           真机后端：ros2_control 硬件接口，USB 串口连 STM32 固件
```

执行后端的替换关系：

```
MoveIt2 规划 → ros2_control 控制器 ─┬→ dm_mc02_hardware → USB → 固件 → 实机
                                    ├→ test_robot_mujoco → MuJoCo 物理计算
                                    └→ GenericSystem     → 模拟硬件（纯演示）
```

## 环境与构建

Ubuntu 22.04 + ROS 2 Humble。依赖安装、串口权限（`dialout` 组）、设备路径（默认 `/dev/ttyACM0`）见 [UBUNTU_SETUP.md](UBUNTU_SETUP.md)。

```bash
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y --rosdistro humble
colcon build --symlink-install
source install/setup.bash          # 每开一个新终端都要 source
```

## 常用启动

| 目的 | 命令 |
|---|---|
| 只看模型，不需要硬件 | `ros2 launch test_robot display.launch.xml` |
| MuJoCo 物理仿真 | `ros2 launch test_robot_mujoco mujoco.launch.py` |
| MoveIt 规划演示 | `ros2 launch test_robot_moveit_config demo.launch.py` |
| **真机：只读遥测**（不发使能） | `ros2 launch dm_mc02_hardware bringup.launch.xml allow_enable:=0` |
| 真机：允许使能 + MoveIt + RViz | `ros2 launch dm_mc02_hardware bringup.launch.xml allow_enable:=1 use_moveit:=true use_rviz:=true` |

> 真机第一次务必先 `allow_enable:=0`：电机不使能，只回传关节角，确认方向、零位、限位无误后再改成 `1`。

## 文档

| 文档 | 用途 |
|---|---|
| [REAL_HARDWARE_STEP_BY_STEP.zh-CN.md](REAL_HARDWARE_STEP_BY_STEP.zh-CN.md) | 实机操作指南：串口配置 → 只读遥测 → 标定 → 首次使能 → 小步验证 → MoveIt，16 个阶段 |
| [UBUNTU_SETUP.md](UBUNTU_SETUP.md) | 环境搭建与联调手册：安全边界、串口权限、控制器启动、故障恢复 |
| [src/CODE_GUIDE.zh-CN.md](src/CODE_GUIDE.zh-CN.md) | 代码阅读指南：各包职责、调用链、改哪个文件要重新构建什么 |
| [src/dm_mc02_hardware/README.md](src/dm_mc02_hardware/README.md) | 串口协议与硬件接口参数 |
| [src/test_robot_mujoco/README.md](src/test_robot_mujoco/README.md) | MuJoCo 后端说明 |

## 说明

- `build/`、`install/`、`log/` 是 colcon 生成物，不入库，**克隆后必须自己构建**，否则 `ros2 launch` 找不到包。
- 模型网格占仓库体积的主要部分（`meshes/Link2.STL` 约 32 MB），克隆时请留意网络。
- 改 C++ 后必须重新 `colcon build`；`--symlink-install` 只对 Python、launch 和配置文件即时生效。
