# DM-MC02 ROS 2 实机接口

首次联调请按 [从串口配置开始的详细操作指南](../../REAL_HARDWARE_STEP_BY_STEP.zh-CN.md) 执行；本文用于查询驱动参数和行为。

适配 2026-11-18 检查的 S27_TEST_2 固件，USB 协议 v2。所有改动位于 ROS 工程，不需要修改固件。

## 当前配置与标定

默认 `allow_enable=0`，只收遥测。当前固件电机侧限位：

| 轴 | 最小值 rad | 最大值 rad |
|---|---:|---:|
| J1 | -2.96054893 | 2.81695309 |
| J2 | -2.05669899 | 0 |
| J3 | -3.00016972 | 0 |
| J4 | 0 | 3.00567291 |
| J5 | -0.904316101 | 1.40938965 |
| J6 | -0.650879326 | 2.32408657 |
| Grip | -12 | 0 |

唯一的实机标定配置入口是 [dm_mc02.ros2_control.xacro](urdf/dm_mc02.ros2_control.xacro)：

- 机械臂：`q_ros = sign * (q_mcu - offset)`，逆变换为 `q_mcu = sign * q_ros + offset`，sign 只能是 ±1。测量每轴方向、URDF 零位对应电机读数，填入 sign/offset 后设置 `mapping_calibrated=1`。现有 1/0 是占位值，不是标定结果。特别是 J2/J3，目前电机负角度范围与 URDF 正角度范围不一致；不能仅凭这一点断言真实零位/方向。
- 固件 J2/J3 的 zero_position 是固件动作目标，不自动等于 URDF offset。
- 夹爪：分别实测 `grip_mcu_closed/open` 与对应 `grip_q_close_m/open_m`，完成后设置 `grip_calibrated=1`。固件 -12 是硬限位，不是实测闭合端点。默认两个电机端点均为 0 占位；标为已标定但端点重合会报错。
- `grip_command_enable=0` 时，7 路命令中的夹爪始终保持使能首帧的原始反馈角度。不会把负角度夹到 0，也不会因 arm_controller 激活而开爪。
- 未标定夹爪的 joint8 位置/速度/力不伪造为米制值，而是 NaN。日志 `grip_motor` 提供原始电机角度（rad），供标定使用。
- `allow_enable=1` 要求机械臂映射已标定；夹爪控制器要求夹爪已标定且允许命令。机械臂直接轨迹控制可保持未标定夹爪，但 MoveIt 要求夹爪反馈也完成标定，以获得完整的碰撞模型状态。

修改配置后重新构建并 source 工作区。硬件限位只可收紧，不能超出上述固件范围。

## 构建与只读检查（Ubuntu / ROS 2 Humble）

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-up-to dm_mc02_hardware
source install/setup.bash
colcon test --packages-select dm_mc02_hardware --event-handlers console_direct+
colcon test-result --verbose

ros2 launch dm_mc02_hardware bringup.launch.xml allow_enable:=0 device:=/dev/ttyACM0
ros2 topic echo /joint_states
```

XML 入口转发到 `bringup.launch.py`。默认只启动 joint_state_broadcaster，轨迹控制器需要手动启动。日志包含模式、状态、故障位、反馈有效位、ACK 错误码、夹爪原始角度和停止原因。

## 使能与轨迹

完成标定配置后，停止只读节点并重新启动：

```bash
ros2 launch dm_mc02_hardware bringup.launch.xml allow_enable:=1
ros2 run controller_manager spawner arm_controller
# 仅在已标定并设置 grip_command_enable=1 时：
ros2 run controller_manager spawner gripper_controller
```

当前固件左上右上进入 PC 模式（arm_mode=10）；电机就绪且七路反馈新鲜时，插件发送 ENABLE，核对 ACK 序号，再等待 MCU 的 ENABLED/RUNNING 状态和故障清零。首帧原样使用七路反馈微弧度值。随后每 20 ms 发 POSITION_CMD，有效期 100 ms。

只有被控制器 claim 的关节接受命令；未 claim 的关节保持最近目标。非法数值、越限或单帧超过 0.01 rad 均撤销 PC 控制。read() 仅接收反馈，write() 在控制器更新后生成命令，避免发送上一控制周期的命令。

## MoveIt 实机入口

```bash
ros2 launch dm_mc02_hardware bringup.launch.xml allow_enable:=1 use_moveit:=true use_rviz:=true
ros2 run controller_manager spawner arm_controller
```

该入口给 robot_state_publisher、ros2_control、move_group 和 RViz 传入同一份实机模型：

- 关节位置范围取 URDF 与映射后电机限位的交集；空交集会报错，提示检查 sign/offset。
- 机械臂规划最大速度 0.4 rad/s、加速度 0.8 rad/s²，低于固件的 0.5 rad/s、1 rad/s²；默认缩放 10%。单帧步长保护仍生效。
- 夹爪规划速度/加速度按实测传动比例转换，分别不超过 0.001 m/s、0.002 m/s²。
- 超出当前实机范围的 SRDF 预设姿态自动移除；范围合法仍不代表已经实机验证。
- 未允许夹爪命令时，MoveIt 不注册 gripper_controller。
- 原 `test_robot_moveit_config move_group.launch.py` 使用独立的通用配置。实机请使用上面的统一入口，不再同时启动原入口。

固件已有速度/加速度插补；POSITION_CMD 的 velocity_limit 字段为 0 表示使用固件默认上限，并非保留字段。跟踪误差和实际运动响应仍需低速实机验证。

## 故障与恢复

通信中断、反馈失效、模式切换、MCU 失能/故障、命令被拒绝都会 STOP 并锁存在插件 FAULT。反馈恢复不会自动重放轨迹。插件保留遥测和控制器切换能力；需要停用全部已激活的命令控制器：

```bash
ros2 control set_controller_state arm_controller inactive
# 若 gripper_controller 也在 active，必须一并设为 inactive。
# 排除故障，确认 PC 模式、反馈和当前姿态正常后：
ros2 control set_controller_state arm_controller active
```

重新激活后允许通过 ENABLE 清除历史通信/反馈故障；当前 CAN 故障或电机使能失败仍阻止使能。ACK 拒绝或 200 ms 握手超时再次锁存，不能无限自动重试。USB 设备断开导致串口失效时，需要重新启动硬件节点。

STATE 超过 100 ms，或某轴反馈年龄加上主机等待时间超过 100 ms，该轴状态变为 NaN，控制停止。STOP 仅撤销 PC 控制；留在 PC 模式时固件执行保持，切换到遥控/无力模式的行为由现有固件决定。

故障位：0x0001 链路超时、0x0002 USB 会话断开、0x0004 RX 溢出、0x0008 反馈丢失、0x0010 CAN 总线、0x0020 电机使能失败。ACK 错误码见 `arm_protocol.hpp`。

## 离线测试与验证边界

不需要 ROS 的测试（要求 Python 3 和支持 C++17 的 g++）：

```bash
python src/dm_mc02_hardware/test/run_standalone.py
# 可选：读取实际固件编解码源文件进行双向互操作检查，不改动固件：
python src/dm_mc02_hardware/test/run_standalone.py --firmware ../S27_TEST_2
```

包含链路状态机、夹爪保持、ACK 批处理、超时/故障恢复、异常命令、映射与实机模型测试。原 GoogleTest 协议测试的字符字面量和索引错误已修正，仍由 colcon 测试运行。

本次在 Windows 上完成原生离线测试和与当前固件编解码的互操作测试。当前环境没有 ROS 2/WSL，完整 ament 构建、实际 launch、控制器生命周期和实机动作尚未验证。标定数据需要实测，不能用默认值替代。

