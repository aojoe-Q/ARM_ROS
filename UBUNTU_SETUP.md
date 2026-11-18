# DM-MC02 实机联调工作手册（Ubuntu 22.04 / ROS 2 Humble）

> **2026-11-18 软件适配更新：下面保留的是旧版联调记录。当前启动、标定和故障恢复流程请使用 [dm_mc02_hardware/README.md](src/dm_mc02_hardware/README.md)。**
> ROS 已同步当前固件限位，修复负角度夹爪首帧保持、故障恢复、ACK 匹配、反馈失效处理及协议测试错误。新增 `mapping_calibrated` / `grip_calibrated` 标定状态；旧的夹爪 0～0.7 rad、旧 J2/J3 限位及下文“尚未修复”表格不再描述当前实现。实机 MoveIt 使用 `dm_mc02_hardware bringup.launch.xml use_moveit:=true` 统一入口。固件未由本次软件适配修改。

修订日期：2026-11-02。适用：`ros_test_8` + `S27_TEST_2`（USB 协议 v2）、DM-MC02 / STM32H723VGT6、J1–J6 与夹爪。
本次整理基于本地源码静态核对；**尚未据此完成 Ubuntu 编译、ROS 运行或实机验收**。

## 1. 使用顺序与当前待办

操作流程看本文；帧字段、故障位和固件行为看 [USB 协议说明](../S27_TEST_2/README_USB_COMMUNICATION.md)。
每阶段必须留下“通过/失败/未执行”和证据；失败先排查，不能跳过后续依赖的检查。

| 阶段 | 目标 | 进入下一阶段的条件 |
|---|---|---|
| 0 | 环境、构建、协议自测 | 构建和测试通过，版本记录完整 |
| 1 | 原始遥测、ROS 只读反馈 | 7 关节反馈新鲜，映射/设备对应正确 |
| 2 | 方向、零点、限位标定 | URDF 坐标与实物一致，安全区间明确 |
| 3 | 使能并保持当前姿态 | 门控、基准帧、撤防无异常动作 |
| 4 | 小步点动与停止 | 各关节方向正确，停止行为已验证 |
| 5 | 夹爪（可选） | 电机角度与直线位移独立标定 |
| 6 | MoveIt | 模型、关节限位、action 与真实反馈一致 |

### 1.1 开始工作前先处理的已知问题

这些是本次检查发现的代码/集成待办，文档修改没有修复代码。

| 优先级 | 位置/问题 | 完成标准 |
|---|---|---|
| P0：构建前 | `src/dm_mc02_hardware/test/test_protocol.cpp` 的 CRC 输入写成 `''1''` 等非法字符字面量；位置编码测试把索引 6 期望成 J6、并把索引 7 误当 Grip | 字面量改为合法 C++；J6 在 position[5]、Grip 在 position[6]，索引 7 已进入 velocity_limit；构建与测试通过 |
| P0：运动前 | sign/offset、夹爪端点、电机 P/V/T 量程尚需实测 | 按第 6、9 节记录；不把默认值视为校准结果 |
| P1：故障恢复 | MCU SYS_FAULT 需成功 ENABLE 清除，但 ROS try_arm 又拒绝 SYS_FAULT | 后续设计恢复流程；当前按第 11 节停机诊断/必要时复位 |
| P1：可观测性 | disarm reason 被丢弃，日志未输出 ACK 具体结果和反馈年龄 | 后续补诊断；当前用独占串口 monitor 分阶段采集 |
| P1：MoveIt | 默认 MoveIt Xacro 使用 FakeSystem；部分预设姿态可能超出实机范围 | 第 10 节核对模型、限位和 action；不要直接执行预设 home/pose |
| P2：通信健壮性 | 自动重连、严格 ACK 序号匹配、有界重试尚不完善 | 后续实现并做故障注入；当前重连后完整重新启动遥测 |

不要用 `BUILD_TESTING=OFF` 或放宽限位来代替解决 P0 问题。测试数量以实际测试结果为准，旧手册的“7 tests”不适用于当前文件。

## 2. 安全边界与状态速查

### 2.1 动作边界

- 机械臂需要防坠支撑；DT7 失联或 ARM_POWERLESS 可使机械臂失力下垂。
- 确认物理急停/断电方式及支撑效果。不要在未知供电规格下按旧文档假定“12V”。
- `allow_enable=0` 仅禁止这个 ROS 插件发 ENABLE；不会停止固件遥控或预设动作。
- USB STOP 只撤销 PC 控制。**留在 PC 模式**时固件执行保持逻辑；拨杆移出 PC 后执行对应的新模式，可能移动或失力。先查明拨杆模式，再做切换测试。
- 不使用“大幅越界轨迹”检验限位：控制器可能先执行合法部分，再触发限位。拒绝逻辑先离线测试。
- 当前 PC 模式与 ROS 插件不执行 J1/J2 碰撞区约束；规划预览不能代替未建模干涉的人工核对。

### 2.2 日志如何读

典型 ROS 日志：`State=3 mode=10 sys=2 tx=... rx=... crc_err=0 ack_err=0`。

| 字段 | 当前值的含义 |
|---|---|
| ROS `State` | 0 IDLE、1 WAIT_ENABLE_ACK、2 WAIT_BASELINE、3 ARMED、4 FAULT |
| MCU `sys` | 0 IDLE、1 ENABLED、2 RUNNING、3 FAULT |
| `mode` | 10=PC_CONTROL（DT7 左上+右上） |
| `rx` | ArmLink 已处理状态/ACK 的计数，不是严格的线上 STATE 帧率 |
| `crc_err / ack_err` | 应无持续增长；非零时保留日志并排查 |

“ARMED”不等于电机到位，“RUNNING”只表示有有效 PC 目标。无效反馈仍可能显示有限数值，不能只看 /joint_states 是否有数字。

| 时间/频率 | 当前实现 |
|---|---|
| ros2_control update_rate | 100 Hz |
| ROS POSITION_CMD | 间隔至少 20 ms，约 50 Hz |
| 每条命令 valid_time_ms | 默认 100 ms |
| ROS STATE 超时 | 默认 100 ms；配置加载时最大钳为 100 ms |
| 固件反馈有效位、ENABLE、运行 watchdog | 反馈年龄 ≤100 ms 才有效，完整掩码 0x7F |
| 固件链路超时 | 已使能后 300 ms 无接收活动 |
| MCU USB 任务/STATE | 目标 100 Hz，忙时会跳过发送 |
| 电机发送 | 每轮等待合计约 17 ms，单电机约 59 Hz 以下、各关节错开发送 |

## 3. Ubuntu 环境与工作空间

### 3.1 前置条件与安装

确认 Ubuntu 22.04 已安装 ROS 2 Humble，存在 `/opt/ros/humble/setup.bash`。
下列命令在 Ubuntu Bash 运行，不在 Windows PowerShell 运行：

~~~bash
source /opt/ros/humble/setup.bash
printenv ROS_DISTRO
sudo apt update
sudo apt install -y \
  build-essential python3-colcon-common-extensions python3-rosdep \
  python3-serial \
  ros-humble-ros2-control ros-humble-ros2-controllers \
  ros-humble-controller-manager ros-humble-joint-trajectory-controller \
  ros-humble-joint-state-broadcaster ros-humble-moveit \
  ros-humble-moveit-configs-utils ros-humble-xacro ros-humble-rviz2
~~~

预期 ROS_DISTRO=humble。若 ROS 尚未安装，先完成 Humble 官方 Ubuntu 安装流程，再继续。

### 3.2 迁移与首次构建

将两个目录分别复制到 `~/ros_test_8`、`~/S27_TEST_2`；保留源代码和文档，不沿用 Windows/其他机器生成的 build、install、log。已有 Ubuntu 工作空间先备份，不在这里执行删除命令。

~~~bash
source /opt/ros/humble/setup.bash
cd ~/ros_test_8
colcon list
# 仅系统首次使用时执行；已初始化则跳过这一行
sudo rosdep init
rosdep update
rosdep install --from-paths src --ignore-src -r -y --rosdistro humble
colcon build --packages-up-to dm_mc02_hardware --event-handlers console_direct+
source install/setup.bash
ros2 pkg prefix dm_mc02_hardware
ros2 pkg prefix test_robot
ros2 pkg prefix test_robot_moveit_config
~~~

使用 --packages-up-to 构建依赖，避免只构建硬件包却缺少运行时引用的机器人/MoveIt 包。
若 rosdep 因可选仿真包失败，单独记录并解决依赖范围，不能把 -r 继续执行当作成功。
首次构建遇到第 1.1 节测试源码问题时，应先修复该项并重跑。

后续每个新终端先执行：

~~~bash
source /opt/ros/humble/setup.bash
source ~/ros_test_8/install/setup.bash
~~~

修改 Xacro/YAML/launch 后重新构建硬件包并重启 bringup；仅改 launch 参数 allow_enable 不需要编辑文件或重新编译。

### 3.3 串口权限与稳定路径

~~~bash
sudo usermod -a -G dialout "$USER"
# 注销并重新登录后，在新终端验证
id -nG
lsusb
ls -l /dev/ttyACM*
udevadm info --name=/dev/ttyACM0 --attribute-walk
~~~

根据实际 VID/PID 创建规则；下例仅适用于确认设备为 0483:5740 的情况。多块同型号板还应加实际序列号匹配。

~~~bash
sudo tee /etc/udev/rules.d/99-dm-mc02.rules >/dev/null <<'EOF'
SUBSYSTEM=="tty", ATTRS{idVendor}=="0483", ATTRS{idProduct}=="5740", SYMLINK+="dm_mc02", GROUP="dialout", MODE="0660"
EOF
sudo udevadm control --reload-rules
sudo udevadm trigger
ls -l /dev/dm_mc02
~~~

需要时重新插拔 USB。把 `src/dm_mc02_hardware/urdf/test_robot_real.urdf.xacro` 的 device 改为该路径并重编译。
当前 bringup 没有 device launch 参数。以下 monitor 示例默认 /dev/ttyACM0，使用符号链接后须同步替换。

## 4. 阶段 0：离线检查

~~~bash
cd ~/ros_test_8
source install/setup.bash
colcon test --packages-select dm_mc02_hardware --event-handlers console_direct+
colcon test-result --verbose
ros2 launch dm_mc02_hardware bringup.launch.xml --show-args
~~~

应能看到 allow_enable（默认 0）与 use_rviz（默认 false）。
当前 C++ 文件有 5 个 TEST 定义，数量不是验收标准；以构建成功、用例实际运行且无失败为准。该套测试也不等于完成运动/故障状态机验证。

固件主机自测（无需接板，需 gcc）：

~~~bash
cd ~/S27_TEST_2
python3 tools/selftest/run_selftest.py
python3 tools/selftest/host_rate_check.py
python3 tools/arm_host_control.py --dry-run --joint 1=0.1
~~~

**通过条件**：命令退出成功，保存测试摘要及版本；没有通过记录时不填写“已通过”。
Keil 烧录使用重新编译的当前固件并记录产物；仅打开旧 hex 文件不能证明运行的固件与文档一致。

## 5. 阶段 1：原始遥测 → ROS 只读

### 5.1 先用 monitor 独立检查 MCU

同一 USB 串口只能由一个程序使用，monitor/control/ROS 不可并行打开。

~~~bash
cd ~/S27_TEST_2
python3 tools/arm_host_monitor.py --list
python3 tools/arm_host_monitor.py --port /dev/ttyACM0 \
  --verbose --duration 30 --csv telemetry_stage1.csv
~~~

检查 ready=yes、7 个反馈年龄持续 ≤100 ms、无 STALE/never、CRC 错误不增长；CSV 的 feedback_valid 应为 127（0x7F）。
STATE 频率应接近 100 Hz；明显偏低需记录实测值并排查，不能只看屏幕刷新率（默认 5 Hz）。

此步骤可在熟悉的遥控模式下小幅移动一个关节，确认对应原始反馈变化；程序不会下发运动，但遥控器仍可驱动。
程序退出后再启动 ROS。保存原始数据，尤其是后续标定所需的 MCU rad。

### 5.2 启动 ROS 遥测

终端 1：

~~~bash
cd ~/ros_test_8
source install/setup.bash
mkdir -p ~/arm_logs
ros2 launch dm_mc02_hardware bringup.launch.xml allow_enable:=0 \
  2>&1 | tee ~/arm_logs/stage1_bringup.log
~~~

终端 2（先 source）：

~~~bash
ros2 control list_controllers
ros2 control list_hardware_interfaces
ros2 topic info /joint_states --verbose
ros2 topic echo /joint_states --once
ros2 topic hz /joint_states
~~~

验收：

- joint_state_broadcaster active；arm_controller/gripper_controller 未 active。
- /joint_states 包含 joint1–joint6、joint8，按 name 对应 position，不依赖排列顺序。
- joint1–6 单位 rad；joint8 单位 m；夹爪 effort 当前仍沿用电机 N·m，不能当已换算夹持力。
- State=0，tx 不应因本插件运动指令增长，rx 持续增长。
- ROS 反馈与原始反馈经映射一致。原始有效性已由 monitor 单独确认；当前 ROS 日志不展示完整反馈年龄。

停止：终端 1 Ctrl+C，确认进程退出后再换用 monitor。
只读模式不保证机械臂静止，sys 也不能作为所有遥控动作的运行状态。

## 6. 阶段 2：标定与限位

标定时 allow_enable=0，用遥控器/机械基准确认姿态。ROS 正方向按 URDF joint axis 的右手规则确定，不统一假定“逆时针/向上”。

对每个 joint1–6：

1. 在机械参考位姿，记录该关节 MCU 原始角度 q_mcu_ref 和模型参考值 q_ros_ref。
2. 沿 ROS 正方向小幅移动；MCU 值增大取 sign=+1，减小取 sign=-1。
3. 计算 `offset = q_mcu_ref - sign*q_ros_ref`。
4. 更新 `src/dm_mc02_hardware/urdf/dm_mc02.ros2_control.xacro` 的 jointN_sign / jointN_offset。
5. 重新编译、重启只读遥测，在至少两个姿态验证位置和方向。

若只记录了已映射的 /joint_states，须先用**旧参数**还原：
`q_mcu = old_sign*q_ros + old_offset`。只有旧 sign=1、offset=0 时 ROS 值才等于 MCU 原始角度。

不要把固件 J3_zero_position=0.1 当成 ROS 标定结果。电机端点、装配零位和 URDF 零位要逐一对应。

| 关节 | 参考 MCU rad | 参考 ROS rad | sign | offset | ROS 可用区间 | 验证 |
|---|---|---|---|---|---|---|
| joint1 | 待测 | 待定 | 待测 | 待算 | 待算 | 未执行 |
| joint2 | 待测 | 待定 | 待测 | 待算 | 待算 | 未执行 |
| joint3 | 待测 | 待定 | 待测 | 待算 | 待算 | 未执行 |
| joint4 | 待测 | 待定 | 待测 | 待算 | 待算 | 未执行 |
| joint5 | 待测 | 待定 | 待测 | 待算 | 待算 | 未执行 |
| joint6 | 待测 | 待定 | 待测 | 待算 | 待算 | 未执行 |

静态 MCU 限位见 [协议第 5 节](../S27_TEST_2/README_USB_COMMUNICATION.md#5-静态限位和映射)。
ROS 规划可用范围取转换后的 MCU 区间与 URDF/MoveIt 区间的交集，sign=-1 时需重排上下界。
修改映射后应同步复核预设姿态和规划限位，而不是直接放大 MCU 限位。

~~~bash
cd ~/ros_test_8
colcon build --packages-select dm_mc02_hardware
source install/setup.bash
~~~

## 7. 阶段 3：首次使能与当前位置保持

本阶段**会发送位置帧**，目标是验证保持当前姿态；不再称为“ENABLE 但不发命令”。
固件使用正常 Kp/Kd，即使目标等于反馈也可能因误差、重力和控制器状态产生动作，需支撑和观察。

### 7.1 启动与占用接口

停止阶段 1；确认机械臂位于安全姿态、DT7 左上+右上、反馈有效，然后：

~~~bash
ros2 launch dm_mc02_hardware bringup.launch.xml allow_enable:=1
~~~

此时仍应 State=0，mode=10，只有 joint_state_broadcaster。
终端 2 使用 spawner 一次完成加载、配置、激活，避免旧手册 start/stop 命令在不同版本下不可用：

~~~bash
ros2 run controller_manager spawner arm_controller -c /controller_manager
ros2 control list_controllers
ros2 control list_hardware_interfaces
~~~

预期：arm_controller active，占用 joint1–6/position。ArmLink 发送 ENABLE，收到 ACK 后发送当前反馈基准帧，再进入 State=3；固件 sys=2。
若控制器显示 active 而 State 仍为 0，先查 allow_enable、mode、ready 和反馈，不要反复发轨迹。

**验收**：持续观察约 10 秒，无意外姿态变化、无 Hardware fault、CRC/ACK 错误不增长，反馈持续新鲜。只看到首帧或某一次 sys=2 不足以判通过。

### 7.2 撤防与模式门控

停止运动控制器（Humble 常用目标状态名 inactive/active；以本机 --help 为准）：

~~~bash
ros2 control set_controller_state --help
ros2 control set_controller_state arm_controller inactive
ros2 control list_controllers
~~~

若 gripper_controller 也 active，必须同样设为 inactive；仍有位置接口被占用时不满足“全部释放”条件。
当前全部释放路径会进入 ROS FAULT/发送 STOP，write() 可能返回 ERROR 触发生命周期处理；不能保证马上稳定显示 State=0。必要时按第 11 节重新启动。

模式门控测试应单独进行：在已确认不会触发危险动作的非 PC 模式，激活控制器。
ROS 会本地阻止 ENABLE，因此预期可以是 State=0、无使能，而**不必出现 WRONG_MODE ACK**。
若要测试固件 ACK=10，使用离线自测或专门协议注入；现有 Python control 工具也有本地模式检查。

## 8. 阶段 4：相对当前位置的小步点动

前提：阶段 3 通过，arm_controller active，只有一个命令来源，机械臂静止且标定正确。
控制器 `allow_partial_joints_goal=false`，必须提供 joint1–6 全部绝对目标。旧示例 `[0.02,0,0,0,0,0]` 会把其余关节送向零位，不能作为通用点动。

### 8.1 生成目标（只生成，不发送）

以下示例读取一次新反馈，保持其余 5 轴，仅 joint1 增加 0.02 rad；生成文件供检查。
在已 source 的 Ubuntu 终端运行。使用的是已安装的参数，请先完成重编译。

~~~bash
python3 - <<'PY'
import json
import math
import time
from pathlib import Path
import xml.etree.ElementTree as ET
import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from ament_index_python.packages import get_package_share_directory

rclpy.init()
node = rclpy.create_node('prepare_small_jog')
messages = []
sub = node.create_subscription(
    JointState, '/joint_states', messages.append, qos_profile_sensor_data)
try:
    deadline = time.monotonic() + 3.0
    while not messages and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
    if not messages:
        raise SystemExit('No joint feedback within 3 s')
    msg = messages[-1]
    measured = dict(zip(msg.name, msg.position))
    names = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
    if any(n not in measured or not math.isfinite(measured[n]) for n in names):
        raise SystemExit('Missing or non-finite feedback')
    q = [measured[n] for n in names]
    target = q.copy()
    target[0] += 0.02
    share = Path(get_package_share_directory('dm_mc02_hardware'))
    root = ET.parse(share / 'urdf/dm_mc02.ros2_control.xacro').getroot()
    params = {e.get('name'): e.text for e in root.iter('param')}
    for name, value in zip(names, target):
        sign = float(params[name + '_sign'])
        offset = float(params[name + '_offset'])
        lo = float(params[name + '_mcu_min'])
        hi = float(params[name + '_mcu_max'])
        if sign not in (-1.0, 1.0) or not all(
                math.isfinite(v) for v in (offset, lo, hi)) or lo >= hi:
            raise SystemExit('Invalid calibration: ' + name)
        if not lo <= sign * value + offset <= hi:
            raise SystemExit('MCU limit exceeded: ' + name)
    goal = {'trajectory': {'joint_names': names, 'points': [
        {'positions': target, 'time_from_start': {'sec': 2, 'nanosec': 0}}
    ]}}
    path = Path('/tmp/dm_mc02_jog_goal.json')
    path.write_text(json.dumps(goal), encoding='utf-8')
    print('Measured rad:', q)
    print('Target rad:  ', target)
    print('Review goal:', path)
finally:
    node.destroy_node()
    rclpy.shutdown()
PY
cat /tmp/dm_mc02_jog_goal.json
~~~

仅当生成命令成功后才使用文件，避免误用旧目标。该检查不包含碰撞、URDF 全部限制或 MCU feedback_valid。
检查所有关节、方向和干涉区域；若生成后发生移动、换模式或等待过久，重新生成，不能复用旧姿态。

### 8.2 执行并验收

~~~bash
ros2 action list -t
ros2 action send_goal /arm_controller/follow_joint_trajectory \
  control_msgs/action/FollowJointTrajectory \
  "$(cat /tmp/dm_mc02_jog_goal.json)" --feedback
~~~

预期：joint1 沿模型正方向小幅移动，其他关节保持，action 成功，反馈达到目标容差，State=3/sys=2 无故障。
方向不符立即停止，回到阶段 2。其它关节点动修改 target 的对应索引，每次重新读取反馈；靠近限位时选朝区间内部的小步。

默认路径/目标容差为 0.05/0.02 rad；0.02 rad 点动仅 action 成功不足以证明方向正确，必须观察实际反馈变化。
默认 max_step=0.01 rad 是相邻发送目标的差，需平滑轨迹，不能把它当成目标总位移限制。

### 8.3 停止与异常测试

先在静止持位时验证撤防，再在小幅低速运动中验证；记录触发方式、目标/反馈、故障位及停止响应。

- 控制器 inactive：验证 PC 接口释放与 STOP；若另一个运动控制器 active，先处理全部占用。
- 留在 PC 模式且通信中断：固件目标到期转保持，随后可能链路 FAULT。
- 拨杆退出：验证 PC 使能清除及新模式行为，不能把任意拨杆位置当全局急停。
- 物理急停：由现场支撑和断电方案保障，不以软件日志作为断电证明。
- 越界、旧时间戳、反馈丢失等拒绝逻辑优先使用离线测试，台架故障注入按协议验收矩阵开展。

## 9. 阶段 5：夹爪标定与点动（可选）

默认 grip_command_enable=0，仅保持基准目标。若不需要夹爪，保持该值并跳过本阶段。

独立记录闭合/张开时的**电机原始 rad**与**URDF joint8 轴向位移 m**。后者不一定等于整个夹口宽度，需按机构关系换算，不能直接用已映射的 /joint_states 反推物理标定。

| 参数 | 当前默认 | 应如何获得 |
|---|---:|---|
| grip_mcu_closed | 0.0 rad | 实测闭合端电机反馈 |
| grip_mcu_open | 0.7 rad | 实测张开端电机反馈 |
| grip_q_close_m | -0.000523 m | 对应 URDF 直线关节坐标 |
| grip_q_open_m | 0.009530 m | 对应 URDF 直线关节坐标 |
| grip_command_enable | 0 | 标定与方向核对后才设 1 |
| grip_max_step | 0.01 rad | 插件默认，按电机侧角度检查 |

当前代码要求 mcu_closed < mcu_open 且两端差非零；q 两端也不能相等。
若实测方向相反，先处理映射和限位实现，不能盲目对调参数。

重编译并重新启动后：

~~~bash
ros2 run controller_manager spawner gripper_controller -c /controller_manager
ros2 topic echo /joint_states --once
# 输入当前夹爪位置附近、已检查安全的小步绝对目标，单位 m
read -r -p "joint8 小步绝对目标（m）: " grip_target_m
ros2 action send_goal /gripper_controller/follow_joint_trajectory \
  control_msgs/action/FollowJointTrajectory \
  "{trajectory: {joint_names: [joint8], points: [{positions: [$grip_target_m], time_from_start: {sec: 2, nanosec: 0}}]}}" \
  --feedback
~~~

先小步确认方向，再逐步接近已测端点；不要首次就发满行程。
夹爪控制器也会占用位置接口并触发整条 USB 链路使能；停止夹爪不等于撤销其它控制器。
夹爪目标容差默认 0.001 m，路径容差 0.002 m，结合实测精度验收。

## 10. 阶段 6：MoveIt 接入检查

当前 move_group.launch.py / moveit_rviz.launch.py 由 MoveItConfigsBuilder 读取默认配置，其 Xacro 内含 FakeSystem。
这不表示单独启动 move_group 会自动启动假硬件，但也**不能假定它自动读取实机 bringup 的 robot_description**。

在执行实机轨迹前完成：

1. 只运行一个实机 controller_manager 和一个 joint_state_broadcaster；不要同时启动 demo.launch.py 或 mujoco.launch.py。
2. 核对 MoveIt 与实机模型的关节名、轴向、零位、几何和限位。后续建议建立专用 real_moveit launch，给 move_group/RViz 使用与实机相同的模型及映射结果。
3. 核对 config/moveit_controllers.yaml：arm_controller 使用 FollowJointTrajectory，joint1–6，action_ns=follow_joint_trajectory；gripper_controller 对应 joint8。
4. 核对转换后的实机限位与规划限位。默认 sign/offset 下，预设 pose 中 J2=1.89 rad 已大于 MCU 上限 1.7732357 rad；预设姿态不是实机合格目标。
5. 先验证规划所用 current state 来自真实 /joint_states，RViz 姿态与实物一致，再开放执行。

仅启动现有规划组件进行核对（实机 bringup 已运行）：

~~~bash
# 终端 3，先 source；此时只检查模型和规划
ros2 launch test_robot_moveit_config move_group.launch.py
# 终端 4，先 source
ros2 launch test_robot_moveit_config moveit_rviz.launch.py
# 终端 2
ros2 node list
ros2 topic info /joint_states --verbose
ros2 control list_controllers
ros2 action info /arm_controller/follow_joint_trajectory
ros2 param get /move_group robot_description
ros2 param get /robot_state_publisher robot_description
~~~

如果两端模型/坐标不一致，记录差异并完成专用实机启动配置后再继续，不把“RViz 能打开”作为集成通过。

首次执行：速度、加速度缩放均设 0.1；先 Plan，检查完整路径和 J1/J2 干涉，再执行当前安全姿态附近的小目标。
记录 action 结果、反馈、最大步长故障和到位误差。仅调整速度缩放不能修复坐标映射或限位错误。

## 11. 停止、故障定位与恢复

### 11.1 常见现象

| 现象 | 当前可能原因 | 下一步 |
|---|---|---|
| Failed to open | 路径、权限、串口被占用 | 停其它串口程序，核对 udev/dialout |
| active 控制器但 State=0 | allow_enable=0、mode 非 10、ready/反馈不足、MCU FAULT | 查看启动参数，再停 ROS 用 monitor 独立检查 |
| State=1 长时间未前进 | ENABLE ACK 丢失/解析或写入问题 | 超时约 2 s 后会撤防，保存日志，不循环盲启 |
| State=4 / Hardware fault | 模式、反馈、ACK、步长或限位 | 当前日志未保存具体 disarm 原因，不能期待总有“limit violation”等字样 |
| 位置 NaN | 未收到状态或 STATE 超时 | 检查 rx、USB、CAN 和实际进程 |
| sys=3 / fault=0x0008 | 已使能时 CAN 反馈丢失 | 停止，检查电机供电、CAN、接插件 |
| sys=3 / fault=0x0001 | 已使能时链路超时 | 查发送调度、进程、USB |
| fault 含 0x0004 | RX 溢出 | 降低积压、查读取和调度；当前不保证自动撤防 |
| MoveIt action 失败 | 控制器未 active、容差、状态或坐标/限位问题 | 分别核对 action、控制器和实机状态 |

### 11.2 保守恢复流程

1. 停止新目标与 MoveIt 执行；确保机械支撑和现场可控。
2. 将所有已加载的运动控制器设 inactive；保留 bringup 日志。若命令失败或硬件进入错误状态，结束 bringup。
3. 确认 ROS 已退出，独占使用 monitor 查看 mode、sys、fault、ready 和全部反馈年龄。
4. 排除实际原因。固件故障位不会被 STOP/ENABLE(0) 清除；ROS 又不会在 SYS_FAULT 下主动 ENABLE，不能承诺停/启控制器即可恢复。
5. 如需复位 MCU，先确认初始化动作安全并支撑机械臂，然后按板卡实际方式复位/重新上电。复位会重新初始化，可能产生动作。
6. 从 allow_enable:=0 遥测重新验收；正常后再进入 PC 模式和阶段 3。不要复用旧轨迹目标文件。
7. 若未来增加专用故障确认/ENABLE 恢复工具，先验证状态机和基准帧，再替换此保守流程。

Ctrl+C/正常生命周期退出会尝试 STOP+ENABLE(0)，但进程崩溃、断线时不保证发送成功；需要固件期限和现场停止方式配合。

## 12. 后续开发、参数调整与回归

一次只修改一类参数，记录旧值、新值、理由及回归结果。

| 改动 | 文件 | 至少回归 |
|---|---|---|
| 设备路径/allow_enable 传递 | urdf/test_robot_real.urdf.xacro、launch/bringup.launch.xml | 只读、门控 |
| sign/offset/夹爪端点 | urdf/dm_mc02.ros2_control.xacro | 两个参考姿态、方向、小步、规划限位 |
| max_step/valid_time/state_timeout | Xacro、ArmLink 配置 | 插值、发送周期、过期、停止 |
| 控制器容差 | config/ros2_controllers.yaml | 到位误差、action 超时、异常停止 |
| 协议或故障恢复 | 固件/Python/ROS 三端 | 协议自测、ACK/会话/故障矩阵、两份文档 |

max_step exceeded 优先检查轨迹和调度，不直接加大步长；轨迹容差错误先排查标定/响应，不先放宽容差。
PC 固件没有独立速度/加速度斜坡，velocity_limit 字段保留；不要误以为填写该字段就能限制速度。

回到 MuJoCo 前停止所有实机进程。若仿真包及其依赖已完成构建：

~~~bash
ros2 launch test_robot_mujoco mujoco.launch.py
~~~

该 launch 默认已启动 MoveIt/RViz/控制器，不要再叠加 demo.launch.py。
仿真回归能检查接口和规划，不能替代电机量程、实物碰撞、制动和反馈延迟验证。

## 13. 工作记录与交接模板

推荐在 Ubuntu `~/arm_logs/日期-测试名/` 保存终端日志、CSV、rosbag、参数快照和结果。
ROS 运行期间用 topic/bag 采集；monitor 的原始 CSV 必须在另一个独占串口的测试阶段采集。

~~~bash
# 新终端先 source；已有 bringup 正在运行时
ros2 bag record -o ~/arm_logs/stage4_bag /joint_states /arm_controller/controller_state
~~~

同一路径已存在时换新的测试目录。退出 rosbag 后再结束本次记录。

~~~text
日期/操作者：
机械臂编号/负载/电机型号与量程：
固件版本或文件哈希/烧录产物：
ROS 代码版本或源码快照：
Ubuntu / ROS / controller_manager 版本：
设备路径/VID/PID/序列号：
allow_enable / mode / ROS State / MCU sys / fault：
sign、offset、夹爪映射及限位快照：
valid_time、state_timeout、max_step、控制器频率：
测试编号/进入条件：
执行命令及目标（明确 MCU rad / ROS rad / joint8 m）：
预期：
实际：
通过 / 失败 / 未执行：
日志、CSV、视频/截图路径：
故障原因、恢复步骤、待办负责人：
~~~

| 检查点 | 结果 | 证据 |
|---|---|---|
| 构建及离线测试 | 未执行 | 待填 |
| 原始反馈 7/7 有效 | 未执行 | 待填 |
| ROS 只读与标定 | 未执行 | 待填 |
| 首次使能保持/撤防 | 未执行 | 待填 |
| 六轴小步与停止 | 未执行 | 待填 |
| 夹爪（可选） | 未执行 | 待填 |
| MoveIt 模型/action/小轨迹 | 未执行 | 待填 |
| 故障恢复回归 | 未执行 | 待填 |

2026-11-02 文档修订：修正状态编号、100 ms 超时、allow_enable 启动方式、标定单位、点动绝对目标、STOP 边界及 MoveIt/仿真启动关系；新增源码待办、阶段验收和交接模板。

