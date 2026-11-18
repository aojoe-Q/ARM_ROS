# 实机操作指南：从 USB 串口配置到 ROS 2 / MoveIt 控制

适用源码：当前 `ros_test_8` 与 `S27_TEST_2`，USB 协议 v2。更新时间：2026-11-18。

本指南使用 **Ubuntu 22.04、ROS 2 Humble、Bash**。除特别说明外，命令都在连接机械臂的 Ubuntu 电脑上执行，不是在 Windows PowerShell 中执行。不会要求修改或重新烧录固件。

本文对应当前软件实现。Windows 下已完成离线协议、状态机和模型测试；完整 Ubuntu 构建、ROS 启动和实机动作仍需要按以下步骤验证。每个阶段达到预期后再进入下一阶段，保留实际输出，不把本文的预期当作实测结果。

## 0. 路径、终端与流程

Windows 源码目前在 `C:\Users\lenovo\Desktop\ros_test_8`。下面假定你将它复制到 Ubuntu 的 `~/ros_test_8`，将固件工具所在工程放到 `~/S27_TEST_2`。

建议只复制需要的源码与文档，在 Ubuntu 重新构建。不要把 Windows 或另一台电脑的 `build/`、`install/` 当作本机可直接使用的构建结果；尤其不要使用其中的 Windows `.exe` 运行 ROS 驱动。

后续约定：

- **终端 A**：持续运行实机 launch，查看驱动日志。
- **终端 B**：检查状态、激活控制器、发送经检查的目标。
- **终端 C**：可选，记录 rosbag 或查看反馈。

整体顺序：

```text
识别 USB 串口 → 配置权限/固定名称 → 检查串口占用
  → 安装构建环境 → 构建并测试
  → 独占串口读取原始反馈 → 退出监视器
  → ROS 只读遥测 → 实测方向/零位/夹爪映射
  → 首次使能保持 → 单轴小步验证
  → 夹爪控制（可选）→ MoveIt → 停机与故障恢复
```

实机开始前确认机械臂得到支撑、运动空间清楚、已有可用的断电/急停方式。当前固件的无力模式会使机械臂失力；USB STOP 是撤销 PC 控制，不是整机急停。`allow_enable=0` 也不会禁止固件的遥控和上电动作。

## 1. 识别 USB 虚拟串口

### 1.1 确认连接的是 MCU 的 USB CDC 接口

使用支持数据传输的 USB 线连接运行当前固件的控制板。Ubuntu 通常自带 `cdc_acm` 驱动，不需要下载 Windows 的 CH340 驱动。

先运行：

```bash
lsusb
ls /dev/ttyACM* /dev/ttyUSB* 2>/dev/null
ls -l /dev/serial/by-id/ 2>/dev/null
```

在未使能、已做好支撑的初次连接阶段，可在插线前打开下面的命令，观察插线后新增的设备：

```bash
sudo dmesg --follow
```

预期看到类似 `cdc_acm ... ttyACM0: USB ACM device`。按 `Ctrl+C` 退出日志查看。

设备编号可能是 `ttyACM1` 等，要以实际结果为准。若只出现 `ttyUSB0`，核实它是否是另一个 USB 转 UART 适配器；本项目使用 MCU USB CDC 链路，不是随意选择任意串口。

如果没有设备：先检查数据线、接口、板卡供电和 USB 枚举日志。虚拟机用户还需要将该 USB 设备分配给 Ubuntu 客体系统；主机占有设备时，客体里不会出现对应的 `/dev/ttyACM*`。

### 1.2 设置当前终端中的设备变量

以下先以实际识别出的 `ttyACM0` 为例：

```bash
export ARM_DEVICE=/dev/ttyACM0
ls -l "$ARM_DEVICE"
udevadm info --query=property --name="$ARM_DEVICE"
```

记录 `ID_VENDOR_ID`、`ID_MODEL_ID`、`ID_SERIAL_SHORT` 等信息。它们用于辨认设备，不要直接套用其他开发板的 VID/PID。

## 2. 配置串口访问权限

查看设备所属用户组，常见为 `dialout`：

```bash
ls -l "$ARM_DEVICE"
id -nG
sudo usermod -aG dialout "$USER"
```

执行后**注销 Ubuntu 桌面会话并重新登录**，然后打开新终端验证：

```bash
id -nG
export ARM_DEVICE=/dev/ttyACM0
test -r "$ARM_DEVICE" && test -w "$ARM_DEVICE" && echo "串口读写权限正常"
```

`id -nG` 中应包含 `dialout`。不要把长期使用 `sudo ros2 ...` 或 `chmod 777` 当作权限配置，这会掩盖用户组或设备路径问题。

### 2.1 优先使用稳定的 by-id 路径

如果 `/dev/serial/by-id/` 下存在唯一对应控制板的链接，可以直接使用它：

```bash
ls -l /dev/serial/by-id/
```

把完整实际路径作为 launch 的 `device` 参数即可，不必创建 udev 规则。

### 2.2 可选：创建 /dev/dm_mc02 固定名称

先查看属性：

```bash
udevadm info --name=/dev/ttyACM0 --attribute-walk
sudo nano /etc/udev/rules.d/99-dm-mc02.rules
```

下面是**模板，必须替换三个占位值，不要原样保存**：

```text
SUBSYSTEM=="tty", KERNEL=="ttyACM*", ATTRS{idVendor}=="实际VID", ATTRS{idProduct}=="实际PID", ATTRS{serial}=="实际序列号", SYMLINK+="dm_mc02", GROUP="dialout", MODE="0660"
```

若设备没有序列号，不能依靠虚构的序列号匹配；单板场景可按实际 VID/PID 匹配，多块相同板卡应使用可区分的设备属性或稳定物理端口路径。

保存后执行：

```bash
sudo udevadm control --reload-rules
sudo udevadm trigger --subsystem-match=tty
ls -l /dev/dm_mc02
readlink -f /dev/dm_mc02
export ARM_DEVICE=/dev/dm_mc02
```

如需拔插使规则生效，只在当前尚未进入运动控制的配置阶段进行。重新打开终端时也要重新设置 `ARM_DEVICE`，或直接在命令中写完整设备路径。

## 3. 串口参数与占用检查

### 3.1 当前串口参数

| 项目 | 当前使用方式 |
|---|---|
| 设备类型 | MCU 原生 USB CDC 虚拟串口 |
| 主机波特率设置 | 115200 |
| 数据格式 | 原始二进制模式，8 数据位、无校验、1 停止位 |
| 流控 | 不使用硬件 RTS/CTS 流控；raw 模式关闭软件流控 |
| 协议 | `0xA5` 帧头、版本 `0x02`、小端字段、CRC16 |
| 固件状态数据 | 约 100 Hz 的 STATE 帧 |

`serial_port.cpp` 会在打开设备时自动配置串口，因此通常不需要事先运行 `stty`。115200 是主机提交的串口参数；当前 USB CDC 固件没有用它将 USB 数据速率限制为 UART 的 115200 bit/s。

不要用 `echo` 向设备写字符串，也不要在普通串口终端里输入 `ENABLE`：固件要求完整的二进制帧和 CRC。文本终端中显示乱码不等于协议出错。

### 3.2 一个阶段只保留一个串口读取程序

```bash
sudo fuser -v "$ARM_DEVICE"
```

没有占用时可能没有输出。若有占用，确认程序身份后正常退出对应的 ROS、监视器或串口终端。不要同时运行监视器和 ROS 驱动，否则读取数据可能互相抢占；当前代码不能替你保证系统级独占。

只有发现 ModemManager 实际探测该设备时，再为此设备增加匹配规则并设置 `ENV{ID_MM_DEVICE_IGNORE}="1"`。不需要无条件卸载或全局关闭系统服务。

## 4. 准备 ROS 环境和工作区

### 4.1 确认基础环境

```bash
cat /etc/os-release
test -f /opt/ros/humble/setup.bash
source /opt/ros/humble/setup.bash
echo "$ROS_DISTRO"
command -v ros2
```

预期为 Ubuntu 22.04 和 `humble`。如果 `/opt/ros/humble/setup.bash` 不存在，先按 ROS 2 官方 Ubuntu deb 安装步骤配置软件源并安装 Humble；下面的 apt 命令以 ROS 软件源已经配置为前提。

退出 Conda 环境，在使用系统 Python 的干净终端安装依赖：

```bash
sudo apt update
sudo apt install build-essential python3-colcon-common-extensions python3-rosdep \
  python3-serial python3-yaml ros-humble-desktop ros-humble-moveit \
  ros-humble-ros2-control ros-humble-ros2-controllers ros-humble-xacro
```

### 4.2 确认源码位置

```bash
cd ~/ros_test_8
ls src
colcon list
```

应能找到 `dm_mc02_hardware`、`test_robot`、`test_robot_moveit_config` 等包。

如果此目录带有其他系统复制来的构建产物，建议使用第 5 节的独立 Ubuntu 构建目录，既不覆盖旧结果，也避免使用旧 CMake 绝对路径。

### 4.3 安装实机范围内的包依赖

第一次使用 rosdep 且尚未初始化时运行：

```bash
sudo rosdep init
```

如果提示已经初始化，不要重复覆盖。然后运行：

```bash
rosdep update
cd ~/ros_test_8
rosdep install --from-paths src/dm_mc02_hardware src/test_robot \
  src/test_robot_moveit_config --ignore-src --rosdistro humble -y
```

这里针对实机链路安装依赖，不要求安装可选 MuJoCo Python 环境。遇到包名或 rosdep key 错误，保留具体错误并解决；不要把依赖安装失败当作可以直接继续。

## 5. 构建、测试和加载环境

### 5.1 使用独立 Ubuntu 产物目录

终端 A：

```bash
source /opt/ros/humble/setup.bash
cd ~/ros_test_8
colcon --log-base log_ubuntu build \
  --build-base build_ubuntu --install-base install_ubuntu \
  --packages-up-to dm_mc02_hardware \
  --event-handlers console_direct+
```

要求构建完成且没有失败包。不要用 `BUILD_TESTING=OFF` 绕过测试编译错误。

### 5.2 运行测试

```bash
colcon --log-base log_ubuntu test \
  --build-base build_ubuntu --install-base install_ubuntu \
  --packages-select dm_mc02_hardware --event-handlers console_direct+
colcon test-result --test-result-base build_ubuntu --verbose
```

可补充不依赖 ROS 的测试：

```bash
python3 src/dm_mc02_hardware/test/run_standalone.py
# 只有 Ubuntu 上也有该固件目录时再执行这一条：
python3 src/dm_mc02_hardware/test/run_standalone.py --firmware ~/S27_TEST_2
```

第二条读取固件编解码源码进行互操作检查，不修改固件、不连接电机。

### 5.3 每个新终端都加载同一工作区

```bash
source /opt/ros/humble/setup.bash
source ~/ros_test_8/install_ubuntu/setup.bash
export ARM_DEVICE=/dev/dm_mc02
# 没有建立固定名称时，上一行改成实际 /dev/ttyACM0 或 by-id 路径。
ros2 pkg prefix dm_mc02_hardware
```

结果应指向本工作区 `install_ubuntu` 下的实机包。若指向其他工作区，打开干净终端重新 source；不要混用旧 `install` 和本指南的 `install_ubuntu`。

## 6. 先独占串口读取原始反馈

停止其他串口程序。使用固件工程已有的只读监视工具：

```bash
mkdir -p ~/arm_logs
python3 ~/S27_TEST_2/tools/arm_host_monitor.py --list
python3 ~/S27_TEST_2/tools/arm_host_monitor.py \
  --port "$ARM_DEVICE" --verbose --duration 30 \
  --csv "$HOME/arm_logs/raw_$(date +%Y%m%d_%H%M%S).csv"
```

此工具需要相邻的 `arm_usb_protocol.py`，不要只复制一个 monitor 文件。工具只读帧，不发送 ENABLE 或运动指令，但板上固件自己的动作不会因此被禁止。

需要确认：

| 观察项 | 通过条件 |
|---|---|
| 串口 | 能打开并收到可解析帧 |
| 固件就绪 | 初始化结束后 `ready=yes` |
| 七路反馈 | `feedback_valid=127`，即 `0x7F` |
| 每路反馈年龄 | 持续不超过 100 ms，无 STALE/never |
| CRC | 错误不持续增长 |
| 状态帧率 | 统计值接近 100 Hz；终端默认 5 Hz 刷新不是链路帧率 |
| 原始位置 | 数值合理，能识别每个电机对应的关节 |

进入 PC 模式后应看到 `mode=10`。当前源码左上右上为 PC 模式；以板上实际反馈验证它确实运行了对应固件。若仍不是 10，先确认板上版本和遥控输入，不要仅凭源码就认定模式已经生效。

监视器正常退出后，再用 `fuser` 确认串口释放，进入下一阶段。若 Ubuntu 没有固件工具目录，可先复制整个 `tools` 目录及依赖文件；也可以直接用下一节检查 ROS 遥测，但原始反馈年龄和标定数据最好仍单独采集。

## 7. ROS 只读遥测

### 7.1 启动硬件节点

终端 A，已 source 环境并设置设备变量：

```bash
cd ~/ros_test_8
mkdir -p ~/arm_logs
ros2 launch dm_mc02_hardware bringup.launch.xml \
  allow_enable:=0 device:="$ARM_DEVICE" \
  2>&1 | tee "$HOME/arm_logs/telemetry_$(date +%Y%m%d_%H%M%S).log"
```

此时只自动启动 `joint_state_broadcaster`，不会自动激活机械臂轨迹控制器。暂时不要运行 `test_moveit`、`commander` 或模拟硬件 launch。

### 7.2 查看 ROS 状态

终端 B，同样 source `install_ubuntu`：

```bash
ros2 node list
ros2 control list_controllers
ros2 control list_hardware_interfaces
ros2 topic echo /joint_states --once
ros2 topic hz /joint_states
```

`ros2 topic hz` 用 `Ctrl+C` 退出。预期：状态广播器 active，七个受控关节名称为 `joint1`～`joint6` 和 `joint8`，机械臂位置反馈持续更新。

当前默认夹爪未标定，**joint8 的 position/velocity/effort 为 NaN 是预期行为**。原始夹爪角度看驱动日志 `grip_motor`，或上节原始监视器。六轴反馈若为 NaN，则应排查反馈有效性和超时。

### 7.3 理解驱动日志

| 字段 | 含义 |
|---|---|
| `State=0` | ROS 链路 IDLE；只读状态正常如此 |
| `State=1 / 2 / 3 / 4` | 等待使能 ACK / 等待首帧条件 / ARMED / FAULT |
| `mode=10` | 固件 PC 模式 |
| `sys=0 / 1 / 2 / 3` | 固件 IDLE / ENABLED / RUNNING / FAULT |
| `feedback=0x7f` | 七路反馈有效位均置位 |
| `fault` | 固件锁存故障位 |
| `flags` | 当前状态/健康位；包含 ready 位和 CAN/电机故障位 |
| `ack` | 最近记录的拒绝码，在新一轮 ENABLE 尝试时清为 OK |
| `grip_motor` | 夹爪原始电机角度，rad |
| `reason` | 插件停止原因 |

只读时不会主动用 ENABLE 清除历史故障。注意 `rx` 是处理帧数，不应直接当作 STATE 帧率；`feedback` 位图还需要结合主机超时判断，不能只看某一行日志。

**本阶段通过后再标定**：串口稳定、CRC 无持续错误、六轴反馈有效、各轴映射关系可核对。

## 8. 实测机械臂方向和零位

### 8.1 编辑哪个文件

停止 ROS 进程后，编辑源码：

```bash
cd ~/ros_test_8
cp src/dm_mc02_hardware/urdf/dm_mc02.ros2_control.xacro \
  "$HOME/arm_logs/dm_mc02_calibration_before_$(date +%Y%m%d_%H%M%S).xacro"
nano src/dm_mc02_hardware/urdf/dm_mc02.ros2_control.xacro
```

标定期间保持 `mapping_calibrated=0`、`grip_command_enable=0`，继续用 `allow_enable:=0`。

### 8.2 每轴至少测两个参考姿态

通过已经验证过的遥控或机械标定流程，把关节放到可辨认的参考姿态；不要通过尚未标定的 ROS 指令反过来验证标定，也不要强行推动带力的位置环。

对每轴记录：参考姿态 A/B 的电机原始角度，以及按照 URDF 关节轴和几何定义确定的 ROS 角度。

```text
q_ros = sign × (q_mcu − offset)
offset = q_mcu_ref − sign × q_ros_ref
```

沿 ROS 正方向运动，电机读数增大取 `sign=+1`，减小取 `sign=-1`。第二个参考姿态用于验证方向、偏置和比例是否一致。如果变化幅度明显不满足 ±1 比例，不能只修改 sign/offset，应检查关节对应关系、角度定义及是否存在传动比例问题。

| 轴 | MCU A（rad） | ROS A（rad） | MCU B（rad） | ROS B（rad） | sign | offset | 验证 |
|---|---|---|---|---|---|---|---|
| J1 | 待测 | 待测 | 待测 | 待测 | 待定 | 待定 | 未执行 |
| J2 | 待测 | 待测 | 待测 | 待测 | 待定 | 待定 | 未执行 |
| J3 | 待测 | 待测 | 待测 | 待测 | 待定 | 待定 | 未执行 |
| J4 | 待测 | 待测 | 待测 | 待测 | 待定 | 待定 | 未执行 |
| J5 | 待测 | 待测 | 待测 | 待测 | 待定 | 待定 | 未执行 |
| J6 | 待测 | 待测 | 待测 | 待测 | 待定 | 待定 | 未执行 |

J2/J3 固件范围目前为负角度，URDF 范围为正角度，但不能因此直接认定 sign=-1、offset=0。固件的回零目标也不能直接作为 URDF offset。

如从 `/joint_states` 反推 MCU 角度，须用当时实际生效的 `q_mcu=sign*q_ros+offset`；只有原 sign=1、offset=0 时才能直接抄读数。原始监视器提供的是电机侧角度。

### 8.3 填写配置并验证限位交集

填好六轴 `jointN_sign` 和 `jointN_offset`，确认后将：

```xml
<param name="mapping_calibrated">1</param>
```

当前电机侧硬限位：

| 轴 | 最小值 rad | 最大值 rad |
|---|---:|---:|
| J1 | -2.96054893 | 2.81695309 |
| J2 | -2.05669899 | 0 |
| J3 | -3.00016972 | 0 |
| J4 | 0 | 3.00567291 |
| J5 | -0.904316101 | 1.40938965 |
| J6 | -0.650879326 | 2.32408657 |
| Grip | -12 | 0 |

这些是电机侧限位，不能改成 ROS 侧角度填写。配置只能收紧，不能越过固件范围。实机模型会取“原 URDF 范围”与“映射后的电机范围”的交集，空交集会拒绝启动，提示检查标定。

### 8.4 每次改配置都重新构建并重启

```bash
source /opt/ros/humble/setup.bash
cd ~/ros_test_8
colcon --log-base log_ubuntu build \
  --build-base build_ubuntu --install-base install_ubuntu \
  --packages-select dm_mc02_hardware --event-handlers console_direct+
source install_ubuntu/setup.bash
```

重新启动只读遥测，逐轴核对 ROS 反馈方向和参考姿态。当前 `mapping_calibrated` 是 Xacro 内的参数，**不是** `bringup.launch.xml` 暴露的命令行参数；不要用 `mapping_calibrated:=1` 代替完成标定。

## 9. 夹爪标定与保持策略

可以先验证六轴直接轨迹控制，夹爪保持不动；但是开启当前统一 MoveIt 入口前，需要夹爪也有可信的米制反馈。

需要测量四个值：

| 参数 | 意义 |
|---|---|
| `grip_mcu_closed` | 实测闭合参考端的电机角度，rad |
| `grip_mcu_open` | 实测张开参考端的电机角度，rad |
| `grip_q_close_m` | 闭合参考端对应 URDF joint8 的直线位移，m |
| `grip_q_open_m` | 张开参考端对应 URDF joint8 的直线位移，m |

joint8 的位移不一定等于两爪之间的总间距，要按 URDF 对该关节的定义测量。检查中间开度，确认线性映射近似可用。固件 `-12 rad` 是兜底限位，不是闭合参考点，不能命令机械机构撞到该限位来做标定。

填写实测值后设置 `grip_calibrated=1`，先保持 `grip_command_enable=0`，重新构建后用只读遥测核对 joint8。当前两个电机端点的默认值都是 0 占位，不能只把 calibrated 改成 1。

启用夹爪运动是另一步：米制反馈和开合方向验证后，才设置 `grip_command_enable=1`、重新构建并启动夹爪控制器。否则不激活它；机械臂控制期间夹爪保持使能首帧的原始角度。

## 10. 首次使能：先保持当前位置

### 10.1 启动允许使能的实机节点

先退出只读 launch，确认没有重复节点或串口占用。机械臂放在已确认可保持的姿态，遥控切入当前固件的 PC 模式，检查反馈实际为 `mode=10`。

终端 A：

```bash
source /opt/ros/humble/setup.bash
source ~/ros_test_8/install_ubuntu/setup.bash
export ARM_DEVICE=/dev/dm_mc02
ros2 launch dm_mc02_hardware bringup.launch.xml \
  allow_enable:=1 device:="$ARM_DEVICE"
```

`allow_enable:=1` 只是允许使能。控制器尚未 claim 位置接口时，插件仍停留在 IDLE。

### 10.2 激活机械臂控制器

终端 B：

```bash
source /opt/ros/humble/setup.bash
source ~/ros_test_8/install_ubuntu/setup.bash
ros2 run controller_manager spawner arm_controller
ros2 control list_controllers
ros2 control list_hardware_interfaces
```

预期 `arm_controller` 为 active，joint1～joint6 的 position 命令接口被 claim。首次加载使用 spawner，已加载但 inactive 时使用 `set_controller_state ... active`，不要重复加载同名控制器。

驱动会依次发送 ENABLE、等待匹配序号的成功 ACK、等待固件已使能且故障清零，再以当前七路反馈发送基准帧。预期最后 `State=3`，固件随后 `sys=2`。

**此时先不发送轨迹。**观察至少数秒：机械臂保持当前姿态，夹爪不因使能发生开合，反馈稳定，CRC/ACK 错误不持续增长。异常就按第 13 节停用控制器并排查。

如果卡在 IDLE 或握手失败，优先查模式、七路反馈、ready、CAN/DM 故障标志及标定配置，不要增大超时或取消限位作为首次处理。

## 11. 六轴小步验证：从新反馈生成目标

当前控制器要求六轴完整目标，不能只发一个关节，也不要把其余五轴随意填成 0。初次试验选择已确认有活动空间和限位余量的轴，保留其余轴的实时位置。

下面例子只生成目标文件，**不会发送运动**。它从当前反馈读取六轴，将 joint1 增加 0.01 rad（约 0.57°），时间 3 s；同时检查硬件映射和模型位置范围。若该方向不可行，应重新选择方向或关节，不要扩大限制。

终端 B，在机械臂已经使能并保持时执行：

```bash
python3 - <<'PY'
import json
import math
import runpy
import time
import xml.etree.ElementTree as ET
from pathlib import Path
import rclpy
import xacro
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from ament_index_python.packages import get_package_share_directory

output = Path('/tmp/dm_mc02_jog_goal.json')
output.unlink(missing_ok=True)  # 先移除本例旧目标，失败时不会误用上次结果
share = Path(get_package_share_directory('dm_mc02_hardware'))
xml = xacro.process_file(str(share / 'urdf/test_robot_real.urdf.xacro'),
                         mappings={'allow_enable': '1'}).toxml()
adapt = runpy.run_path(str(share / 'launch/real_model.py'))['adapt_description']
xml, limits, params = adapt(xml)
names = [f'joint{i}' for i in range(1, 7)]
rclpy.init()
node = rclpy.create_node('prepare_small_jog')
messages = []
sub = node.create_subscription(JointState, '/joint_states', messages.append, qos_profile_sensor_data)
try:
    deadline = time.monotonic() + 3
    while not messages and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
    if not messages:
        raise SystemExit('3 秒内没有 joint_states，未生成目标')
    values = dict(zip(messages[-1].name, messages[-1].position))
    current = [values.get(n, float('nan')) for n in names]
    if not all(math.isfinite(q) for q in current):
        raise SystemExit('反馈缺失或无效，未生成目标')
    target = current.copy()
    target[0] += 0.01  # 仅在确认 joint1 正方向有活动空间后使用
    for n, start, end in zip(names, current, target):
        lo, hi = limits[n]['min_position'], limits[n]['max_position']
        if not (lo <= start <= hi and lo <= end <= hi):
            raise SystemExit(f'{n} 当前值或目标超出实机模型范围')
        mcu = float(params[n + '_sign']) * end + float(params[n + '_offset'])
        if not float(params[n + '_mcu_min']) <= mcu <= float(params[n + '_mcu_max']):
            raise SystemExit(f'{n} 超出电机侧限位')
    goal = {'trajectory': {'joint_names': names, 'points': [
        {'positions': current, 'time_from_start': {'sec': 0, 'nanosec': 0}},
        {'positions': target, 'time_from_start': {'sec': 3, 'nanosec': 0}}
    ]}, 'goal_tolerance': [{'name': n, 'position': 0.002} for n in names],
       'goal_time_tolerance': {'sec': 1, 'nanosec': 0}}
    output.write_text(json.dumps(goal), encoding='utf-8')
    print('当前 rad:', current)
    print('目标 rad:', target)
    print('已生成（未发送）:', output)
finally:
    node.destroy_node()
    rclpy.shutdown()
PY
```

检查终端打印的当前值、目标和运动方向。此检查不检查碰撞、遥控模式或 MCU 原始反馈年龄，仍要确认驱动日志和实际空间。生成后若姿态改变、重新使能或经历停机，必须重新生成，不复用旧文件。

确认后，下面命令才会执行实际运动：

```bash
ros2 action send_goal /arm_controller/follow_joint_trajectory \
  control_msgs/action/FollowJointTrajectory \
  "$(cat /tmp/dm_mc02_jog_goal.json)" --feedback
```

预期目标被接受，反馈随运动更新，最终结果 `error_code: 0`。同时用 `/joint_states` 和实物确认轴、方向及实际位移正确。例子将到位位置容差设为 0.002 rad，避免默认 0.02 rad 容差比本次小步还大；若实机达不到该精度，应检查跟踪和测量，不把成功状态当作唯一判断。

逐轴验证时，每次都从新反馈生成目标，仅修改 `target` 中一个索引，并检查方向和范围。结束后不要直接发全零或调用未经验证的 `home`。

## 12. 夹爪与 MoveIt

### 12.1 夹爪独立控制（可选）

完成第 9 节、设置 `grip_command_enable=1` 并重新构建/重启后：

```bash
ros2 run controller_manager spawner gripper_controller
ros2 control list_controllers
ros2 topic echo /joint_states --once
```

先验证激活不引起跳动，再在已验证范围内从当前 joint8 值选择一个很小的目标。以下命令会提示你输入**实测确认后的 joint8 绝对目标，单位 m**，不会替你猜开合端点：

```bash
read -r -p '输入已检查的 joint8 目标位移（m）: ' GRIP_TARGET_M
ros2 action send_goal /gripper_controller/follow_joint_trajectory \
  control_msgs/action/FollowJointTrajectory \
  "{trajectory: {joint_names: [joint8], points: [{positions: [$GRIP_TARGET_M], time_from_start: {sec: 5, nanosec: 0}}]}}" \
  --feedback
```

不要将 -12 等电机角度填入这里。目标与当前位移之差还必须结合标定比例和 5 s 时间满足电机侧速度、步长限制；首测不要直接从一端走到另一端。夹爪默认到位容差为 0.001 m，小步试验也要观察实际反馈，不能只看 action 成功。

### 12.2 使用统一实机 MoveIt 入口

机械臂和夹爪反馈标定都完成后，先停用控制器并退出原 launch，再启动：

```bash
ros2 launch dm_mc02_hardware bringup.launch.xml \
  allow_enable:=1 device:="$ARM_DEVICE" use_moveit:=true use_rviz:=true
```

另一已 source 的终端激活机械臂控制器：

```bash
ros2 run controller_manager spawner arm_controller
# 仅在已经允许夹爪命令时启动：
# ros2 run controller_manager spawner gripper_controller
```

不要同时启动 `test_robot_bringup`、MuJoCo 或旧的独立 `test_robot_moveit_config move_group.launch.py`，否则会出现同名节点、控制器或模型不一致。

RViz 操作顺序：

1. 检查模型姿态与实物一致，特别是 J2/J3 和夹爪。
2. MotionPlanning 中选择 `arm`，起点采用当前状态。
3. 速度和加速度缩放先设为 0.1。
4. 选择很近且空间明确的目标，先点击 **Plan**。
5. 检查整段轨迹、末端方向、周围物体和自碰撞情况。
6. 确认后点击 **Execute**，观察真实反馈和驱动日志。

实机入口使用映射后的限位交集，机械臂规划上限 0.4 rad/s、0.8 rad/s²；默认缩放 0.1。固件还会执行自己的限速/限加速度插补。旧 SRDF 越界命名姿态会被过滤，但保留下来的姿态仍不等于已经验证可执行。

当前 `commander` 中关闭夹爪目标名写作 `gripper_closed`，SRDF 是 `gripper_close`。这个现存名称差异不影响上述直接 action 流程；在使用该上层程序前需处理并单独验证。初次实机联调不运行 `test_moveit`，它包含主动执行代码。

## 13. 正常停止、故障恢复与串口重连

### 13.1 正常停止

先取消正在执行的上层任务/MoveIt 轨迹，然后将已激活的命令控制器设为 inactive：

```bash
ros2 control set_controller_state arm_controller inactive
# 只有该控制器已加载并处于 active 时执行：
ros2 control set_controller_state gripper_controller inactive
ros2 control list_controllers
```

全部命令接口释放后驱动发送 STOP，保留状态广播器可继续观察反馈。PC 模式中的固件按自身逻辑保持当前姿态。要关闭 ROS，再在终端 A 按 `Ctrl+C`。

终端客户端的 `Ctrl+C` 不应被当成所有控制器都已停止的证明。不要为停止 PC 运动而随意切入无力模式；支撑和断电顺序按你的实际机械系统执行。

### 13.2 FAULT 后恢复

1. 记录 `reason`、`ack`、`fault`、`flags`、`mode` 和异常时间。
2. 停止上层自动发目标，停用所有 active 命令控制器，不能只停 arm 而留下 active 的 gripper。
3. 排除原因，确认 PC 模式、七路新鲜反馈和可保持的当前姿态。
4. 如果串口仍正常，重新激活已加载的控制器：

```bash
ros2 control set_controller_state arm_controller active
```

成功 ENABLE 可清除历史通信/反馈故障。当前 CAN 或电机使能失败仍会阻止使能。再次握手失败时继续查原因，不循环重启掩盖问题；恢复后重新验证保持，并从新反馈生成目标。

### 13.3 USB 拔插后

设备重新出现不代表旧串口文件描述符已恢复。当前驱动不提供完整自动重连流程：退出实机 launch，重新确认路径/权限/占用后启动，再按流程激活控制器。

## 14. 常见问题速查

| 现象 | 优先检查 | 处理方向 |
|---|---|---|
| 无 ttyACM 设备 | 数据线、USB 枚举、板卡供电、虚拟机设备归属 | 先解决系统识别，不修改 ROS 协议 |
| Permission denied | dialout、是否重新登录、udev 权限 | 配置用户组和规则 |
| Package not found | 是否构建实机包、是否 source install_ubuntu | 用 `ros2 pkg prefix` 验证实际工作区 |
| Failed to open | 路径、权限、设备是否消失 | `ls -l`、`fuser`、系统日志 |
| 没有 /joint_states | 控制节点和状态广播器是否正常 | 查看 launch 输出、list_controllers |
| joint8 为 NaN | 是否完成夹爪标定 | 未标定时是预期行为，不填假值绕过 |
| 六轴为 NaN | STATE 或电机反馈是否过期 | 独占串口监视器检查年龄和掩码 |
| arm mapping has not been calibrated | Xacro 中标定状态和 sign/offset | 完成实测后重建；不是只改开关 |
| motor/URDF ranges do not overlap | sign、offset、轴对应关系 | 修正标定，不任意扩大模型限位 |
| ENABLE WRONG_MODE（0x0A） | 实际 mode 是否 10 | 检查拨杆、遥控输入、板上固件版本 |
| ENABLE NOT_READY（0x08） | 初始化、全部电机使能和反馈 | 查 ready、CAN/DM 状态、七路年龄 |
| OUT_OF_RANGE（0x04） | 任一路位置是否超出固件限位 | 检查电机/ROS 坐标及夹爪端点 |
| STALE（0x07） | 指令排队、主机时序、链路延迟 | 查 CPU 负载、串口占用和日志 |
| max_step exceeded | 相邻发送目标超过 0.01 rad | 降低轨迹速度并检查旧目标跳变，不先放宽保护 |
| PATH/GOAL_TOLERANCE_VIOLATED | 实际反馈是否跟得上规划 | 对比目标与反馈，检查速度、插补及标定 |
| STOP 后仍看到其他模式动作 | 是否已离开 PC 模式 | STOP 仅撤销 PC 控制，不能停止所有固件模式 |
| 重连后仍无反馈 | 旧串口句柄或设备路径变化 | 重启硬件 launch 并重新激活 |

固件故障位可组合：`0x0001` 链路超时、`0x0002` USB 断开、`0x0004` RX 溢出、`0x0008` 反馈丢失、`0x0010` CAN 总线、`0x0020` 电机使能失败。

## 15. 记录与下一次启动

发生问题时，至少保存 launch 日志、标定配置、实际设备路径、控制器状态、具体 goal 和结果。ROS 运行期间可以在第三个终端记录：

```bash
mkdir -p ~/arm_logs
ros2 bag record -o "$HOME/arm_logs/session_$(date +%Y%m%d_%H%M%S)" \
  /joint_states /arm_controller/controller_state
```

使用 `ros2 topic list` 核对实际控制器状态话题名称；仅录 bag 不会占用 USB 串口。原始 monitor 的 CSV 应在另一个独占串口阶段采集。

每个阶段记录“通过/失败/未执行”：

| 阶段 | 结果 | 证据 |
|---|---|---|
| USB 识别、权限和稳定路径 | 未执行 | 系统日志、设备路径 |
| Ubuntu 构建与测试 | 未执行 | colcon 输出与测试结果 |
| 原始串口遥测 | 未执行 | 30 s CSV、帧率与错误统计 |
| ROS 只读反馈 | 未执行 | joint_states、驱动日志 |
| 六轴方向/零位 | 未执行 | 两参考姿态记录和配置快照 |
| 首次使能保持 | 未执行 | 状态、姿态变化记录 |
| 六轴分别小步 | 未执行 | goal、反馈与执行结果 |
| 夹爪反馈/控制 | 未执行 | 标定端点与中间开度验证 |
| MoveIt 低速执行 | 未执行 | 规划与反馈记录 |
| 停止、故障恢复、重连 | 未执行 | 现象和日志 |

完成首次联调后，日常仍按“检查连接 → source 正确工作区 → 启动 → 确认反馈 → 激活控制器 → 确认保持 → 发送任务”的顺序运行。

相关文档：[代码阅读指南](src/CODE_GUIDE.zh-CN.md)、[实机驱动 README](src/dm_mc02_hardware/README.md)、[标定配置](src/dm_mc02_hardware/urdf/dm_mc02.ros2_control.xacro)。
