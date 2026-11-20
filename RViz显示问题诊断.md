# RViz 显示问题诊断报告

## 问题现象

RViz 启动后：
- 左侧 RobotModel 显示器状态为 "Status: Ok"
- 但 3D 视图中**完全没有机器人模型**（只有网格）
- 终端持续输出 TF 错误：`TF_NAN_INPUT` 和 `TF_DENORMALIZED_QUATERNION`

## 根本原因

### 1. 夹爪关节数据为 NaN

`/joint_states` 话题数据：
```
name:
- joint1, joint2, joint3, joint4, joint5, joint6  # 6轴关节
- joint8                                           # 夹爪关节

position:
- -0.0906, -0.008583, 0.011635, ...  # 前6个关节：正常数值 ✅
- .nan                                # joint8：NaN ❌
```

**原因：** `grip_calibrated=0`（夹爪未标定），硬件接口无法将电机角度转换为 URDF 坐标系的位置，返回 NaN。

### 2. 夹爪连杆的 TF 变换链断裂

URDF 结构：
```
joint6 → gripper_center_link → [joint8] → Link8 → Link10 → Link11 → Link12 → Link13 → Link14 → Link15 → Link16
                                   ↑
                                 NaN 位置
```

- `Link10`-`Link16` 都是 **mimic joint**（从动关节），它们的位置由 `joint8` 计算：
  ```xml
  <mimic joint="joint8" multiplier="-17.5101" offset="0" />
  ```

- 当 `joint8 = .nan` 时，所有从动关节的位置也变成 NaN
- TF 树无法用 NaN 值计算坐标变换
- `robot_state_publisher` 拒绝发布这些 TF，导致夹爪部分的几何体无法渲染

### 3. 为什么整个模型都不显示？

虽然只有夹爪有问题，但 RViz 的 RobotModel 显示器采用**全有或全无**的渲染策略：
- 如果 TF 树不完整（任何一个 link 缺失），整个模型都不会显示
- 这是为了避免显示不一致的、部分缺失的机器人状态

## 技术细节

### 硬件接口代码（dm_mc02_system.cpp）

```cpp
if (grip_calibrated_ == 0) {
    // 未标定：无法转换，返回 NaN
    state_interfaces_[6].set_value(std::numeric_limits<double>::quiet_NaN());
} else {
    // 已标定：线性映射 grip_motor → [0, 1]
    double norm = (grip_motor - grip_mcu_closed_) / (grip_mcu_open_ - grip_mcu_closed_);
    state_interfaces_[6].set_value(norm);
}
```

### TF 错误日志

```
Error: TF_NAN_INPUT: Ignoring transform for child_frame_id "Link10" 
       because of a nan value in the transform (nan nan nan) (nan nan nan nan)

Error: TF_DENORMALIZED_QUATERNION: Ignoring transform for child_frame_id "Link10"
       because of an invalid quaternion (nan nan nan nan)
```

## 解决方案

有 3 种方法可以让 RViz 显示模型：

### 方案 A：临时设置 grip_calibrated=1（推荐用于验证显示）

修改 `dm_mc02.ros2_control.xacro`：
```xml
<param name="grip_calibrated">1</param>  <!-- 改为 1 -->
<param name="grip_mcu_open">0.0</param>   <!-- 临时值 -->
<param name="grip_mcu_closed">-6.0</param><!-- 临时值 -->
```

重新编译并启动：
```bash
colcon build --packages-select dm_mc02_hardware
source install/setup.bash
ros2 launch dm_mc02_hardware bringup.launch.xml allow_enable:=0 device:=/dev/ttyACM1
```

**效果：** joint8 会输出一个临时的归一化值（可能不准确），但足够让 TF 树完整，RViz 可以显示模型。

### 方案 B：完成夹爪标定（第 8 节）

1. 手动闭合夹爪到极限位置，记录 `grip_motor` 值 → `grip_mcu_closed`
2. 手动张开夹爪到极限位置，记录 `grip_motor` 值 → `grip_mcu_open`
3. 写入配置并设置 `grip_calibrated=1`

**效果：** joint8 输出准确的归一化位置 [0, 1]，RViz 显示准确的夹爪开度。

### 方案 C：跳过 RViz 可视化

RViz 只是辅助工具，**不影响实际控制**。可以：
- 直接进行第 7 节（六轴关节映射标定）
- 标定完成后，joint1-6 的显示会准确
- 第 8 节完成夹爪标定后，整个模型会完整显示

## 当前状态总结

✅ **正常部分：**
- 串口通信（`/dev/ttyACM1`，`feedback=0x7f`，100Hz）
- joint1-6 数据（位置、速度、力矩都正常）
- `/robot_description` 话题（URDF 已发布）
- `/joint_states` 话题（数据正常，除了 joint8）

❌ **待解决：**
- joint8 = `.nan`（`grip_calibrated=0`）
- Link10-16 的 TF 变换失败
- RViz 无法显示任何模型

## 建议

**推荐流程：**
1. 临时设置 `grip_calibrated=1`（方案 A）→ 验证 RViz 能显示
2. 进行第 7 节：六轴关节映射标定
3. 进行第 8 节：夹爪标定（获取准确的 `grip_mcu_open` 和 `grip_mcu_closed`）
4. 所有标定完成后，系统完全正常

或者直接跳过 RViz 验证，进入第 7 节标定流程。
