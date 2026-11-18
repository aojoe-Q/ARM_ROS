# ROS 2 Humble + MuJoCo

本包将 `test_robot` 机械臂接入 MuJoCo，复用现有 MoveIt、
`arm_controller`、`gripper_controller` 和 `joint_state_broadcaster`。
控制链路：MoveIt → FollowJointTrajectory → ros2_control → MuJoCo 位置执行器
→ 物理关节状态 → `/joint_states` → TF/RViz。

## 安装和构建

需要 ROS 2 Humble、现有工作空间的 MoveIt/ros2_control 依赖，以及同一 Python
环境中的 MuJoCo：

```bash
python3 -m pip install -r src/test_robot_mujoco/requirements.txt
source /opt/ros/humble/setup.bash
colcon build --base-paths src --packages-up-to test_robot_mujoco --symlink-install
source install/setup.bash
```

构建使用 Python wheel 中版本匹配的 C 头文件和 `libmujoco`，接口和模型生成流程
参考 `/home/yaq/mujoco_main`。不需要把 MuJoCo 源码复制进 ROS 工作空间；更新
MuJoCo wheel 后应重新构建本包。

构建时从原始 URDF/SRDF 生成 `model/test_robot.xml` 和 OBJ 网格；STL 到 OBJ
保持三角形几何，解决 Link2 超出 MuJoCo STL 面数上限的问题。原始文件不变。

## 启动

在 `/home/yaq/ros_test_8` 中运行：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch test_robot_mujoco mujoco.launch.py
```

默认启动 MuJoCo 后端、现有控制器、MoveIt、RViz 和 MuJoCo 状态查看窗口。
在 RViz 的 MotionPlanning 面板选择 `arm` 或 `gripper`，规划并执行轨迹。
不要同时启动原 mock 硬件 launch，否则控制器和 joint_states 会重名。

无图形界面运行：

```bash
ros2 launch test_robot_mujoco mujoco.launch.py use_rviz:=false use_viewer:=false
```

仅启动物理仿真和控制器，添加 `use_moveit:=false`。

```bash
ros2 control list_controllers
ros2 action send_goal /gripper_controller/follow_joint_trajectory control_msgs/action/FollowJointTrajectory "{trajectory: {joint_names: [joint8], points: [{positions: [0.01], time_from_start: {sec: 3}}]}}
```

## 仿真约定和局限

- 模型为当前工作区中的 `test_robot`，关节命名沿用其 URDF/SRDF。
- 物理步长 1 ms，控制器约 100 Hz；通过实际循环时间推进物理，使用墙钟，
  不发布 `/clock`。主机长时间卡顿时限制补算为 100 ms，避免无限追赶。
- `/joint_states` 发布 7 个受控关节；`/mujoco/joint_states` 额外发布全部
  16 个物理关节，供 MuJoCo 窗口查看实际联动状态。
- 夹爪 mimic 通过 MuJoCo 等式约束实现，含丝杆 3141.59265 rad/m 传动。
  约束为数值近似，RViz 中的 URDF mimic 则是精确运动学关系。
- 默认启用理想重力/科里奥利前馈补偿，便于轨迹调试；
  `gravity_compensation:=false` 可关闭。补偿额外施加广义力，
  不受位置执行器的力矩限制，不能据此评估真实电机负载能力。
- PD 增益、阻尼、转子惯量为仿真初始参数，尚未做实机参数辨识。
  执行器使用 URDF 的力/力矩限值；URDF 本身注明部分夹爪参数为占位值。
- 网格碰撞采用 MuJoCo 凸包近似，并沿用 SRDF 排除的碰撞对；
  地面位于 z=0，与固定底座底面齐平。没有添加抓取物体或验证抓取接触。
- MuJoCo 窗口是 ROS 状态查看器，鼠标拖拽物体不会向后端施加力；
  物理仿真只有 ros2_control 插件中的一份。

## 验证

```bash
colcon test --packages-select test_robot_mujoco --event-handlers console_direct+
colcon test-result --verbose
```

物理测试检查六轴位置跟踪、夹爪位移、联动约束及数值稳定性。
硬件集成测试通过 pluginlib 加载实际插件，检查初始化、激活、七关节指令与
状态反馈，并验证非有限数值指令会被拒绝。测试关闭额外状态发布以避免依赖 DDS。

本次执行环境中，完整 ROS 启动受 DDS 网络接口访问限制阻断
（`failed to enumerate interfaces for udp`），另有控制器 spawner 的 NumPy
导入失败。因此尚未验证完整 MoveIt 轨迹执行及 GUI；这两项需在本机正常
ROS 终端中运行上述启动命令确认。

## 关节力矩显示与画面同步

通过本包 launch 启动的 MuJoCo 窗口只新增六轴关节力矩表，10 Hz 刷新。
显示 joint1～joint6 的总驱动力矩，单位 N·m，计算为
qfrc_actuator + qfrc_applied，包含理想重力/科里奥利补偿，
不包含接触/约束反力，也不是实机传感器读数。joint8 是直线关节，
其驱动力单位为 N，不列入力矩表，仍可通过 ROS 话题读取。

窗口通过独立接收线程直接订阅 /mujoco/joint_states，按最新位置和速度
更新模型，不再等待 /mujoco/joint_targets 配对。后端时间戳重置不会
阻止更新。LIVE 表示正在接收，WAITING 表示尚未收到数据，STALE
表示超过 1 秒没有新数据，未知力矩显示 --。ROS 数据接收独立于渲染。

/joint_states 与 /mujoco/joint_states 的 effort 为总驱动力；
/mujoco/joint_targets 保留供外部调试，其 position 为控制目标，effort
为执行器输出。窗口不显示目标位置、位置误差或双列力矩。

如果 RViz 已执行而 MuJoCo 没有动作，请检查窗口是否显示 LIVE，以及
启动终端是否出现 Received ... joints。没有实时数据时，终端会每 5 秒
提示检查 MuJoCo 后端和 ROS domain。单独打开 XML 的原生 MuJoCo 窗口
不会接收 ROS 运动，必须使用本包 launch。

```bash
ros2 topic hz /mujoco/joint_states
ros2 topic echo /mujoco/joint_states
```

本次查阅到的 2026-10-21 18:34 启动日志确认后端加载 MuJoCo，
arm_controller 接收目标并返回成功，MoveIt 返回 SUCCEEDED。查看器已
移除双话题配对门槛，并以不提供目标数据的连续关节状态进行回归测试。
图形窗口中的最终同步效果仍需重启后确认。
