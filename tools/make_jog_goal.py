#!/usr/bin/env python3
"""Build a small single-joint FollowJointTrajectory goal from live /joint_states.

Only writes a JSON file; it never sends a goal. Based on REAL_HARDWARE_STEP_BY_STEP.zh-CN.md section 11.
python3 tools/make_jog_goal.py --joint 1 --delta 0.01 [--seconds 3] [--out /tmp/dm_mc02_jog_goal.json]
"""
import argparse
import json
import math
import runpy
import time
from pathlib import Path

import rclpy
import xacro
from ament_index_python.packages import get_package_share_directory
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState

p = argparse.ArgumentParser()
p.add_argument("--joint", type=int, required=True, choices=range(1, 7))
p.add_argument("--delta", type=float, required=True, help="rad, |delta| <= 0.05")
p.add_argument("--seconds", type=int, default=3)
p.add_argument("--out", type=Path, default=Path("/tmp/dm_mc02_jog_goal.json"))
args = p.parse_args()
if not 0 < abs(args.delta) <= 0.05:
    raise SystemExit("delta 必须在 (0, 0.05] rad 范围内")
if args.seconds < 2:
    raise SystemExit("seconds 至少为 2")

args.out.unlink(missing_ok=True)  # never leave a stale goal behind on failure
share = Path(get_package_share_directory("dm_mc02_hardware"))
xml = xacro.process_file(str(share / "urdf/test_robot_real.urdf.xacro"),
                         mappings={"allow_enable": "1"}).toxml()
adapt = runpy.run_path(str(share / "launch/real_model.py"))["adapt_description"]
xml, limits, params = adapt(xml)  # raises if mapping_calibrated=0

names = [f"joint{i}" for i in range(1, 7)]
rclpy.init()
node = rclpy.create_node("prepare_small_jog")
messages = []
node.create_subscription(JointState, "/joint_states", messages.append, qos_profile_sensor_data)
try:
    deadline = time.monotonic() + 3
    while not messages and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
    if not messages:
        raise SystemExit("3 秒内没有 /joint_states，未生成目标")
    values = dict(zip(messages[-1].name, messages[-1].position))
    current = [values.get(n, float("nan")) for n in names]
    if not all(math.isfinite(q) for q in current):
        raise SystemExit("反馈缺失或为 NaN，未生成目标")
    target = current.copy()
    target[args.joint - 1] += args.delta
    for n, start, end in zip(names, current, target):
        lo, hi = limits[n]["min_position"], limits[n]["max_position"]
        if not (lo <= start <= hi and lo <= end <= hi):
            raise SystemExit(f"{n} 当前值或目标超出实机模型范围 [{lo:.4f}, {hi:.4f}]")
        mcu = float(params[n + "_sign"]) * end + float(params[n + "_offset"])
        if not float(params[n + "_mcu_min"]) <= mcu <= float(params[n + "_mcu_max"]):
            raise SystemExit(f"{n} 目标超出电机侧限位")
    goal = {"trajectory": {"joint_names": names, "points": [
        {"positions": current, "time_from_start": {"sec": 0, "nanosec": 0}},
        {"positions": target, "time_from_start": {"sec": args.seconds, "nanosec": 0}}]},
        "goal_tolerance": [{"name": n, "position": 0.002} for n in names],
        "goal_time_tolerance": {"sec": 1, "nanosec": 0}}
    args.out.write_text(json.dumps(goal), encoding="utf-8")
    print("当前 rad:", [round(q, 4) for q in current])
    print("目标 rad:", [round(q, 4) for q in target])
    print("已生成（未发送）:", args.out)
finally:
    node.destroy_node()
    rclpy.shutdown()
