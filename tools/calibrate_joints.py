#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""六轴关节映射标定（简化版）：摆两个姿态，零算术。

原理（来自 dm_mc02_hardware/src/arm_link.cpp:141）
    q_ros = sign * (mcu - offset)

    1) 在 URDF 零位时 q_ros = 0，代入得 offset = mcu。
       —— offset 就是「零位时的电机读数」，与 sign 无关，读一次即可。
    2) sign 只取 ±1，只看符号：沿 URDF 正方向动一点点，
       mcu 变大 = +1，变小 = -1。不需要动 0.5 rad，不需要量角器。

用法（两步，第二步不能省）
    # 第一步：摆姿态，算 sign/offset（先不要 --mark）
    python3 tools/calibrate_joints.py --write
    colcon build --packages-select dm_mc02_hardware && source install/setup.bash
    ros2 launch dm_mc02_hardware bringup.launch.xml allow_enable:=0 use_rviz:=true device:=/dev/ttyACM1

    # 第二步：RViz 逐个对照，反向的自动取反
    python3 tools/calibrate_joints.py --verify --write
    #   重建后再跑一次 --verify，应全部回答 y
    python3 tools/calibrate_joints.py --mark-only     # 确认后放行使能

    # 只重测某几个关节
    python3 tools/calibrate_joints.py --verify --write --joint joint4,joint6

说明
    /joint_states 是「映射后」的值。脚本会读 xacro 里当前的 sign/offset，
    反算回原始电机角 mcu = sign*q + offset，所以可以反复运行、逐步修正。

    为什么必须做第二步：J1/J4/J5/J6 的 sign 填反了不会报任何错（行程自检
    只对 J2/J3 有效，因为它们的电机量程是单侧的），只会让模型镜像。
    RViz 对照是这四个关节唯一的兜底，而「模型是否同向」与视角无关，
    不像「逆时针」那样依赖你从哪边看。
"""

import argparse
import re
import sys
import threading
import time
import xml.etree.ElementTree as ET

JOINTS = [f"joint{i}" for i in range(1, 7)]

# 固件零点（RoboMaster_S27-main/APP/Arm_L_task.h），仅用于交叉核对
FIRMWARE_ZERO = {
    "joint1": 0.0, "joint2": -0.1, "joint3": -0.05,
    "joint4": 0.0, "joint5": 0.0, "joint6": 0.0,
}

XACRO_DEFAULT = "src/dm_mc02_hardware/urdf/dm_mc02.ros2_control.xacro"
URDF_DEFAULT = "src/test_robot/urdf/test_robot.urdf"


# ---------------------------------------------------------------- 配置读取
def read_xacro(path):
    text = open(path, encoding="utf-8").read()
    cfg = {}
    for j in JOINTS:
        for key in ("sign", "offset", "mcu_min", "mcu_max"):
            m = re.search(rf'<param name="{j}_{key}">([^<]+)</param>', text)
            if not m:
                sys.exit(f"[错误] {path} 里找不到 {j}_{key}")
            cfg[f"{j}_{key}"] = float(m.group(1))
    m = re.search(r'<param name="mapping_calibrated">([^<]+)</param>', text)
    cfg["mapping_calibrated"] = float(m.group(1)) if m else 0.0
    return cfg, text


def read_urdf_limits(path):
    limits = {}
    root = ET.parse(path).getroot()
    for j in JOINTS:
        node = root.find(f"joint[@name='{j}']/limit")
        if node is None:
            sys.exit(f"[错误] {path} 里 {j} 没有 <limit>")
        limits[j] = (float(node.get("lower")), float(node.get("upper")))
    return limits


def parse_joints(spec):
    """'joint4,joint6' / '4,6' / 'j4 j6' 都能解析；不传则返回全部 6 轴。"""
    if not spec:
        return list(JOINTS)
    out = []
    for part in re.split(r"[,\s]+", spec.strip()):
        if not part:
            continue
        name = part if part.startswith("joint") else "joint" + part.lstrip("jJ")
        if name not in JOINTS:
            sys.exit(f"[错误] 未知关节 {part!r}，可选：{', '.join(JOINTS)}")
        if name not in out:
            out.append(name)
    return out


def write_config(args, cfg, xacro_text, joints):
    text = xacro_text
    for j in joints:
        for key, val in (("sign", f"{cfg[f'{j}_sign']:.1f}"),
                         ("offset", f"{cfg[f'{j}_offset']:.6f}")):
            text, n = re.subn(rf'(<param name="{j}_{key}">)[^<]*(</param>)',
                              rf"\g<1>{val}\g<2>", text)
            if n != 1:
                sys.exit(f"[错误] 写回 {j}_{key} 失败（匹配 {n} 次）")
    if args.mark:
        text, n = re.subn(r'(<param name="mapping_calibrated">)[^<]*(</param>)',
                          r"\g<1>1\g<2>", text)
        if n != 1:
            sys.exit("[错误] 写回 mapping_calibrated 失败")
    open(args.xacro, "w", encoding="utf-8").write(text)
    print(f"\n已写回 {args.xacro}" + ("（mapping_calibrated=1）" if args.mark else ""))


# ---------------------------------------------------------------- RViz 对照验证
def run_verify(args, cfg, xacro_text):
    """不依赖 ROS、不依赖方向判断：只看模型和实物是否同向，反向就把 sign 取反。

    这是对 J1/J4/J5/J6 唯一的兜底 —— 这四个关节 sign 填反了，
    行程自检不会报错，只会让模型镜像。
    """
    targets = args.joints
    print("\n【RViz 对照验证】判断“同向/反向”与视角无关，不存在逆时针从哪边看的问题。")
    print("前提：xacro 已是标定后的值、已 colcon build、已启动")
    print("      ros2 launch dm_mc02_hardware bringup.launch.xml allow_enable:=0 use_rviz:=true device:=/dev/ttyACM1")
    print("做法：每次只慢慢推一个关节 5~10 度，盯着 RViz 里的模型。\n")

    print(f"{'关节':<10}{'当前 sign':>12}{'当前 offset':>14}")
    print("-" * 40)
    for j in targets:
        print(f"{j:<10}{cfg[f'{j}_sign']:>+12.1f}{cfg[f'{j}_offset']:>14.4f}")
    print("-" * 40)
    if cfg.get("mapping_calibrated", 0.0) > 0.5:
        print("⚠ 当前 mapping_calibrated=1：若手臂已使能，它会真的跟着动！")
        print("  请先停掉，用 allow_enable:=0 重启再验证。\n")
    else:
        print("mapping_calibrated=0，只读模式，安全。\n")

    flipped, quit_early = [], False
    for j in targets:
        while True:
            ans = input(f"  {j}: RViz 模型与实物 [y]同向 [n]反向 [s]跳过 [q]退出 > ")
            ans = ans.strip().lower()
            if ans in ("y", "n", "s", "q"):
                break
            print("    请输入 y / n / s / q")
        if ans == "q":
            quit_early = True
            print("  提前退出，已答的部分会保留。")
            break
        if ans == "s":
            print(f"    {j} 跳过，保持 {cfg[f'{j}_sign']:+.0f}")
            continue
        if ans == "n":
            cfg[f"{j}_sign"] *= -1.0
            flipped.append(j)
            print(f"    → {j}_sign 取反为 {cfg[f'{j}_sign']:+.0f}")

    print("\n" + "=" * 78)
    print(f"{'关节':<10}{'sign':>7}{'offset':>12}")
    print("-" * 78)
    for j in targets:
        print(f"{j:<10}{cfg[f'{j}_sign']:>+7.0f}{cfg[f'{j}_offset']:>12.4f}")
    print("=" * 78)

    if flipped:
        print(f"\n取反了 {len(flipped)} 个关节：{', '.join(flipped)}")
        print("这些改动要重建后才在 RViz 里生效，重建后再跑一次 --verify 应全部为 y。")
    elif not quit_early:
        print("\n全部同向，sign 没问题。")
    if quit_early:
        print("注意：本次没走完，未答的关节保持原值。")

    if args.write:
        write_config(args, cfg, xacro_text, targets)
        print("下一步：\n  colcon build --packages-select dm_mc02_hardware\n"
              "  source install/setup.bash")
        if flipped and args.mark:
            print("  ⚠ 本次有取反、又置了 mapping_calibrated=1。建议先重建、")
            print("    再跑一次 --verify（不带 --mark）确认全 y，之后才使能。")
        elif flipped:
            print("  重建后再起一次只读+RViz，跑 --verify 确认全为 y；")
            print("  确认无误后加 --mark 重跑（或直接改 mapping_calibrated=1）。")
        elif args.mark:
            print("  ./start_hardware.sh")
        else:
            print("  确认无误后加 --mark 重跑，把 mapping_calibrated 置 1。")
    else:
        print(f"\n未写回。确认后加 --write 重跑，或手动改 {args.xacro}")


# ---------------------------------------------------------------- 读数采集
class Monitor:
    def __init__(self):
        import rclpy
        from rclpy.node import Node
        from sensor_msgs.msg import JointState

        self._rclpy = rclpy
        rclpy.init()
        self.node = Node("calibrate_joints")
        self.latest = None
        self.node.create_subscription(JointState, "/joint_states", self._cb, 10)

    def _cb(self, msg):
        data = dict(zip(msg.name, msg.position))
        self.latest = {j: data.get(j) for j in JOINTS}

    def spin(self, seconds):
        self._rclpy.spin_once(self.node, timeout_sec=seconds)

    def wait_enter(self, prompt):
        print(prompt, flush=True)
        done = threading.Event()
        threading.Thread(target=lambda: (input(), done.set()), daemon=True).start()
        while not done.is_set():
            self.spin(0.05)

    def capture(self, seconds=0.8):
        """平均若干帧，抑制抖动。"""
        acc = {j: [] for j in JOINTS}
        deadline = time.time() + seconds
        while time.time() < deadline:
            self.spin(0.05)
            if self.latest and all(v is not None for v in self.latest.values()):
                for j in JOINTS:
                    acc[j].append(self.latest[j])
        if not acc["joint1"]:
            sys.exit("[错误] 没收到 /joint_states，确认 bringup 已启动、"
                     "joint_state_broadcaster 已加载")
        return {j: sum(v) / len(v) for j, v in acc.items()}

    def wait_ready(self, timeout=15.0):
        deadline = time.time() + timeout
        while time.time() < deadline:
            self.spin(0.05)
            if self.latest and all(v is not None for v in self.latest.values()):
                return True
        sys.exit(f"[错误] {timeout:.0f} 秒内没收到完整的 /joint_states，"
                 f"确认 bringup 已启动、joint_state_broadcaster 已加载")

    def shutdown(self):
        self.node.destroy_node()
        self._rclpy.shutdown()


# ---------------------------------------------------------------- 主流程
def main():
    ap = argparse.ArgumentParser(description="六轴关节映射标定（两姿态法）")
    ap.add_argument("--xacro", default=XACRO_DEFAULT)
    ap.add_argument("--urdf", default=URDF_DEFAULT)
    ap.add_argument("--write", action="store_true", help="把结果写回 xacro")
    ap.add_argument("--mark", action="store_true", help="同时置 mapping_calibrated=1")
    ap.add_argument("--min-delta", type=float, default=0.02,
                    help="判定方向所需的最小读数变化（rad），默认 0.02")
    ap.add_argument("--verify", action="store_true",
                    help="RViz 对照验证 sign（不连 ROS，纯问答；反向自动取反）")
    ap.add_argument("--mark-only", action="store_true",
                    help="只把 mapping_calibrated 置 1，不做任何提问")
    ap.add_argument("--joint", default="",
                    help="只处理指定关节，如 joint4,joint6 或 4,6（默认全部）")
    args = ap.parse_args()
    args.joints = parse_joints(args.joint)

    cfg, xacro_text = read_xacro(args.xacro)

    if args.mark_only:
        args.mark = True
        write_config(args, cfg, xacro_text, [])
        print("mapping_calibrated=1 —— 现在可以 ./start_hardware.sh 使能了。")
        return

    if args.verify:
        run_verify(args, cfg, xacro_text)
        return

    urdf_limits = read_urdf_limits(args.urdf)

    mon = Monitor()
    try:
        print("等待 /joint_states ...", flush=True)
        mon.wait_ready()

        # ---- 姿态 A：URDF 零位
        mon.wait_enter(
            "\n【姿态 A】把机械臂摆到 URDF 零位：\n"
            "  joint1 底座朝正前方 / joint2 大臂水平 / joint3 小臂与大臂成直线\n"
            "  joint4~6 腕部中位\n"
            "  注意：只读模式下电机不出力，大臂会自重下落，请托住。\n"
            "摆好后按回车 ...")
        q_a = mon.capture()
        mcu_a = {j: cfg[f"{j}_sign"] * q_a[j] + cfg[f"{j}_offset"] for j in JOINTS}
        print("  零位读数: " + "  ".join(f"{j[-1]}={mcu_a[j]:+.4f}" for j in JOINTS))

        # ---- 姿态 B：每个关节都朝 URDF 正方向动一点
        mon.wait_enter(
            "\n【姿态 B】把 6 个关节同时朝 URDF 正方向轻推（2~3 度即可，不用量角度）：\n"
            "  joint1 从上往下看逆时针 / joint2、joint3 向上抬\n"
            "  joint4 绕小臂轴逆时针 / joint5 向上抬（俯仰） / joint6 绕法兰轴逆时针\n"
            "摆好后按回车 ...")
        q_b = mon.capture()
        mcu_b = {j: cfg[f"{j}_sign"] * q_b[j] + cfg[f"{j}_offset"] for j in JOINTS}
        print("  姿态B读数: " + "  ".join(f"{j[-1]}={mcu_b[j]:+.4f}" for j in JOINTS))
    finally:
        mon.shutdown()

    # ---- 计算
    result, warnings = {}, []
    for j in JOINTS:
        d = mcu_b[j] - mcu_a[j]
        sign = 1.0 if d >= 0 else -1.0
        offset = mcu_a[j]
        result[j] = (sign, offset, d)
        if abs(d) < args.min_delta:
            warnings.append(f"{j} 只动了 {d:+.4f} rad，方向不可信 —— 重跑一次，"
                            f"或手动填 sign")

    # ---- 报告
    print("\n" + "=" * 78)
    print(f"{'关节':<8}{'零位读数':>12}{'姿态B读数':>12}{'Δ':>10}{'sign':>7}{'offset':>12}")
    print("-" * 78)
    for j in JOINTS:
        sign, offset, d = result[j]
        print(f"{j:<8}{mcu_a[j]:>12.4f}{mcu_b[j]:>12.4f}{d:>+10.4f}"
              f"{sign:>+7.0f}{offset:>12.4f}")
    print("=" * 78)

    # ---- 交叉核对：零位读数 vs 固件零点
    drift = max(abs(mcu_a[j] - FIRMWARE_ZERO[j]) for j in JOINTS)
    if drift < 0.15:
        print(f"零位读数与固件零点吻合（最大偏差 {drift:.3f} rad）→ "
              f"说明固件零点位姿就是 URDF 零位，结果可信。")
    else:
        print(f"注意：零位读数与固件零点最大差 {drift:.3f} rad。"
              f"若零位姿态没摆准，offset 会整体偏移，请复核姿态 A。")

    # ---- 行程自检（代替原来的 0.5 rad 验证）
    print(f"\n{'关节':<8}{'电机量程':>22}{'映射后 ROS 量程':>24}{'∩URDF':>20}{'可用行程':>10}")
    print("-" * 78)
    bad = []
    for j in JOINTS:
        sign, offset, _ = result[j]
        lo, hi = cfg[f"{j}_mcu_min"], cfg[f"{j}_mcu_max"]
        mapped = sorted((sign * (lo - offset), sign * (hi - offset)))
        ulo, uhi = urdf_limits[j]
        lo_i, hi_i = max(ulo, mapped[0]), min(uhi, mapped[1])
        w = max(0.0, hi_i - lo_i)
        print(f"{j:<8}[{lo:>7.3f},{hi:>7.3f}]{'':>3}"
              f"[{mapped[0]:>8.3f},{mapped[1]:>8.3f}]{'':>5}"
              f"[{lo_i:>7.3f},{hi_i:>7.3f}]{'':>2}{w:>9.3f}")
        # 理论上限 = min(URDF 宽度, 电机行程)；可用行程不到一半基本就是 sign 反了
        expect = min(uhi - ulo, hi - lo)
        if w < 0.2 or w < 0.5 * expect:
            bad.append(f"{j} 可用行程 {w:.3f} rad（理论可达 {expect:.3f}）—— 多半是 sign 反了")
    print("-" * 78)
    if bad:
        print("行程异常：")
        for b in bad:
            print("  ✗ " + b)
    else:
        print("行程自检通过（各关节可用行程都正常）。")

    if warnings:
        print("\n方向存疑：")
        for w in warnings:
            print("  ! " + w)

    # ---- 生成配置块
    block = ["<!-- 由 tools/calibrate_joints.py 生成 -->"]
    for j in JOINTS:
        sign, offset, _ = result[j]
        block.append(f'<param name="{j}_sign">{sign:.1f}</param>'
                     f'<param name="{j}_offset">{offset:.6f}</param>')
    print("\n" + "\n".join(block))

    # ---- 写回（--joint 指定时只写这些关节，其余保持现值）
    if args.write:
        for j in args.joints:
            cfg[f"{j}_sign"], cfg[f"{j}_offset"], _ = result[j]
        write_config(args, cfg, xacro_text, args.joints)
        if args.joints != JOINTS:
            print(f"只写回了 {', '.join(args.joints)}，其余关节未改动。")
        print("下一步：\n  colcon build --packages-select dm_mc02_hardware\n"
              "  source install/setup.bash\n"
              "  ros2 launch dm_mc02_hardware bringup.launch.xml allow_enable:=0 use_rviz:=true device:=/dev/ttyACM1\n"
              "  python3 tools/calibrate_joints.py --verify --write   # 逐个确认同向\n"
              "  # 全部同向后再 python3 tools/calibrate_joints.py --mark-only")
    else:
        print(f"\n未写回。确认无误后加 --write 重跑，或手动粘贴到 {args.xacro}")


if __name__ == "__main__":
    main()
