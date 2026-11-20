"""Build real-hardware limits from the same sign/offset parameters as the driver.

Pure standard-library functions so model checks can run without ROS.
"""
import math
import xml.etree.ElementTree as ET


def adapt_description(xml):
    root = ET.fromstring(xml)
    hardware = root.find("ros2_control/hardware")
    if hardware is None:
        raise ValueError("Missing real hardware configuration")
    params = {}
    for param in hardware.findall("param"):
        name = param.attrib["name"]
        if name in params:
            raise ValueError(f"Duplicate hardware parameter: {name}")
        params[name] = param.text.strip()

    def number(name):
        value = float(params[name])
        if not math.isfinite(value):
            raise ValueError(f"Non-finite hardware parameter: {name}")
        return value

    calibrated = number("mapping_calibrated") > 0.5
    grip_calibrated = number("grip_calibrated") > 0.5
    if number("allow_enable") > 0.5 and not calibrated:
        raise ValueError("Measure joint sign/offset in dm_mc02.ros2_control.xacro and set mapping_calibrated=1 first")
    planning = {}
    for i in range(1, 7):
        name = f"joint{i}"
        limit = root.find(f"joint[@name='{name}']/limit")
        if limit is None:
            raise ValueError(f"Missing URDF limits: {name}")
        entry = dict(has_velocity_limits=True, max_velocity=0.4,
                     has_acceleration_limits=True, max_acceleration=0.8)
        limit.set("velocity", "0.4")
        if calibrated:
            sign, offset = number(name + "_sign"), number(name + "_offset")
            if sign not in (-1, 1):
                raise ValueError(f"Invalid sign for {name}")
            lo, hi = number(name + "_mcu_min"), number(name + "_mcu_max")
            if lo >= hi:
                raise ValueError(f"Invalid motor limits for {name}")
            # MoveIt must respect BOTH model limits and calibrated motor travel.
            mapped = sorted((sign * (lo - offset), sign * (hi - offset)))
            lower = max(float(limit.attrib["lower"]), mapped[0])
            upper = min(float(limit.attrib["upper"]), mapped[1])
            if upper - lower < 1e-6:
                raise ValueError(f"{name}: motor/URDF ranges do not overlap; check sign and offset")
            limit.set("lower", str(lower))
            limit.set("upper", str(upper))
            entry.update(has_position_limits=True, min_position=lower, max_position=upper)
        planning[name] = entry
    grip_limit = root.find("joint[@name='joint8']/limit")
    grip_velocity, grip_acceleration = 0.001, 0.002
    if grip_calibrated:
        mc, mo = number("grip_mcu_closed"), number("grip_mcu_open")
        qc, qo = number("grip_q_close_m"), number("grip_q_open_m")
        if abs(mc - mo) < 1e-6 or qo <= qc:
            raise ValueError("Invalid gripper calibration endpoints")
        scale = abs((qo - qc) / (mo - mc))
        grip_velocity, grip_acceleration = min(0.001, 0.4 * scale), min(0.002, 0.8 * scale)
        lower = max(float(grip_limit.attrib["lower"]), qc)
        upper = min(float(grip_limit.attrib["upper"]), qo)
        if upper <= lower:
            raise ValueError("Gripper calibration does not overlap URDF")
        grip_limit.set("lower", str(lower))
        grip_limit.set("upper", str(upper))
    grip_limit.set("velocity", str(grip_velocity))
    planning["joint8"] = dict(has_velocity_limits=True, max_velocity=grip_velocity,
                              has_acceleration_limits=True, max_acceleration=grip_acceleration)
    return ET.tostring(root, encoding="unicode"), planning, params


def filter_named_states(srdf, description):
    """Omit old named poses outside the real robot's mapped range."""
    root, robot = ET.fromstring(srdf), ET.fromstring(description)
    bounds = {j.attrib["name"]: j.find("limit").attrib for j in robot.findall("joint")
              if j.find("limit") is not None}
    for state in list(root.findall("group_state")):
        for j in state.findall("joint"):
            b = bounds.get(j.attrib["name"])
            if b and not float(b["lower"]) <= float(j.attrib["value"]) <= float(b["upper"]):
                root.remove(state)
                break
    return ET.tostring(root, encoding="unicode")
