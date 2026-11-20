import copy
from pathlib import Path
import runpy
import unittest
import xml.etree.ElementTree as ET

PACKAGE = Path(__file__).resolve().parents[1]
MODEL = runpy.run_path(str(PACKAGE / "launch/real_model.py"))


def source_model():
    # Assemble the real XML inputs without requiring xacro/ROS on the native test host.
    robot = ET.parse(PACKAGE.parent / "test_robot/urdf/test_robot.urdf").getroot()
    macro = ET.parse(PACKAGE / "urdf/dm_mc02.ros2_control.xacro").getroot()
    control = copy.deepcopy(macro.find("{http://www.ros.org/wiki/xacro}macro/ros2_control"))
    control.set("name", "dm_mc02")
    robot.append(control)
    params = {p.attrib["name"]: p for p in control.findall("hardware/param")}
    params["device"].text = "/dev/ttyACM0"
    params["allow_enable"].text = "0"
    return robot, params


class ModelTest(unittest.TestCase):
    def adapt(self, robot):
        return MODEL["adapt_description"](ET.tostring(robot, encoding="unicode"))

    def calibrated(self):
        robot, params = source_model()
        params["mapping_calibrated"].text = "1"
        # Synthetic calibration, never written into production defaults.
        params["joint2_sign"].text = params["joint3_sign"].text = "-1"
        return robot, params

    def test_default_telemetry(self):
        robot, _ = source_model()
        _, limits, params = self.adapt(robot)
        self.assertEqual(params["mapping_calibrated"], "0")
        self.assertEqual(limits["joint2"]["max_velocity"], 0.4)

    def test_uncalibrated_motion_rejected(self):
        robot, params = source_model()
        params["allow_enable"].text = "1"
        with self.assertRaisesRegex(ValueError, "sign/offset"):
            self.adapt(robot)

    def test_mapped_limits_intersect_model(self):
        robot, params = self.calibrated()
        xml, limits, _ = self.adapt(robot)
        self.assertEqual(limits["joint2"]["min_position"], 0)
        self.assertEqual(limits["joint2"]["max_position"], 1.89)
        self.assertAlmostEqual(limits["joint3"]["max_position"], 3.00016972)
        self.assertEqual(limits["joint4"]["min_position"], 0)
        self.assertEqual(limits["joint4"]["max_position"], 2.09)
        out = ET.fromstring(xml)
        for name, limit in limits.items():
            self.assertAlmostEqual(float(out.find(f"joint[@name='{name}']/limit").get("velocity")), limit["max_velocity"])

    def test_wrong_direction_has_empty_intersection(self):
        robot, params = self.calibrated()
        params["joint2_sign"].text = "1"
        with self.assertRaisesRegex(ValueError, "joint2.*overlap"):
            self.adapt(robot)

    def test_gripper_rate_in_motor_units(self):
        robot, params = self.calibrated()
        params["grip_calibrated"].text = "1"
        params["grip_mcu_closed"].text = "-6"
        xml, limits, _ = self.adapt(robot)
        scale = (0.009530 + 0.000523) / 6
        self.assertLessEqual(limits["joint8"]["max_velocity"], 0.4 * scale)
        self.assertLessEqual(limits["joint8"]["max_acceleration"], 0.8 * scale)

    def test_unknown_gripper_endpoints_rejected_if_marked_calibrated(self):
        robot, params = self.calibrated()
        params["grip_calibrated"].text = "1"
        with self.assertRaisesRegex(ValueError, "endpoints"):
            self.adapt(robot)

    def test_old_named_poses_removed(self):
        robot, _ = self.calibrated()
        xml, _, _ = self.adapt(robot)
        srdf = (PACKAGE.parent / "test_robot_moveit_config/config/test_robot.srdf").read_text(encoding="utf-8")
        filtered = ET.fromstring(MODEL["filter_named_states"](srdf, xml))
        names = {s.get("name") for s in filtered.findall("group_state")}
        self.assertIn("home", names)
        self.assertNotIn("pose_1", names)
        self.assertNotIn("pose_2", names)

    def test_defaults_match_firmware_snapshot(self):
        _, params = source_model()
        expected = [(-2.96054893, 2.81695309), (-2.05669899, 0), (-3.00016972, 0),
                    (0, 3.00567291), (-0.904316101, 1.40938965), (-0.650879326, 2.32408657)]
        for i, (lo, hi) in enumerate(expected, 1):
            self.assertEqual(float(params[f"joint{i}_mcu_min"].text), lo)
            self.assertEqual(float(params[f"joint{i}_mcu_max"].text), hi)


if __name__ == "__main__":
    unittest.main()
