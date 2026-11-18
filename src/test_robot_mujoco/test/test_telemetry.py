import importlib.util
import os
from pathlib import Path
from types import SimpleNamespace as NS

import mujoco
import numpy as np

spec = importlib.util.spec_from_file_location(
    'joint_telemetry', Path(__file__).parents[1] / 'scripts/joint_telemetry.py')
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


def message(sec, names, position, effort):
    return NS(header=NS(stamp=NS(sec=sec, nanosec=0)),
              name=names, position=position, velocity=[0.0] * len(names), effort=effort)


def test_torque_only_and_staleness():
    table = module.JointTelemetry(['joint1', 'joint2'])
    assert 'WAITING' in table.torque_text(now=0)
    table.accept(message(5, ['joint2', 'joint1'], [0.1, 0.2], [2.5, -1.25]), now=1)
    text = table.torque_text(now=1.1)
    assert 'joint1: -1.250' in text
    assert 'joint2: 2.500' in text
    assert 'Nm' in text and 'LIVE' in text
    assert 'Actual' not in text and 'Target' not in text
    assert 'STALE' in table.torque_text(now=3)


def test_motion_without_targets_or_effort_and_after_clock_reset():
    model = mujoco.MjModel.from_xml_path(os.environ['MUJOCO_TEST_MODEL'])
    data = mujoco.MjData(model)
    table = module.JointTelemetry(['joint1'])
    # No target messages are supplied. Even identical or reset timestamps must
    # not freeze the viewer when the physics state keeps arriving.
    poses = []
    for stamp, q in [(20, 0.0), (21, 0.2), (21, 0.4), (0, 0.6)]:
        table.accept(message(stamp, ['joint1'], [q], []), now=stamp)
        current, _ = table.snapshot()
        module.apply_joint_state(model, data, current)
        assert abs(data.qpos[model.joint('joint1').qposadr[0]] - q) < 1e-12
        poses.append(data.body('Link6').xpos.copy())
    assert np.linalg.norm(poses[-1] - poses[0]) > 0.01
    assert 'joint1: --' in table.torque_text(now=0)
