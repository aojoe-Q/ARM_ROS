import os

import mujoco
import numpy as np


def test_physics_tracking_and_mimic():
    model = mujoco.MjModel.from_xml_path(os.environ['MUJOCO_TEST_MODEL'])
    data = mujoco.MjData(model)
    assert model.nu == 7
    target = np.array([0.2, 0.3, 0.2, 0.1, 0.1, 0.1, 0.01])
    addresses = [model.jnt_qposadr[model.actuator_trnid[i, 0]] for i in range(model.nu)]
    for step in range(5000):
        data.ctrl[:] = target * min(step / 2000, 1.0)
        mujoco.mj_forward(model, data)
        data.qfrc_applied[:] = data.qfrc_bias
        mujoco.mj_step(model, data)
    assert np.isfinite(data.qpos).all()
    assert not data.warning.number.any()
    np.testing.assert_allclose(data.qpos[addresses[:6]], target[:6], atol=0.01)
    assert abs(data.qpos[addresses[6]] - target[6]) < 0.001
    for index in range(model.neq):
        if model.eq_type[index] != mujoco.mjtEq.mjEQ_JOINT:
            continue
        follower, master = model.eq_obj1id[index], model.eq_obj2id[index]
        offset, multiplier = model.eq_data[index, :2]
        expected = offset + multiplier * data.qpos[model.jnt_qposadr[master]]
        assert abs(data.qpos[model.jnt_qposadr[follower]] - expected) < 0.002
