"""Latest physics state and torque display, independent of target messages."""
import math
import threading
import time


class JointTelemetry:
    def __init__(self, names):
        self.names = list(names)
        self._lock = threading.Lock()
        self._message = None
        self._received_at = None

    def accept(self, message, now=None):
        # Arrival order is authoritative, including after a backend clock reset.
        # Target/effort availability must never gate position updates.
        with self._lock:
            self._message = message
            self._received_at = time.monotonic() if now is None else now

    def snapshot(self):
        with self._lock:
            return self._message, self._received_at

    def torque_text(self, now=None):
        now = time.monotonic() if now is None else now
        message, received = self.snapshot()
        if message is None:
            return 'Joint torque: WAITING for /mujoco/joint_states'
        age = max(0.0, now - received)
        status = 'LIVE' if age < 1.0 else f'STALE ({age:.1f}s old)'
        effort = dict(zip(message.name, message.effort))
        lines = [f'Joint torque [{status}]', 'Total drive torque incl. compensation [Nm]']
        for name in self.names:
            value = effort.get(name)
            if value is None or not math.isfinite(value):
                text = '--'
            else:
                rounded = round(value, 3)
                text = f'{0.0 if rounded == 0 else rounded:.3f}'
            lines.append(f'{name}: {text}')
        return '\n'.join(lines)


def apply_joint_state(model, data, message):
    """Apply measured joints only; never wait for a command or effort sample."""
    import mujoco

    for name, position in zip(message.name, message.position):
        index = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
        if index >= 0 and math.isfinite(position):
            data.qpos[model.jnt_qposadr[index]] = position
    for name, velocity in zip(message.name, message.velocity):
        index = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
        if index >= 0 and math.isfinite(velocity):
            data.qvel[model.jnt_dofadr[index]] = velocity
    mujoco.mj_forward(model, data)
