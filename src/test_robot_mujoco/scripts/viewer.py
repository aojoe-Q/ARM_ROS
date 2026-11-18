#!/usr/bin/env python3
"""Display live MuJoCo backend state and six-axis joint torque."""
import threading
import time

import mujoco
import mujoco.viewer
import rclpy
from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from joint_telemetry import JointTelemetry, apply_joint_state


class Viewer(Node):
    def __init__(self):
        super().__init__('mujoco_viewer')
        path = self.declare_parameter('model_path', '').value
        self.model = mujoco.MjModel.from_xml_path(path)
        self.data = mujoco.MjData(self.model)
        self.telemetry = JointTelemetry([f'joint{i}' for i in range(1, 7)])
        self.subscription = self.create_subscription(
            JointState, '/mujoco/joint_states', self.receive, qos_profile_sensor_data)
        self._first_message = True
        self.get_logger().info('Waiting for MuJoCo state on /mujoco/joint_states')

    def receive(self, message):
        self.telemetry.accept(message)
        if self._first_message:
            self.get_logger().info(
                f'Received {len(message.name)} joints; viewer follows physics state directly')
            self._first_message = False


def main():
    rclpy.init()
    node = Viewer()
    executor = SingleThreadedExecutor()
    executor.add_node(node)

    def receive_states():
        try:
            executor.spin()
        except ExternalShutdownException:
            pass

    receiver = threading.Thread(target=receive_states, name='mujoco-state-receiver', daemon=True)
    receiver.start()
    try:
        with mujoco.viewer.launch_passive(node.model, node.data) as viewer:
            with viewer.lock():
                viewer.cam.distance = 1.8
                viewer.cam.lookat[:] = [-0.15, 0, 0.1]
                viewer.cam.azimuth = 135
                viewer.cam.elevation = -20
            overlay_supported = hasattr(viewer, 'set_texts')
            if not overlay_supported:
                node.get_logger().warning(
                    'This MuJoCo version lacks torque overlays; state animation remains active')
            next_overlay = 0.0
            next_warning = time.monotonic() + 5.0
            while rclpy.ok() and viewer.is_running():
                message, received = node.telemetry.snapshot()
                if message is not None:
                    with viewer.lock():
                        apply_joint_state(node.model, node.data, message)
                # Update the passive renderer from physics state, without copying
                # large mesh/model data or waiting for a second ROS topic.
                viewer.sync(state_only=True)
                now = time.monotonic()
                if overlay_supported and now >= next_overlay:
                    viewer.set_texts((
                        mujoco.mjtFont.mjFONT_NORMAL, mujoco.mjtGridPos.mjGRID_TOPLEFT,
                        node.telemetry.torque_text(now), ''))
                    next_overlay = now + 0.1
                if now >= next_warning:
                    if received is None or now - received > 1.0:
                        node.get_logger().warning(
                            'No fresh /mujoco/joint_states: check the MuJoCo backend and ROS domain')
                    next_warning = now + 5.0
                time.sleep(0.01)
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown(timeout_sec=2.0)
        receiver.join(timeout=2.0)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
