#!/usr/bin/env python3
"""实时显示关节位置，用于标定测量"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

class JointMonitor(Node):
    def __init__(self):
        super().__init__('joint_monitor')
        self.subscription = self.create_subscription(
            JointState, '/joint_states', self.callback, 10)
        self.get_logger().info('开始监听关节位置...')
        self.get_logger().info('按 Ctrl+C 停止')

    def callback(self, msg):
        print('\n' + '='*60)
        for i, (name, pos) in enumerate(zip(msg.name, msg.position)):
            if name.startswith('joint') and name != 'joint8':
                print(f'{name}: {pos:9.6f} rad')
        print('='*60)

def main():
    rclpy.init()
    monitor = JointMonitor()
    try:
        rclpy.spin(monitor)
    except KeyboardInterrupt:
        pass
    finally:
        monitor.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
