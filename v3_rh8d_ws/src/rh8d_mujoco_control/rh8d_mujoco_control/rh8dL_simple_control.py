#!/usr/bin/env python3

"""
ROS 2 node that publishes actuator position commands to the RH8D hand forward position controller.
"""

import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray


class RH8DActuatorCommander(Node):
    def __init__(self):
        super().__init__('rh8d_actuator_commander')

        self.pub = self.create_publisher(
            Float64MultiArray,
            '/hand_forward_position_controller/commands',
            10
        )
        self.actuator_names = [
            'pos_forearm',
            'pos_palm_axis',
            'pos_palmL',
            'pos_thumb_axis',
            'pos_index_tendon',
            'pos_middle_tendon',
            'pos_ring_small_tendon',
            'pos_thumb_tendon',
        ]

        self.get_logger().info('RH8D actuator commander started')
        self.get_logger().info(f'Actuator order: {self.actuator_names}')

    def send_command(self, values):
        if len(values) != 8:
            raise ValueError(f'Expected 8 actuator values, got {len(values)}')

        msg = Float64MultiArray()
        msg.data = [float(v) for v in values]
        self.pub.publish(msg)
        self.get_logger().info(f'Sent: {msg.data}')


def main(args=None):
    rclpy.init(args=args)
    node = RH8DActuatorCommander()

    try:
        time.sleep(1.0)

        # Open pose
        node.send_command([
            0.0,   
            0.0,   
            0.0,   
            1.57,  
            0.0,   
            0.0,   
            0.0,   
            0.0,   
        ])
        time.sleep(3.0)

        # Close pose
        node.send_command([
            0.0,  
            0.0,   
            0.0,   
            1.57,  
            1.0,   
            1.0,   
            1.0,   
            1.0,   
        ])
        time.sleep(3.0)

        # Open again
        node.send_command([
            0.0,
            0.0,
            0.0,
            1.57,
            0.0,
            0.0,
            0.0,
            0.0,
        ])
        time.sleep(2.0)

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()