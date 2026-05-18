#!/usr/bin/env python3

"""
    ROS 2 node that reads RH8D sensor data (Ranger finder and Force sensors) from /dynamic_joint_states and prints formatted values.

    Note: An exact copy of this file is well-commented in the RH8D left hand version.
"""

import math

import rclpy
from rclpy.node import Node
from control_msgs.msg import DynamicJointState


class RH8DSensorReader(Node):
    def __init__(self):
        super().__init__('rh8d_sensor_reader')

        self.create_subscription(
            DynamicJointState,
            '/dynamic_joint_states',
            self.dynamic_joint_state_callback,
            10
        )

        self.last_print_time = self.get_clock().now()
        self.print_period_sec = 0.5

        self.get_logger().info('RH8D sensor reader started')
        self.get_logger().info('Listening on /dynamic_joint_states')

    def get_interface_value(self, msg: DynamicJointState, joint_name: str, interface_name: str):
        for i, name in enumerate(msg.joint_names):
            if name != joint_name:
                continue

            if i >= len(msg.interface_values):
                return None

            interface_block = msg.interface_values[i]

            for j, if_name in enumerate(interface_block.interface_names):
                if if_name == interface_name:
                    if j < len(interface_block.values):
                        return interface_block.values[j]
                    return None

        return None

    def get_vec3(self, msg: DynamicJointState, sensor_name: str):
        x = self.get_interface_value(msg, sensor_name, 'x')
        y = self.get_interface_value(msg, sensor_name, 'y')
        z = self.get_interface_value(msg, sensor_name, 'z')
        return x, y, z

    def dynamic_joint_state_callback(self, msg: DynamicJointState):
        now = self.get_clock().now()
        dt = (now - self.last_print_time).nanoseconds / 1e9
        if dt < self.print_period_sec:
            return
        self.last_print_time = now

        palm_range = self.get_interface_value(msg, 'palm_range', 'value')

        thumb_fx, thumb_fy, thumb_fz = self.get_vec3(msg, 'thumb_tip_force')
        index_fx, index_fy, index_fz = self.get_vec3(msg, 'index_tip_force')
        middle_fx, middle_fy, middle_fz = self.get_vec3(msg, 'middle_tip_force')
        ring_fx, ring_fy, ring_fz = self.get_vec3(msg, 'ring_tip_force')
        small_fx, small_fy, small_fz = self.get_vec3(msg, 'small_tip_force')

        self.get_logger().info('---------------- SENSOR VALUES ----------------')
        self.print_scalar('palm_range.value', palm_range)

        self.print_vec3('thumb_tip_force', thumb_fx, thumb_fy, thumb_fz)
        self.print_vec3('index_tip_force', index_fx, index_fy, index_fz)
        self.print_vec3('middle_tip_force', middle_fx, middle_fy, middle_fz)
        self.print_vec3('ring_tip_force', ring_fx, ring_fy, ring_fz)
        self.print_vec3('small_tip_force', small_fx, small_fy, small_fz)

    def print_scalar(self, label: str, value):
        if value is None:
            self.get_logger().warn(f'{label:<18} = NOT FOUND')
        else:
            self.get_logger().info(f'{label:<18} = {value:.6f}')

    def print_vec3(self, label: str, x, y, z):
        if x is None or y is None or z is None:
            self.get_logger().warn(f'{label:<18} = NOT FOUND')
        else:
            mag = math.sqrt(x * x + y * y + z * z)
            self.get_logger().info(
                f'{label:<18} = [{x:.6f}, {y:.6f}, {z:.6f}] |mag|={mag:.6f}'
            )


def main(args=None):
    rclpy.init(args=args)
    node = RH8DSensorReader()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()