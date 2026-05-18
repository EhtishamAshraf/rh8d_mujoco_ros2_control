#!/usr/bin/env python3

"""
ROS2 based RH8D robotic hand controller:

-   Uses forward_command_controller
-   Read sensor/joint data from a robotic hand
-   Detect whether an object is present
-   Close the fingers gradually
-   Detect contact forces on fingertips
-   Stop increasing force once a stable grasp is achieved
-   Execute a sequence of wrist/hand gestures automatically
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor, ExternalShutdownException
from std_msgs.msg import Float64MultiArray
from control_msgs.msg import DynamicJointState


class RH8DGestureController(Node):
    def __init__(self):
        super().__init__('rh8d_object_pick')

        # State variables
        self.have_state = False
        self.latest_dynamic_state = None

        # Subscribe to the dynamic joint state topic to receive real-time sensor/interface data from the RH8D hand
        self.create_subscription(
            DynamicJointState,
            '/dynamic_joint_states',
            self.dynamic_state_callback,
            10
        )
        # Publisher used to send actuator position commands to the RH8D forward position controller
        self.command_pub = self.create_publisher(
            Float64MultiArray,
            '/hand_forward_position_controller/commands',
            10
        )

        # Stable contact detection: Each finger tracks how many consecutive cycles contact was detected
        self.contact_counter = {
            'thumb': 0,
            'index': 0,
            'middle': 0,
            'ring': 0,
            'small': 0,
        }
        self.required_contact_cycles = 5

        # Rangefinder threshold
        self.range_threshold = 0.25

        # Force threshold
        self.tip_force_threshold = 0.5

        # Close values
        self.thumb_flexion = 1.57
        self.finger_flexion = 1.57

        # Per-actuator command rates
        self.rate_forearm = 0.5
        self.rate_palm_axis = 0.5
        self.rate_palm_l = 0.5
        self.rate_thumb_axis = 0.5

        self.rate_index = 0.35
        self.rate_middle = 0.35
        self.rate_thumb = 0.35
        self.rate_ring_small = 0.35

        # Order: [forearm, palm_axis, palm_l, thumb_axis, index_tendon, middle_tendon, ring_small_tendon, thumb_tendon]
        self.current_command = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

        # Latch tendon commands after first stable grasp contact so, no more force is applied on the object
        self.grasp_locked = False
        self.latched_tendon_command = {
            'index': 0.0,
            'middle': 0.0,
            'ring_small': 0.0,
            'thumb': 0.0,
        }

        # Pre-defined Movements which the RH8D hand will follow:
        self.gestures = [
            {
                "name": "Hand Open with Thumb Adduction",
                "positions": [0.0, 0.0, 0.0, 1.57, 0.0, 0.0, 0.0, 0.0],
                "checks": [
                    ('Thumb_axis--palmL:1', 1.57),
                ]
            },
            {
                "name": "Hand Close",
                "positions": [0.0, 0.0, 0.0, 1.57,
                              self.finger_flexion, self.finger_flexion, self.finger_flexion, self.thumb_flexion],
                "require_object": True,
                "use_contact": True
            },
            {
                "name": "Wrist Rotation (-)",
                "positions": [-1.57, 0.0, 0.0, 1.57,
                              self.finger_flexion, self.finger_flexion, self.finger_flexion, self.thumb_flexion],
                "checks": [
                    ('forearm:1--base:1', -1.57),
                ]
            },
            # comment out "Wrist Adduction (-)" when picking up the ball
            # {
            #     "name": "Wrist Adduction (-)",
            #     "positions": [-1.57, -0.8, 0.0, 1.57, self.thumb_flexion, self.finger_flexion, self.finger_flexion, self.finger_flexion],
            #     "checks": [
            #         ('palm_axis:1--forearm:1', -0.8),
            #     ]
            # },
            {
                "name": "Wrist Adduction (+)",
                "positions": [-1.57, 0.8, 0.0, 1.57,
                              self.finger_flexion, self.finger_flexion, self.finger_flexion, self.thumb_flexion],
                "checks": [
                    ('palm_axis:1--forearm:1', 0.8),
                ]
            },
            {
                "name": "Wrist Flexion (-)",
                "positions": [-1.57, 0.8, -0.6, 1.57,
                              self.finger_flexion, self.finger_flexion, self.finger_flexion, self.thumb_flexion],
                "checks": [
                    ('palmL:1--palm_axis:1', -0.6),
                ]
            },
            {
                "name": "Wrist Rotation (+)",
                "positions": [0.0, 0.0, 0.0, 1.57,
                              self.finger_flexion, self.finger_flexion, self.finger_flexion, self.thumb_flexion],
                "checks": [
                    ('forearm:1--base:1', 0.0),
                ]
            },
            {
                "name": "Hand Open",
                "positions": [0.0, 0.0, 0.0, 1.57, 0.0, 0.0, 0.0, 0.0],
                "checks": [
                    ('Thumb_Methacarpal--Thumb_axis', 0.0),
                    ('Index_Proximal--palmL:1', 0.0),
                    ('Middle_Proximal--palmL:1', 0.0),
                    ('Ring_Proximal--palmL:1', 0.0),
                ]
            }
        ]
        
        # Index of the current state/pose in the motion sequence
        self.state_index = 0
        self.state_sent = False
        self.start_time = self.get_clock().now()

        # Timer update period (50 ms = 20 Hz loop rate). The timer repeatedly calls self.step() to run the state machine
        self.timer_period = 0.05
        self.timer = self.create_timer(self.timer_period, self.step)

        self.get_logger().info("RH8DGestureController started...")

    # Callback function: saves the latest robot state
    def dynamic_state_callback(self, msg: DynamicJointState):
        self.latest_dynamic_state = msg
        self.have_state = True

    """
    Function to extract a specific interface value from a DynamicJointState message.

        It searches the DynamicJointState message and returns the value of a specific interface for a given joint/sensor.
        
        Example:
        joint_name     = 'thumb_tip_force'
        interface_name = 'x'
        
        The function works because:
        - msg.joint_names and msg.interface_values are parallel arrays
        - each joint/sensor has a matching interface block at the same index
        - each interface block contains interface names and their values
        
        Returns:
        float value if found, otherwise None
    """
    def get_interface_value(self, name: str, interface_name: str):
        if self.latest_dynamic_state is None:
            return None

        msg = self.latest_dynamic_state

        for i, joint_name in enumerate(msg.joint_names):
            if joint_name != name:
                continue

            if i >= len(msg.interface_values):
                return None

            interface_block = msg.interface_values[i]
            for j, if_name in enumerate(interface_block.interface_names):
                if if_name == interface_name and j < len(interface_block.values):
                    return interface_block.values[j]

        return None

    # Function to check if object is present or not (detection done by palm range finger)
    def object_present(self) -> bool:
        rf = self.get_interface_value('palm_range', 'value')
        if rf is None:
            return False
        return (rf > 0.0) and (rf < self.range_threshold)

    # Function to publish the joint commands
    def publish_command(self, cmd_values):
        msg = Float64MultiArray()
        msg.data = [float(v) for v in cmd_values]
        self.command_pub.publish(msg)

    # Function for gradual movement of the joints: current += rate * dt
    def move_towards(self, current: float, target: float, rate: float, dt: float) -> float:
        if current < target:
            return min(current + rate * dt, target)
        elif current > target:
            return max(current - rate * dt, target)
        return current
    
    """
    Workflow:
       1. Uses the current actuator positions as the starting point
       2. Smoothly ramps each actuator toward its target using move_towards()
       3. Applies individual movement rates for each actuator
       4. Optionally freezes selected actuators using freeze_mask
       5. Publishes the updated command values to the controller
       6. Checks whether all non-frozen actuators reached their targets
    """
    def ramp_command_towards(self, target_positions, dt, freeze_mask=None):
        if freeze_mask is None:
            freeze_mask = [False] * 8

        new_cmd = list(self.current_command)

        def move(index, target, rate):
            if freeze_mask[index]:
                return self.current_command[index] # return current value if actuator should be kept frozen
            return self.move_towards(self.current_command[index], target, rate, dt) # else move towards target

        new_cmd[0] = move(0, target_positions[0], self.rate_forearm)
        new_cmd[1] = move(1, target_positions[1], self.rate_palm_axis)
        new_cmd[2] = move(2, target_positions[2], self.rate_palm_l)
        new_cmd[3] = move(3, target_positions[3], self.rate_thumb_axis)

        new_cmd[4] = move(4, target_positions[4], self.rate_index)
        new_cmd[5] = move(5, target_positions[5], self.rate_middle)
        new_cmd[6] = move(6, target_positions[6], self.rate_ring_small)
        new_cmd[7] = move(7, target_positions[7], self.rate_thumb)

        self.current_command = new_cmd
        self.publish_command(new_cmd) # publish the commands to the joints

        reached = all(
            freeze_mask[i] or abs(new_cmd[i] - target_positions[i]) < 1e-6
            for i in range(8)
        )
        return reached

    # Function to get the current position of the joints
    def get_joint_value(self, joint_name: str) -> float:
        value = self.get_interface_value(joint_name, 'position')
        return 0.0 if value is None else value

    # Function to check if a particular joint has reached the target value or not
    def is_value_at_target(self, joint_name: str, target: float, tolerance: float = 0.03) -> bool:
        current = self.get_joint_value(joint_name)
        diff = abs(current - target)
        self.get_logger().info(
            f"difference for {joint_name}: {diff:.4f} (current: {current:.4f}, target: {target:.4f})"
        )
        return diff <= tolerance

    # Function to calculate the fingertip force magnitude 
    def get_tip_force_mag(self, sensor_name: str) -> float:
        x = self.get_interface_value(sensor_name, 'x')
        y = self.get_interface_value(sensor_name, 'y')
        z = self.get_interface_value(sensor_name, 'z')
        if x is None or y is None or z is None:
            return 0.0
        return math.sqrt(x * x + y * y + z * z)

    # Function to check if the grasp is stable: check if the applied force is above threshold and the contact is stable
    def is_grasp_contact_stable(self, finger_name: str) -> bool:
        tip_map = {
            'thumb': 'thumb_tip_force',
            'index': 'index_tip_force',
            'middle': 'middle_tip_force',
            'ring': 'ring_tip_force',
            'small': 'small_tip_force',
        }

        tip_contact = False

        if finger_name in tip_map:
            tip_mag = self.get_tip_force_mag(tip_map[finger_name])
            tip_contact = tip_mag > self.tip_force_threshold

        contact_now = tip_contact

        if contact_now:
            self.contact_counter[finger_name] += 1
        else:
            self.contact_counter[finger_name] = 0

        return self.contact_counter[finger_name] >= self.required_contact_cycles

    # Save the current tendon actuator positions so the current grasp can be maintained
    def latch_current_grasp(self):
        self.latched_tendon_command['index'] = self.current_command[4]
        self.latched_tendon_command['middle'] = self.current_command[5]
        self.latched_tendon_command['ring_small'] = self.current_command[6]
        self.latched_tendon_command['thumb'] = self.current_command[7]
        self.grasp_locked = True

        self.get_logger().info(
            f"Latched grasp | "
            f"index={self.latched_tendon_command['index']:.4f}, "
            f"middle={self.latched_tendon_command['middle']:.4f}, "
            f"ring_small={self.latched_tendon_command['ring_small']:.4f}, "
            f"thumb={self.latched_tendon_command['thumb']:.4f}"
        )
    
    # Maintain the current grasp by overriding tendon commands with the stored latched values.
    def apply_latched_tendons(self, positions):
        new_positions = list(positions)

        if self.grasp_locked:
            new_positions[4] = self.latched_tendon_command['index']
            new_positions[5] = self.latched_tendon_command['middle']
            new_positions[6] = self.latched_tendon_command['ring_small']
            new_positions[7] = self.latched_tendon_command['thumb']

        return new_positions
    
    """
    Main control loop executed periodically by the timer:
       1. Waits until valid sensor/state data is available
       2. Delays execution briefly after startup for system stability
       3. Sequentially executes predefined hand gestures
       4. Checks whether an object is present before grasp gestures
       5. Gradually moves actuators toward gesture target positions
       6. Monitors fingertip contact/force sensors during grasping
       7. Freezes individual fingers once stable contact is detected
       8. Latches and maintains the grasp after successful contact
       9. Determines when the current gesture is complete
      10. Advances to the next gesture in the sequence
    """
    def step(self):
        if not self.have_state:
            return

        if (self.get_clock().now() - self.start_time).nanoseconds < 1e9:
            return

        if self.state_index >= len(self.gestures):
            self.get_logger().info("All gestures completed.")
            if rclpy.ok():
                rclpy.shutdown()
            return

        gesture = self.gestures[self.state_index]

        if gesture["name"] in ["Hand Open with Thumb Adduction", "Hand Open"]:
            self.grasp_locked = False

        if gesture.get("require_object", False) and not self.object_present():
            palm_range = self.get_interface_value('palm_range', 'value')
            palm_range = -1.0 if palm_range is None else palm_range
            self.get_logger().info(
                f"Object not present | palm_range={palm_range:.4f}. Fingers will remain open."
            )
            safe_open = [0.0, 0.0, 0.0, 1.57, 0.0, 0.0, 0.0, 0.0]
            self.ramp_command_towards(safe_open, self.timer_period)
            return

        if not self.state_sent:
            self.get_logger().info(f"Gesture {self.state_index}: {gesture['name']}")
            self.state_sent = True
            self.contact_counter = {
                'thumb': 0,
                'index': 0,
                'middle': 0,
                'ring': 0,
                'small': 0,
            }

        if gesture.get("use_contact", False):
            thumb_contact = self.is_grasp_contact_stable('thumb')
            index_contact = self.is_grasp_contact_stable('index')
            middle_contact = self.is_grasp_contact_stable('middle')
            ring_contact = self.is_grasp_contact_stable('ring')
            small_contact = self.is_grasp_contact_stable('small')

            freeze_mask = [
                False,                         # forearm
                False,                         # palm_axis
                False,                         # palm_l
                False,                         # thumb_axis
                index_contact,                 # index_tendon
                middle_contact,                # middle_tendon
                (ring_contact or small_contact),   # ring_small_tendon
                thumb_contact,                 # thumb_tendon
            ]


            thumb_tip_mag = self.get_tip_force_mag('thumb_tip_force')
            index_tip_mag = self.get_tip_force_mag('index_tip_force')
            middle_tip_mag = self.get_tip_force_mag('middle_tip_force')
            ring_tip_mag = self.get_tip_force_mag('ring_tip_force')
            small_tip_mag = self.get_tip_force_mag('small_tip_force')

            palm_range = self.get_interface_value('palm_range', 'value')
            palm_range = -1.0 if palm_range is None else palm_range


            self.get_logger().info(
                f"Contacts | "
                f"thumb={thumb_contact}, index={index_contact}, middle={middle_contact}, "
                f"ring={ring_contact}, small={small_contact}, palm_range={palm_range:.4f} | "
                f"tip_mag: thumb={thumb_tip_mag:.6f}, index={index_tip_mag:.6f}, "
                f"middle={middle_tip_mag:.6f}, ring={ring_tip_mag:.6f}, small={small_tip_mag:.6f}"
            )

            target_positions = gesture["positions"]

            any_contact = (
                thumb_contact and
                index_contact and
                middle_contact and
                (ring_contact or
                small_contact)
            )

            if any_contact and not self.grasp_locked:
                self.latch_current_grasp()

            if self.grasp_locked:
                target_positions = self.apply_latched_tendons(target_positions)
                freeze_mask[4] = True
                freeze_mask[5] = True
                freeze_mask[6] = True
                freeze_mask[7] = True

            self.ramp_command_towards(
                target_positions,
                self.timer_period,
                freeze_mask=freeze_mask
            )

            done = self.grasp_locked

        else:
            target_positions = self.apply_latched_tendons(gesture["positions"])
            self.ramp_command_towards(target_positions, self.timer_period)

            done = all(
                self.is_value_at_target(joint_name, target)
                for joint_name, target in gesture["checks"]
            )

        if done:
            self.get_logger().info(f"{gesture['name']} complete.")
            self.state_index += 1
            self.state_sent = False


def main(args=None):
    rclpy.init(args=args)
    node = RH8DGestureController()
    ex = SingleThreadedExecutor()
    ex.add_node(node)

    try:
        while rclpy.ok():
            ex.spin_once(timeout_sec=0.1)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except RuntimeError as e:
        node.get_logger().warn(f"Stopping after runtime error: {e}")
    finally:
        try:
            ex.remove_node(node)
        except Exception:
            pass
        try:
            node.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()