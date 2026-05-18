import os
import xacro

from launch import LaunchDescription
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    rh8d_mujoco_description_path = get_package_share_directory('rh8d_mujoco_description')
    rh8d_mujoco_control_path = get_package_share_directory('rh8d_mujoco_control')

    # Process the xacro file to generate the robot description
    xacro_file = os.path.join(
        rh8d_mujoco_description_path,
        'urdf',
        'rh8dR_mujoco.urdf.xacro'
    )
    doc = xacro.parse(open(xacro_file))
    xacro.process_doc(doc)
    robot_description = {'robot_description': doc.toxml()}

    # Include the controller configuration file
    controller_config_file = os.path.join(
        rh8d_mujoco_control_path,
        'config',
        # 'rh8dR_mujoco_controllers.yaml'
        'rh8dR_mujoco_JointTrajectory_Controller.yaml'
    )

    # Include the MuJoCo model file
    mujoco_model_path = os.path.join(
        rh8d_mujoco_description_path,
        'mjcf',
        'scene.xml'
    )

    # Include the RViz configuration file
    rviz_config_file = os.path.join(
        rh8d_mujoco_description_path,
        'rviz',
        'rh8d_mujoco.rviz'
    )

    # Launch the mujoco_ros2_control node with the robot description and controller configuration
    node_mujoco_ros2_control = Node(
        package='mujoco_ros2_control',
        executable='mujoco_ros2_control',
        output='screen',
        parameters=[
            robot_description,
            controller_config_file,
            {'mujoco_model_path': mujoco_model_path}
        ]
    )

    # Launch the robot_state_publisher node to publish the robot's state to TF
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description]
    )

    # Launch the RViz node with the specified configuration file
    node_rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file]
    )

    # Load the controllers after the mujoco_ros2_control node has started
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_state_broadcaster',
            '--controller-manager',
            '/controller_manager'
        ],
        output='screen'
    )

    hand_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'hand_trajectory_controller',
            '--controller-manager',
            '/controller_manager'
        ],
        output='screen'
    )

    # hand_controller_spawner = Node(
    #     package='controller_manager',
    #     executable='spawner',
    #     arguments=[
    #         'hand_forward_position_controller',
    #         '--controller-manager',
    #         '/controller_manager'
    #     ],
    #     output='screen'
    # )

    return LaunchDescription([
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=joint_state_broadcaster_spawner,
                on_exit=[hand_controller_spawner],
            )
        ),
        node_mujoco_ros2_control,
        node_robot_state_publisher,
        node_rviz,
        joint_state_broadcaster_spawner,
    ])
