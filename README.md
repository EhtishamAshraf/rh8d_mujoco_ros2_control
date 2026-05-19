# RH8D Hand Simulation (MuJoCo + ROS 2 Control)

This project simulates the **RH8D tendon-driven robotic hand** using **MuJoCo** and **ROS 2 Control**.

In this project, the hand is controlled through the standard **ros2_control** framework using a modified `mujoco_ros2_control` plugin. Check out the [Pure ROS2 version](https://github.com/EhtishamAshraf/rh8d_mujoco_ros2.git).

The system uses:

1. A MuJoCo MJCF model for physics simulation.
2. A URDF/Xacro robot description for ROS 2 visualization and control interfaces.
3. A modified `mujoco_ros2_control` plugin.
4. ROS 2 Control controllers such as `joint_state_broadcaster` and trajectory/position controllers.
5. MuJoCo native spatial tendons for RH8D tendon-driven finger motion.

The main advantage of using MuJoCo is that it supports **native spatial tendon modeling**, which is more suitable for simulating the tendon-driven RH8D hand than the Gazebo mimic-joint approximation.

Tested on:

- ROS 2 Humble
- MuJoCo
- ros2_control
- Modified `mujoco_ros2_control` plugin

Demo video:

[![Demo Video](https://github.com/EhtishamAshraf/rh8d_mujoco_ros2_control/blob/6e2285dd5158654c70342f12fa1c80b0b7976ec1/assets/6.png)](https://www.youtube.com/watch?v=uwueH81qcI0)

---

## Repository Branches

This repository is organized into separate branches for the two RH8D hand configurations. One branch contains the **left-hand simulation**, while the other branch contains the **right-hand simulation**. 

Both branches follow the same MuJoCo + ROS 2 Control architecture, but the robot description, MJCF model, joint names, actuator mappings, and mesh orientations are adjusted according to the corresponding hand side.


## Project-Specific Modifications to the `mujoco_ros2_control` Plugin

The original [`mujoco_ros2_control plugin`](https://github.com/moveit/mujoco_ros2_control) was modified for the RH8D tendon-driven hand simulation. The main modifications are:

- Added support for tendon-driven finger control, where a ROS 2 Control command can be sent to a MuJoCo tendon actuator instead of directly commanding every finger joint.

- Added separate URDF joint state export so real movable finger joints can be exposed as ROS 2 Control state interfaces, including position, velocity, and effort, and published through `joint_state_broadcaster` to `/joint_states`.

- Added support for the palm rangefinder sensor:
  - `palm_range/value`

- Added support for RH8D fingertip force sensors:
  - `thumb_tip_force`
  - `index_tip_force`
  - `middle_tip_force`
  - `ring_tip_force`
  - `small_tip_force`

- Example: 3D sensor state interfaces:
  - `index_tip_force/x`
  - `index_tip_force/y`
  - `index_tip_force/z`

---


## Build and Launch Instructions

Go to the workspace and Build the packages:

```bash
colcon build
```

Source the workspace:

```bash
source install/setup.bash
```

Launch the MuJoCo ROS 2 Control simulation:

```bash
ros2 launch rh8d_mujoco_control rh8d_mujoco.launch.py
```

Check active controllers:

```bash
ros2 control list_controllers
```
Run the ROS2 node for controlling the hand:

```bash
ros2 run rh8d_mujoco_control rh8dL_object_pick_JointTrajectory
```

**Picking Ball video:**
If you want to pick the ball instead of the cylinder, please adjust the XML file here: rh8d_mujoco_description/mjcf/rh8dL.xml (you just need to uncomment the relevant section)

[![Demo Video](https://github.com/EhtishamAshraf/rh8d_mujoco_ros2_control/blob/6e2285dd5158654c70342f12fa1c80b0b7976ec1/assets/8.png)](https://www.youtube.com/watch?v=wV0Q-Gtomvw)

---

## References

1. To convert urdf file to xml file, use the following command: [Reference](https://www.youtube.com/watch?v=v2OfmQaoIH4)
```bash
compile file_name.urdf file_name.xml
```

2. MuJoCo ROS2 Control plugin examples can be found [here](https://github.com/moveit/mujoco_ros2_control_examples)
