EVAL-AD-AMR ROS2 Architecture
=============================

.. contents:: Table of Contents
   :depth: 2
   :local:

Overview
--------
The EVAL-AD-AMR ROS2 architecture is an autonomous robot system based on the Arduino AGV platform, integrating CANOpen motor control, ADI Time-of-Flight (ToF) camera, IMU, and the ROS2 Navigation2 stack for 2D localization and navigation.

System Architecture
-------------------

.. mermaid::

   graph TD
       K[Lift Server] <-->|/lift_cmd, /lift_state| J
       A[Drive System] -->|/odom| F[Robot Localization]
       B[IMU] -->|/imu| F
       C[ToF Camera] -->|/cam1/depth_image| D[Depth to LaserScan]
       D -->|/cam1/scan| I[Nav2 AMCL]
       F -->|/odom_filtered| G[Navigation Stack]
       E[Remote Control] -->|/cmd_vel_joy| H[Command Multiplexer]
       G -->|/cmd_vel_nav| H
       H -->|/cmd_vel| A
       I -->|/amcl_pose| G
       I -->|/amcl_pose| J[Demo Commander]
       J -->|/goal_pose| G
       G -->|/feedback| J
       G -->|/status| J

Components
----------

Drive System and Odometry
~~~~~~~~~~~~~~~~~~~~~~~~~
The robot uses multiple data sources for accurate positioning:
 - Motor encoders → `/diff_drive_controller/odom`
 - IMU measurements → `/imu`
 - Fused data → `/odom` (via robot_localization)

.. code-block:: bash

    # Start the drive system (specify CAN interface)
    ros2 launch adrd_demo_ros2 just_motors.launch.py can_iface:=can0

This motor launch starts:

**controller_manager_node**
 - Core ROS2 control component that manages and coordinates robot controllers
 - Loads and manages different controller plugins
 - Config: reads from ``ros2_controllers.yaml`` containing:

    - Controller configurations
    - Update rates
    - Controller interfaces

**robot_state_publisher**
 - Publishes the robot's state (joint positions) to tf2
 - Computes forward kinematics and broadcasts the robot's state
 - Config:

    - URDF model of the robot from *urdf* directory
    - Joint states from */joint_states* topic

**controller_spawner**
 - Utility to dynamically load and start controllers
 - Manages controller lifecycle (configured → active)

**joint_state_broadcaster_spawner**
 - Specialized spawner for the joint state broadcaster
 - Publishes joint states from hardware to */joint_states* topic

**robot_localization_node**
 - Provides state estimation for robot pose
 - Fuses data from various sensors (IMU, odometry, etc.)
 - Config: Uses ``ekf.yaml`` containing:
 
    - Sensor inputs and frame IDs
    - Covariance matrices
    - Update frequencies
    - State estimation parameters

IMU (Inertial Measurement Unit)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

The ADI IMU node publishes sensor data to the `/imu` topic using the following configuration:

#### Parameters
- **iio_context_string**: *'ip:localhost'*
    - Defines the connection method to the IMU device via Industrial I/O (IIO) framework
- **measured_data_topic_selection**: *2*
    - Selects standard IMU message type for the `/imu` topic
    - Follows `sensor_msgs/Imu <http://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/Imu.html>`__ format

.. code-block:: bash

    # Launch the IMU node using the above parameters
    ros2 launch adrd_demo_ros2 just_imu.launch.py


For additional configuration details, refer to the `imu_ros2 documentation <https://github.com/analogdevicesinc/imu_ros2>`__.

The IMU is mounted to the robot using a fixed joint as defined in the URDF:

.. code-block:: xml

    <joint name="imu_joint" type="fixed">
        <parent link="base_link"/>
        <child link="imu_link"/>
        <origin xyz="0.133 -0.01 ${wheel_radius}" rpy="0 ${pi} ${-pi/2}"/>
    </joint>

This means the IMU is positioned 0.133 m forward, -0.01 m to the left, and at the height of the wheel radius from the base_link, with a rotation of (0, π, -π/2) radians.


Time-of-Flight Camera
~~~~~~~~~~~~~~~~~~~~~
- Publishes depth images (`/cam1/depth_image`)
- Camera calibration (`/cam1/camera_info`)
- Depth to LaserScan node converts depth to 2D scan (`/cam1/scan`)
- Parameters: ``depthimage_to_laserscan/cfg/param.yaml``
- Camera position: ``urdf/camera.xacro``
- Launch: ``just_tof.launch.py``

Remote Control Interface
~~~~~~~~~~~~~~~~~~~~~~~~
- Command multiplexer prioritizes RC joystick, keyboard, and navigation stack
- Launch: ``just_crsf.launch.py``

Mapping and Localization
~~~~~~~~~~~~~~~~~~~~~~~~
- SLAM Toolbox for mapping/localization (``online_async_launch.py``)
- AMCL for Monte Carlo localization (``localization_launch.py``)
- Parameters: ``config/mapper_params_online_async.yaml``, ``config/nav2_params_sim.yaml``

Autonomous Navigation
~~~~~~~~~~~~~~~~~~~~~
- Navigation2 stack for path planning and execution (``navigation_launch.py``)
- Subscribes to `/odom`, `/cam1/scan`, `/tf`
- Publishes to `/cmd_vel_nav` (smoothed to `/cmd_vel`)

High-Level Control (Commander Nodes)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
- Nodes for demo, waypoint following, and elevator integration
- Integrate with Nav2's BasicNavigator
- Launch: ``scripts/commander/``

Visualization and Manual Control
--------------------------------

RViz Visualization
~~~~~~~~~~~~~~~~~~
- Launch RViz: ``ros2 run rviz2 rviz -d src/adrd_demo_ros2/rviz/main.rviz``
- Fixed frame selection: `/base_link`, `/odom`, `/map`
- TF tree: ``map → odom → base_link → [camera_link, wheel_*_link, imu_link]``

Manual Control
~~~~~~~~~~~~~~
- Keyboard teleop: ``ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/diff_drive_controller/cmd_vel_unstamped``

Quick Start
-----------

.. code-block:: bash

   #!/bin/bash
   ros2 launch adrd_demo_ros2 just_motors.launch.py can_iface:=can0 &
   sleep 20
   ros2 launch adrd_demo_ros2 just_crsf.launch.py &
   sleep 10
   ros2 launch adrd_demo_ros2 just_imu.launch.py &
   sleep 10
   ros2 launch adrd_demo_ros2 just_tof.launch.py &
   sleep 10
   ros2 launch adrd_demo_ros2 localization_launch.py &

See the main README and component documentation for further details.
