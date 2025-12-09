6) AD-R1M and Jetson AGX Orin: hardware setup
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Connect a DC power jack with a 5.5 mm outer diameter and 2.5 mm inner diameter (male connector) to the ADRD5161 BMS. Ensure that the center pin is wired with positive polarity:

.. figure:: /solutions/reference-designs/ad-r1m/agx_orin_and_cuvslam_setup/cuvslam_orin_setup/Robot_power_connection2.png
        :alt: Barrel connection to robot battery
        :align: center
        :width: 600px

Connect the Ethernet cable to Raspberry Pi:

.. figure:: /solutions/reference-designs/ad-r1m/agx_orin_and_cuvslam_setup/cuvslam_orin_setup/Robot_ethernet_connection2.png
        :alt: Rpi ethernet cable
        :align: center
        :width: 600px

Connect the DC jack power supply and the Ethernet cable to the AGX Orin:

.. figure:: /solutions/reference-designs/ad-r1m/agx_orin_and_cuvslam_setup/cuvslam_orin_setup/Orin_ethernet_and_power.jpg
        :alt: Power and Eth connection to AGX Orin
        :align: center
        :width: 600px

With this setup, the Jetson AGX Orin will have sufficient power to operate in 30 W performance mode.

7) AD-R1M and cuVSLAM setup
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Up to this point, the NVIDIA Docker container has been configured with all required dependencies for running cuVSLAM with a RealSense camera. To enable full integration of cuVSLAM on the AD-R1M platform, a few additional configuration steps are necessary.

The following steps should be run on the AGX Orin platform. Open a new terminal and run the following commands:

.. todo::
    Remember to add the github link bellow

.. code-block:: bash
    
    cd $ISAAC_ROS_WS
    git clone **ADD adrd_demo_ros2 REPO LINK HERE**
    cp -r src/adrd_demo_ros2/ad_r1m_cuvslam/* src/isaac-ros-common/docker

This step copies the additional Dockerfiles and entrypoint scripts required to install extra dependencies inside 
the container and to enable proper communication between the NVIDIA Docker environment and the AD-R1M platform.

These changes ensure that the container can successfully run zenoh using mDNS, which is necessary for discovering 
and communicating with the robot (ad-r1m-0) over Ethernet.

Go to the scripts folder in the **isaac-ros-common** package:

.. code-block:: bash

    cd /src/isaac_ros_common/scripts

Modify the **run_dev.sh** script from **$ISAAC_ROS_WS/src/isaac_ros_common** as follows:
    * Look for the following lines of code 

    .. code-block:: bash

        BASE_NAME="isaac_ros_dev-$PLATFORM"
        if [[ ! -z "$CONFIG_CONTAINER_NAME_SUFFIX" ]] ; then
            BASE_NAME="$BASE_NAME-$CONFIG_CONTAINER_NAME_SUFFIX"
        fi
        CONTAINER_NAME="$BASE_NAME-container"

        # Remove any exited containers.
        if [ "$(docker ps -a --quiet --filter status=exited --filter name=$CONTAINER_NAME)" ]; then
            docker rm $CONTAINER_NAME > /dev/null
        fi
    
    * Insert the following lines after BASE_NAME="isaac_ros_dev-$PLATFORM". This ensures that intermediate images are not left unnamed and that each image is tagged based on the Dockerfiles used in the build process:

    .. code-block:: bash

        if [[ ! -z "$IMAGE_KEY" && "$IMAGE_KEY" != "ros2_humble" ]]; then
        IMAGE_KEY_SAFE="${IMAGE_KEY//./-}"   # replace dots with dashes
        BASE_NAME="${BASE_NAME}-${IMAGE_KEY_SAFE}"
        fi
    
    * In order to automatically configure Zenoh at startup, set the necessary Docker container environment variables:

    .. code-block:: bash

        docker run -it --rm \
            -e ROS_NAMESPACE=ad_r1m_0 \
            -e RMW_IMPLEMENTATION=rmw_zenoh_cpp \
            -e ROBOT_TCP_NAME=ad-r1m-0 \
            -e ZENOH_CONFIG_OVERRIDE="connect/endpoints=[\"tcp/ad-r1m-0.local:7447\"];mode=\"client\"" \
            --privileged \
            --network host \
            --ipc=host \
            "${DOCKER_ARGS[@]}" \
            -v "$ISAAC_ROS_DEV_DIR":/workspaces/isaac_ros-dev \
            -v /etc/localtime:/etc/localtime:ro \
            --name "$CONTAINER_NAME" \
            --runtime nvidia \
            --entrypoint /usr/local/bin/scripts/workspace-entrypoint.sh \
            --workdir /workspaces/isaac_ros-dev \
            "$BASE_NAME" \
            /bin/bash

After completing all these steps, you can build the updated Docker image by running:

.. code-block:: bash

    cd $ISAAC_ROS_WS/src/isaac_ros_common
    ./scripts/run_dev.sh -i ros2_humble.realsense.visualslam

This command creates a new image layer on top of the base Docker image provided by NVIDIA, incorporating all 
additional dependencies and configuration changes.

You can then launch a new container instance from this updated image using the same script: 

.. code-block:: bash

    cd $ISAAC_ROS_WS/src/isaac_ros_common
    ./scripts/run_dev.sh -i ros2_humble.realsense.visualslam


8) Launching AD-R1M and NVIDIA cuVSLAM
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Configure **robot_localization** to use cuVSLAM visual odometry feedback. 

Open a new terminal and connect to the AD-R1M robot via SSH:

.. code-block:: bash

    ssh analog@ad-r1m-0.local

Enter the password *analog* when prompted.

The Extended Kalman Filter (EKF) configuration file tells robot_localization which sensor topics to subscribe to and how to use them. Open the configuration file:

.. code-block:: bash

    vim ros_data/ekf.yaml

Add the following lines to enable visual odometry from cuVSLAM:

.. code-block:: yaml

    odom1: /visual_slam/tracking/odometry
    odom1_config: [true, true, false,
                  false, false, true,
                  false, false, false,
                  false, false, false,
                  false, false, false]

After saving the configuration, start the localization system on the robot:

.. code-block:: bash

    sudo ./bringup_blind.sh

Open a new terminal on the NVIDIA Jetson and launch the cuVSLAM node:

.. code-block:: bash

    cd $ISAAC_ROS_WS/src/isaac-ros-common
    ./scripts/run_dev.sh -i ros2_humble.realsense.visualslam
    source install/setup.sh
    ros2 launch ad_r1m_cuvslam cuvslam_multirealsense.launch.py

The EKF will now combine measurements from the IMU, wheel odometry, and visual odometry to produce the pose estimate.

.. important::
    * Make sure the transform between *ad_r1m_0/base_link* and *camera1_link* matches your configuration. By default, it is set as a translation of +0.335 m along the X-axis (front of the robot) in **single_realsense_calibration.urdf**.

    .. code-block:: xml

        <joint name="camera1" type="fixed">
            <parent link="ad_r1m_0/base_link"/>
            <child link="camera1_link"/>
            <origin xyz="0.335 0.0 0.0" rpy="0 0 0"/>
        </joint>
    
    * Make sure the serial number of your RealSense camera matches the *serial_no* parameter in **vslam_single_realsense.yaml**:

    .. code-block:: yaml

        cameras:
            - camera_name: camera1
              serial_no: '243322074768' # This should match your camera serial number
              usb_port_id: ''
              depth_module.inter_cam_sync_mode: 1

.. figure:: /solutions/reference-designs/ad-r1m/agx_orin_and_cuvslam_setup/cuvslam_orin_setup/ad_r1m_and_cuvslam_demo1.gif
    :alt: AD-R1M and cuVSLAM demo
    :align: center
    :width: 800px

**Multi RealSense cameras**

.. note::
    * You can use multiple RealSense cameras (up to 16 stereo cameras) without modifying the launch script. Simply configure each camera's serial number in **vslam_single_realsense.yaml** and set the corresponding transform in **single_realsense_calibration.urdf**.
    * When using multiple stereo cameras, it is recommended to perform inter-camera synchronization to ensure reliable visual odometry feedback. RealSense cameras support hardware synchronization by designating one camera as the master and the others as slaves:
    
    .. code-block:: yaml
        
        depth_module.inter_cam_sync_mode: 1 # setting the master camera
        depth_module.inter_cam_sync_mode: 2 # setting the slave cameras

    * For more information on RealSense hardware synchronization, see: https://dev.realsenseai.com/docs/multiple-depth-cameras-configuration

**cuVSLAM parameter setup**

.. note::
    * cuVSLAM supports IMU fusion with a single stereo camera. If IMU fusion is enabled, it is crucial to determine the specific noise and bias parameters of your IMU:

    .. code-block:: yaml

        visual_slam:
            enable_imu_fusion: True
            gyro_noise_density: 0.00015593952556059264 
            gyro_random_walk: 7.371377089394156e-06 
            accel_noise_density: 0.00246907522075608 
            accel_random_walk: 0.0004886176561242167 
    
    * To analyze your IMU’s noise parameters, you can use the https://github.com/CruxDevStuff/allan_ros2 tool, which applies Allan deviation analysis.  
    * To use cuVSLAM for mapping and pose estimation, *enable_localization_n_mapping* must be set to True. If not, cuVSLAM will only compute visual odometry.
    * For planar robots, it is recommended to enable the ground constraint in cuVSLAM to reduce or eliminate drift along the Z-axis:

    .. code-block:: yaml

        enable_localization_n_mapping: True
        enable_ground_constraint_in_odometry: True
        enable_ground_constraint_in_slam: True
    
    * *enable_localization_n_mapping* switches cuVSLAM from pure odometry mode to full SLAM mode.
    * *enable_ground_constraint_in_odometry* and *enable_ground_constraint_in_slam* help maintain a stable Z-axis estimate for robots moving on flat surfaces.
    
    * When using an EKF to fuse all available sensor data (IMU, wheel odometry, visual odometry), cuVSLAM should not publish the odom -> base_link transform, as the EKF will provide the robot’s final pose. However, if localization and mapping is enabled, cuVSLAM can still publish the map -> odom transform for visualization and global reference:

    .. code-block:: yaml

        publish_map_to_odom_tf: True
        publish_odom_to_base_tf: False #When EKF publish_tf is true


9) Map building for NAV2
~~~~~~~~~~~~~~~~~~~~~~~~~

To perform autonomous navigation, you can either build a 2D occupancy grid using the ToF camera mounted on the robot, or use the **ad_r1m_pointcloud_to_occupancygrid** package to convert the 3D pointcloud generated by cuVSLAM into a 2D occupancy grid required by NAV2.

To convert a point cloud into an occupancy grid, run the **build_occupancy_grid.launch.py** launch file:

.. code-block:: bash

    ros2 launch ad_r1m_pointcloud_to_occupancygrid build_occupancy_grid.launch.py topic:=/visual_slam/vis/landmarks_cloud

The pointcloud resulted as an output from the cuVSLAM can be noisy or include ground points which, for a reliable navigation
map, should be filtered. To enable pointcloud filtering before map building set the *filter_enable* parameter in **build_occupancy_grid.launch.py**. 
You can also enable/disable each individual filter to achieve an accurate occupancy grid, depending on the available pointcloud:

.. code-block:: python

    {'filter_enable': True}, # Global filtering enable
    {'ror_enable': True}, # Radius outlier removal enable
    {'sor_enable': False}, # Statistical outlier removal
    {'pass_enable': True}, # Pass through filter (z-band)
    {'voxel_enable': True}, # Voxel grid enable
    {'cluster_enable': False}, # Filtering using clusters

This package allows optional averaging of consecutive grid maps to improve map quality when using noisy or incomplete 
point clouds, such as cuVSLAM landmarks. By averaging multiple frames, random sensor noise is reduced, persistent structures 
become clearer, and short-lived or unstable points are suppressed.

.. code-block:: python

     # Normal mean averaging of maps
    {'normal_averaging_enable': True},
    # Moving average parameters
    {'moving_average_enable': False},
    {'ma_alpha': 0.5},

.. warning::
    If both *normal_averaging_enable* and *moving_average_enable* are true, only normal averaging will be applied.

.. important::
    When *filter_enable* is **True**, a Gaussian Z-probability weighting is applied based on how close its height is to the robot’s “obstacle-relevant” vertical band. 
    Set the Gaussian mean to roughly the robot’s mid-height and the standard deviation to about half the robot’s height. Points 
    within this band receive higher weight, while very low or very high points are suppressed, helping reduce vertical noise and emphasize 
    likely obstacles.

    .. code-block:: python

        {'gaussian_mean': 0.2},
        {'gaussian_stddev': 0.05},

.. figure:: /solutions/reference-designs/ad-r1m/agx_orin_and_cuvslam_setup/cuvslam_orin_setup/Map_build_sim_demo.gif
    :alt: AD-R1M, cuVSLAM and 2D occupancy grid build
    :align: center
    :width: 800px

More information about the **ad_r1m_pointcloud_to_occupancygrid** can be found at:

.. todo::
    ADD LINK TO **ad_r1m_pointcloud_to_occupancygrid** PACKAGE DOCUMENTATION














