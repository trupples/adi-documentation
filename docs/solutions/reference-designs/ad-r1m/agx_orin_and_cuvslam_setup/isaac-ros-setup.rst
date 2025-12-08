2) Setup required packages on AGX Orin
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

This section deals with prerequisites for accessing and building applications on target.

.. code-block:: bash
    
    sudo apt update
    sudo apt install apt-utils
    sudo apt install ssh putty
    sudo apt install git-gui cmake-gui kdiff3
    sudo apt install curl gnupg software-properties-common wget
    sudo apt install v4l-utils qv4l2

3) Setup Isaac ROS on AGX Orin
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

3.1) Set locale 
^^^^^^^^^^^^^^^^

Check the locale settings:

.. code-block:: bash

    locale

Set the UTF-8 locale:

.. code-block:: bash

    sudo apt update
    sudo apt install locales
    sudo locale-gen en_US en_US.UTF-8
    sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
    export LANG=en_US.UTF-8


3.2) Add the **universe** repository
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. code-block:: bash
    
    sudo add-apt-repository universe

3.3) Setup ROS2 source
^^^^^^^^^^^^^^^^^^^^^^^

.. code-block:: bash
        
    wget -qO - https://isaac.download.nvidia.com/isaac-ros/repos.key | sudo apt-key add -
    grep -qxF "deb https://isaac.download.nvidia.com/isaac-ros/release-3 $(lsb_release -cs) release-3.0" /etc/apt/sources.list || \
    echo "deb https://isaac.download.nvidia.com/isaac-ros/release-3 $(lsb_release -cs) release-3.0" | sudo tee -a /etc/apt/sources.list
    sudo apt update
    sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

3.4) Install ROS2 packages
^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. code-block:: bash
    
    sudo apt update
    sudo apt install ros-humble-desktop-full
    sudo apt install ros-dev-tools
 
    sudo apt install ros-humble-urdf-launch
    sudo apt install ros-humble-ackermann-msgs
    sudo apt install ros-humble-moveit
    sudo apt install ros-humble-moveit-resources
    sudo apt install ros-humble-controller-manager
    sudo apt install ros-humble-ros2-controllers
 
    sudo apt install rospack-tools
 
    sudo apt install python3-colcon-clean

3.5) Check apt sources list
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

If the steps given in these sections are run multiple times, it may happen to have at the end of  /etc/apt/sources.list  the next two lines

.. code-block:: bash

    deb https://isaac.download.nvidia.com/isaac-ros/release-3 jammy universe release-3.0
    deb https://isaac.download.nvidia.com/isaac-ros/release-3 jammy release-3.0

.. warning:: 
    If it is the case, comment only the line containing the universe component, save the file, then run apt update again.

3.6) Configure **nvidia-container-toolkit**
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Verify that nvidia-container-toolkit has been installed:

.. code-block:: bash
    
    nvidia-container-toolkit -version

It should display:

.. code-block:: bash
 
    NVIDIA Container Runtime Hook version 1.16.2
    commit: a5a5833c14a15fd9c86bcece85d5ec6621b65652

Configure Docker to use Nvidia container runtime:

.. code-block:: bash
    
    sudo nvidia-ctk runtime configure --runtime=docker

This will modify the  /etc/docker/daemon.json  file.
Restart the Docker daemon:

.. code-block:: bash
    
    sudo systemctl daemon-reload
    sudo systemctl restart docker

Check that docker can run a minimal application (make sure Docker is installed):

.. code-block:: bash

    sudo docker run hello-world

3.7) Install Large File Storage for **git**
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. code-block:: bash

    sudo apt-get install git-lfs
    git lfs install --skip-repo

3.8) Create a Isaac ROS workspace
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. code-block:: bash
   
    mkdir -p  ~/workspaces/isaac_ros-dev/src
    echo "export ISAAC_ROS_WS=${HOME}/workspaces/isaac_ros-dev/" >> ~/.bashrc
    source ~/.bashrc

3.9) Setup the Isaac ROS workspace
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. code-block:: bash

    cd $ISAAC_ROS_WS
    git clone https://github.com/isaac-sim/IsaacSim-ros_workspaces.git

We need only Humble from this list, so remove any other before setting up ROS dependencies and building colcon.

.. code-block:: bash
    
    cd IsaacSim-ros_workspaces
    rm -r jazzy_ws
    cd ..

Install required packages:

.. code-block:: bash

    sudo apt install python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential
    sudo apt install python3-colcon-common-extensions

Ensure that ROS2 has been sourced:

.. code-block:: bash

    source /opt/ros/humble/setup.bash
    cd $ISAAC_ROS_WS

Initialize the ROS dependencies:

.. code-block:: bash

    sudo rosdep init

.. tip::
    This will update  /etc/ros/rosdep/sources.list.d/20-default.list

.. warning::
    The file needs to be deleted prior to every reinitialization of the ROS dependencies.

Update the ROS dependencies:

.. code-block:: bash

    rosdep update

Resolve any package dependencies from the root of the workspace:

.. code-block:: bash

    rosdep install -i --from-path src --rosdistro humble -y

Build the workspace:

.. code-block:: bash

    colcon build

The output should be:

.. code-block:: bash
    
    Starting >>> carter_navigation
    Starting >>> cmdvel_to_ackermann
    Starting >>> custom_message
    Starting >>> h1_fullbody_controller
    Starting >>> isaac_moveit
    Starting >>> isaac_ros2_messages
    Starting >>> isaac_ros_navigation_goal
    Starting >>> isaac_tutorials
    Finished <<< h1_fullbody_controller [0.72s]
    Finished <<< carter_navigation [0.76s]
    Starting >>> isaacsim
    Starting >>> iw_hub_navigation
    Finished <<< cmdvel_to_ackermann [0.79s]
    Finished <<< isaac_tutorials [0.82s]
    Finished <<< isaac_moveit [0.88s]
    Finished <<< custom_message [1.02s]
    Finished <<< isaac_ros2_messages [1.08s]
    Finished <<< isaacsim [1.27s]
    Finished <<< iw_hub_navigation [1.30s]
    Finished <<< isaac_ros_navigation_goal [2.05s]
 
    Summary: 10 packages finished [2.66s]

.. tip::
    To start using the ROS2 packages built within this workspace, open an new terminal and source the workspace with the following commands:

    .. code-block:: bash

        source /opt/ros/humble/setup.bash
        cd $ISAAC_ROS_WS
        source install/local_setup.bash

3.10) Build the Isaac ROS workspace
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
.. code-block:: bash
    
    source /opt/ros/humble/setup.bash
    cd $ISAAC_ROS_WS
    cd IsaacSim-ros_workspaces/humble_ws
    colcon build

3.11) Cleanup
^^^^^^^^^^^^^^^^^^^^^^^^

.. warning::
    It is recommended to use **colcon clean** command to clean workspace/packages. Manually deleting generated ROS folders - such as builds, installs, and logs - may break dependencies!

    .. code-block:: bash
    
        colcon clean workspace
        colcon clean packages

4) Setup Isaac ROS Visual SLAM
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

4.1) Clone **isaac_ros_common**
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. warning:: 
    This step is required only if camera is of RealSense type.
    It should not be run for other types of cameras.

.. code-block:: bash

    cd $ISAAC_ROS_WS/src
    git clone -b release-3.2 https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_common.git isaac_ros_common

Configure the container to include RealSense packages.

.. code-block:: bash
    
    cd $ISAAC_ROS_WS/src/isaac_ros_common/scripts
    touch .isaac_ros_common-config
    echo CONFIG_IMAGE_KEY=ros2_humble.realsense > .isaac_ros_common-config

4.2) Download quickstart data from NGC (Nvidia GPU Cloud)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. code-block:: bash

    sudo apt install curl jq tar

Download the asset from NGC

.. code-block:: bash

    NGC_ORG="nvidia"
    NGC_TEAM="isaac"
    PACKAGE_NAME="isaac_ros_visual_slam"
    NGC_RESOURCE="isaac_ros_visual_slam_assets"
    NGC_FILENAME="quickstart.tar.gz"
    MAJOR_VERSION=3
    MINOR_VERSION=2
    VERSION_REQ_URL="https://catalog.ngc.nvidia.com/api/resources/versions?orgName=$NGC_ORG&teamName=$NGC_TEAM&name=$NGC_RESOURCE&isPublic=true&pageNumber=0&pageSize=100&sortOrder=CREATED_DATE_DESC"
    AVAILABLE_VERSIONS=$(curl -s \
        -H "Accept: application/json" "$VERSION_REQ_URL")
    LATEST_VERSION_ID=$(echo $AVAILABLE_VERSIONS | jq -r "
        .recipeVersions[]
        | .versionId as \$v
        | \$v | select(test(\"^\\\\d+\\\\.\\\\d+\\\\.\\\\d+$\"))
        | split(\".\") | {major: .[0]|tonumber, minor: .[1]|tonumber, patch: .[2]|tonumber}
        | select(.major == $MAJOR_VERSION and .minor <= $MINOR_VERSION)
        | \$v
        " | sort -V | tail -n 1
    )
    if [ -z "$LATEST_VERSION_ID" ]; then
        echo "No corresponding version found for Isaac ROS $MAJOR_VERSION.$MINOR_VERSION"
        echo "Found versions:"
        echo $AVAILABLE_VERSIONS | jq -r '.recipeVersions[].versionId'
    else
        mkdir -p ${ISAAC_ROS_WS}/isaac_ros_assets && \
        FILE_REQ_URL="https://api.ngc.nvidia.com/v2/resources/$NGC_ORG/$NGC_TEAM/$NGC_RESOURCE/\
    versions/$LATEST_VERSION_ID/files/$NGC_FILENAME" && \
        curl -LO --request GET "${FILE_REQ_URL}" && \
        tar -xf ${NGC_FILENAME} -C ${ISAAC_ROS_WS}/isaac_ros_assets && \
        rm ${NGC_FILENAME}
    fi

4.3) Build **isaac_ros_visual_slam**
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

**Check the CDI (Container Device Interface)**
List the content of CDI:

.. code-block:: bash

    sudo nvidia-ctk cdi list

.. important::
    If the output looks like:

    .. code-block:: bash
        
        INFO[0000] Found 2 CDI devices
        nvidia.com/gpu=0
        nvidia.com/gpu=all
    
    **OR**

    .. code-block:: bash
  
        INFO[0000] Found 4 CDI devices
        nvidia.com/gpu=0
        nvidia.com/gpu=all
        nvidia.com/pva=0
        nvidia.com/pva=all  
    
    you can skip the instructions from the warning displayed bellow and jump at the adding user to docker group step.

.. warning::
    Proceed with the rest of the steps from this warning only if the output looks like the following, particularly if there is no gpu listed besides pva (Programmable Vision Accelerator).

    .. code-block:: bash
 
        INFO[0000] Found 2 CDI devices
        nvidia.com/pva=0
        nvidia.com/pva=all
    
    Generate the CDI spec:

    .. code-block:: bash

        sudo nvidia-ctk cdi generate --mode=csv --output=/etc/cdi/nvidia.yaml

    Install the pva-allow-2 package:

    .. code-block:: bash

        sudo apt update
        sudo apt install software-properties-common
        sudo apt-key adv --fetch-key https://repo.download.nvidia.com/jetson/jetson-ota-public.asc
        sudo add-apt-repository 'deb https://repo.download.nvidia.com/jetson/common r36.4 main'
        sudo apt update
        sudo apt install pva-allow-2

**Add current user to docker group**

.. code-block:: bash

    sudo usermod -aG docker $USER && newgrp docker

**Launch and create the docker container using the run_dev.sh script**

.. code-block:: bash

    cd $ISAAC_ROS_WS/src/isaac_ros_common
    ./scripts/run_dev.sh -d $ISAAC_ROS_WS

.. note::
    Depending on the network speed, this step may take up to one or two hours.
    The download is performed only once, so all next docker launches will take a few seconds, only sanity checks will take place.

**Install the VSLAM package**

.. code-block:: bash
    
    sudo apt update
    sudo apt install ros-humble-isaac-ros-visual-slam

**Build all packages**

.. code-block:: bash

    cd $ISAAC_ROS_WS
    source ./install/setup.bash
 
    cp ./src/isaac_ros_common/isaac_ros_rosbag_utils/isaac_ros_rosbag_utils/__init__.py ./src/isaac_ros_common/isaac_ros_rosbag_utils/isaac_ros_rosbag_utils/scripts/
 
    colcon build

5) VSLAM setup and usage
~~~~~~~~~~~~~~~~~~~~~~~~~

Connect the camera to any USB-A connector on the AGX Orin (USB-A 3.2 are recommended).


5.1) Intel RealSense cameras
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

**Check how the camera is detected**
List only the USB devices with VID **0x8086** (Intel).

.. code-block:: bash

    lsusb -d 8086:

The output should be similar to:

.. code-block:: bash

    Bus 002 Device 005: ID 8086:0b3a Intel Corp. Intel(R) RealSense(TM) Depth Camera 435i

where **0x0B3A** is the PID of RealSense D435i.

**Install the packages**
Register the server's public key:

.. code-block:: bash

    sudo apt-key adv --keyserver keyserver.ubuntu.com --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE || sudo apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE

Add the server to the list of repositories:

.. code-block:: bash

    sudo add-apt-repository "deb https://librealsense.intel.com/Debian/apt-repo $(lsb_release -cs) main" -u

Install the SDK:

.. code-block:: bash

    sudo apt install librealsense2-utils
    sudo apt install librealsense2-dev

**Test the functionality with RealSense Viewer**

.. code-block:: bash

    realsense-viewer

.. note::
    This is the application running in "native mode", outside the docker.

.. figure:: /solutions/reference-designs/ad-r1m/agx_orin_and_cuvslam_setup/cuvslam_orin_setup/RealSense_viewer_non_docker.png
    :alt: RealSense viewer outside Docker
    :align: center
    :width: 600px

.. tip::
    * The application version is 2.56.5
    * Note the current firmware version, which is 5.13.0.50. The last firmware version is reported as 5.17.0.10.

.. important::
    There is no IMU group in the left panel, under the Stereo Module and RGB Camera.

**Load the Docker**

.. code-block:: bash

    cd $ISAAC_ROS_WS
    source ./install/setup.bash
    cd ./src/isaac_ros_common
    ./scripts/run_dev.sh

Inside the container, relaunch the viewer:

.. code-block:: bash

    realsense-viewer

.. figure:: /solutions/reference-designs/ad-r1m/agx_orin_and_cuvslam_setup/cuvslam_orin_setup/RealSense_viewer_docker.png
    :alt: RealSense viewer inside Docker
    :align: center
    :width: 600px

.. tip::
    * The application version is 2.55.1
    * Note that the last firmware version is reported as 5.16.0.1, which is different than the non-docker one (5.17.0.10).

.. important::
    There is a Motion Module group in the left panel, under the Stereo Module and RGB Camera. 

Select the 2D view mode and enable the Motion Module.  Rotate the camera module and check how gyroscope and accelerometer values change.

.. figure:: /solutions/reference-designs/ad-r1m/agx_orin_and_cuvslam_setup/cuvslam_orin_setup/RealSense_viewer_docker_2D.png
    :alt: RealSense accelerometer and gyroscope in realsense-viewer
    :align: center
    :width: 600px

**Test the VSLAM ROS nodes**
In the existing terminal run:

.. code-block:: bash

    sudo apt update
    sudo apt install -y ros-humble-isaac-ros-visual-slam ros-humble-isaac-ros-examples ros-humble-isaac-ros-realsense

Start a quick VSLAM demo:

.. code-block:: bash

    ros2 launch isaac_ros_examples isaac_ros_examples.launch.py launch_fragments:=realsense_stereo_rect,visual_slam \
    interface_specs_file:=${ISAAC_ROS_WS}/isaac_ros_assets/isaac_ros_visual_slam/quickstart_interface_specs.json \
    base_frame:=camera_link camera_optical_frames:="['camera_infra1_optical_frame', 'camera_infra2_optical_frame']"

.. warning::
    Note that closing this terminal will unload the docker from all terminals which are created in the following, and where docker has been loaded.

**Open another terminal** and run:

.. code-block:: bash

    cd $ISAAC_ROS_WS
    source ./install/setup.bash
    cd ./src/isaac_ros_common
    ./scripts/run_dev.sh
 
    rviz2 -d $(ros2 pkg prefix isaac_ros_visual_slam --share)/rviz/default.cfg.rviz

The RViz scene, displaying the point cloud, should look similar to:

.. figure:: /solutions/reference-designs/ad-r1m/agx_orin_and_cuvslam_setup/cuvslam_orin_setup/RViz2_pointcloud_docker.png
    :alt: cuVSLAM demo run
    :align: center
    :width: 600px

You can also check the available topics:

.. code-block:: bash
    
    ros2 topic list

The output should be similar to:

.. code-block:: bash

    /diagnostics
    /extrinsics/depth_to_infra1
    /extrinsics/depth_to_infra2
    /imu
    /infra1/image_rect_raw/compressed
    /infra1/image_rect_raw/compressedDepth
    /infra1/image_rect_raw/theora
    /infra1/image_rect_raw_mono
    /infra1/image_rect_raw_mono/nitros
    /infra1/metadata
    /infra2/image_rect_raw/compressed
    /infra2/image_rect_raw/compressedDepth
    /infra2/image_rect_raw/theora
    /infra2/image_rect_raw_mono
    /infra2/image_rect_raw_mono/nitros
    /infra2/metadata
    /left/camera_info_rect
    /left/image_rect
    /left/image_rect/nitros
    /left/image_rect_mono
    /left/image_rect_mono/nitros
    /parameter_events
    /right/camera_info_rect
    /right/image_rect
    /right/image_rect/nitros
    /right/image_rect_mono
    /right/image_rect_mono/nitros
    /rosout
    /tf
    /tf_static
    /visual_slam/initial_pose
    /visual_slam/status
    /visual_slam/tracking/odometry
    /visual_slam/tracking/slam_path
    /visual_slam/tracking/vo_path
    /visual_slam/tracking/vo_pose
    /visual_slam/tracking/vo_pose_covariance
    /visual_slam/trigger_hint
    /visual_slam/vis/gravity
    /visual_slam/vis/landmarks_cloud
    /visual_slam/vis/localizer
    /visual_slam/vis/localizer_loop_closure_cloud
    /visual_slam/vis/localizer_map_cloud
    /visual_slam/vis/localizer_observations_cloud
    /visual_slam/vis/loop_closure_cloud
    /visual_slam/vis/observations_cloud
    /visual_slam/vis/pose_graph_edges
    /visual_slam/vis/pose_graph_edges2
    /visual_slam/vis/pose_graph_nodes
    /visual_slam/vis/slam_odometry
    /visual_slam/vis/velocity

**Reference links**

This tutorial is based on the official **NVIDIA Isaac ROS** documentation. More information about configuring VSLAM in the Nvidia ecosystem can be found at the following pages:
    * https://nvidia-isaac-ros.github.io/concepts/visual_slam/cuvslam/index.html
    * https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_visual_slam/index.html
    * https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_visual_slam/isaac_ros_visual_slam/index.html
    * https://nvidia-isaac-ros.github.io/getting_started/hardware_setup/sensors/realsense_setup.html  
    * https://nvidia-isaac-ros.github.io/concepts/visual_slam/cuvslam/validating_cuvslam_setup.html
