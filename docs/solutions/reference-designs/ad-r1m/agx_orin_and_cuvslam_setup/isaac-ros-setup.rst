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
