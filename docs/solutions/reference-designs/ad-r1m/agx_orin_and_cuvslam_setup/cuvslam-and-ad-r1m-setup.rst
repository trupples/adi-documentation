6) AD-R1M and cuVSLAM setup
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Up to this point, the NVIDIA Docker container has been configured with all required dependencies for running cuVSLAM with a RealSense camera. To enable full integration of cuVSLAM on the AD-R1M platform, a few additional configuration steps are necessary.

The following steps should be run on the AGX Orin platform. Open a new terminal and run the following commands:

.. code-block:: bash
    
    cd $ISAAC_ROS_WS
    git clone **ADD adrd_demo_ros2 REPO LINK HERE**

Go to the scripts folder in the **isaac-ros-common** package:

.. code-block:: bash

    cd /src/isaac_ros_common/scripts

.. todo::
    Write a chain of bash command that should be pasted in the terminal to:

    * Copy the Dockerfile.visualslam into **$ISAAC_ROS_WS/src/isaac_ros_common/docker**
    * Create a folder inside  ~/workspaces/isaac_ros-dev/src/isaac_ros_common/docker/ named entrypoint_addition
    * Copy the entrypoint script (put this in the repo alongside the dockerfile) in the prev folder

This step copies the additional Dockerfiles and entrypoint scripts required to install extra dependencies inside 
the container and to enable proper communication between the NVIDIA Docker environment and the AD-R1M platform.

These changes ensure that the container can successfully run zenoh using mDNS, which is necessary for discovering 
and communicating with the robot (ad-r1m-0) over Ethernet.

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

.. todo::
    * what scripts should be launched and their specific Meaning
    * how to configure the ekf.yaml file to include visualslam odometry
    * Mention the urdf file that should be modified depending on where is the realsense placedon the robot
    * short demo on the scripts that should be run on the robot and on the Orin to perform navigation + some rviz video
    * Describe how to use the pointcloud_to_grid package to build and save the 2D occupancy grid build from the pointcloud
    * For ad_r1m_cuvslam and pointcloud_to_grid (which should be cleaned and maybe renamed) include more specific documentation on the GitHub Repo












