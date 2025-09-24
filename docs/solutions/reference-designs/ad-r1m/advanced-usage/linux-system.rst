AD-R1M Linux system structure (kernel, configs, services, docker)
=================================================================

The AD-R1M Raspberry Pi runs a custom linux image based on Kuiper 2, with a custom Kernel, a handful
of system configuration files, systemd services.

Custom kernel
-------------

To enable all supported hardware configurations and software functionalities, the AD-R1M runs a
kernel with a handful of extra modules enabled, among which:

* ADIS16xxx IMU drivers (default ADIS16475)
* SLCAN support
* Proper options for running Docker containers

You can download a `pre-built kernel tarball`_ or build it yourself. For the latter, start by
cloning the linux branch with AD-R1M defconfigs::

    git clone https://github.com/analogdevicesinc/linux.git --branch ad-r1m/rpi-6.13.y --depth 1

.. _pre-built kernel tarball: https://github.com/analogdevicesinc/linux/releases/DOESNTEXISTYET

Build kernel, modules, dtbs::

    # Skip these two if running this build on an arm64 machine:
    export ARCH=arm64
    export CROSS_COMPILE=aarch64-linux-gnu-

    export KERNEL=kernel_2712
    make -j12 bcm2712_ad_r1m_defconfig Image.gz modules dtbs

Export modules and boot files::

    sudo mkdir -p rootfs/boot/firmware/overlays
    sudo env PATH="$PATH" make INSTALL_MOD_PATH=$PWD/rootfs modules_install
    sudo cp arch/arm64/boot/Image.gz rootfs/boot/$KERNEL.img
    sudo cp arch/arm64/boot/dts/broadcom/*.dtb rootfs/boot/firmware/
    sudo cp arch/arm64/boot/dts/overlays/*.dtb* rootfs/boot/firmware/overlays/
    sudo cp arch/arm64/boot/dts/overlays/README rootfs/boot/firmware/overlays/

Create an archive for easy copying to the target image::

    cd rootfs
    sudo tar -cpf ../ad-r1m-kernel.tar.gz boot lib

The created ``ad-r1m-kernel.tar.gz`` archive can be used in the next steps.

Adding the kernel to an already working linux system
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. caution::
    
    This step may break your setup if you use a bad kernel build. If you feel unsure, it's never a
    bad idea to back up your current ``/boot/kernel_2712.img`` and ``/lib/modules/...`` to be able
    to bring them back in case of a failed update.

.. danger::

    Only extract tarballs you trust! The following command will overwrite system files.

If you have an already working system, adding the newly built kernel is as simple as copying the
tar archive and extracting it in the filesystem root (make sure ``/boot/firmware`` is mounted)::

    sudo tar -xpf path/to/ad-r1m-kernel.tar.gz -C / --keep-directory-symlink

Custom SD card image
--------------------

The SD card image is built as a variant of ADI Kuiper linux. You may download a 
`pre-built SD card image`_ or build one yourself.

.. _pre-built SD card image: https://github.com/analogdevicesinc/adi-kuiper-gen/releases/DOESNTEXISTYET

Prerequisites for building your own image:

* A working `Docker install <https://docs.docker.com/engine/install/>`_
* ~10GB of storage (the faster, the better: write speed will be the main bottleneck)

Start by cloning `adi-kuiper-gen <https://github.com/analogdevicesinc/adi-kuiper-gen>`_, branch ``ad-r1m``::

    git clone https://github.com/analogdevicesinc/adi-kuiper-gen --branch ad-r1m

The ``stages/07.extra-tweaks/10.ad-r1m`` folder contains most relevant configs. You may swap
out the kernel by changing ``.../files/ad-r1m-kernel.tar.gz`` with the one you prepared. To run the
build::

    sudo ./build_docker.sh

... and wait. The final SD card image will be stored in ``kuiper-volume/``.

Bootup process and service management
-------------------------------------

The robot software starts up automatically through the ``ros_app.service``, which runs
``/home/analog/bringup.sh``, which does device initialization and then launches docker containers
for the various ROS 2 nodes using Docker Compose.

.. mermaid::

    flowchart LR
        def(Default systemd target) --> multi_user.target -- Wants --> ros_app.service -- ExecStart--> br[~/bringup.sh]
        def --> graphical.target --> multi_user.target
        br --> c1(Configure SLCAN)
        br --> c2(Configure IMU)
        br --> c3(Enable RC receiver)
        br --> c4(Start Docker containers)

Example commands
~~~~~~~~~~~~~~~~

Show app status and latest logs::

    systemctl status ros_app

Show current run's logs::

    journalctl -b -u ros_app

Show live logs::

    journalctl -f -u ros_app

Stop/start/restart ROS 2 stack::

    systemctl stop ros_app
    systemctl start ros_app
    systemctl restart ros_app

You may also run ``sudo ~/bringup.sh`` directly, but make sure the service isn't already running, 
or else you might trigger odd behaviour.

Disable/enable auto startup after boot:: 

    systemctl disable ros_app
    systemctl enable ros_app


Docker runtime
--------------

ROS 2 nodes are run inside Docker containers brought up using a Docker Compose file. This allows
for easier lifecycle management, as well as reconfiguring and adding new nodes without needing to
rebuild or modify the docker container (though that's absolutely possible).

All persisted data should be stored in ``/home/analog/ros_data``, which is bound to ``/ros_data``
inside each container. Use this for: launch configurations you want to tweak, storing maps, etc.

The base system consists of the following containers:

* ``canopen`` - CANopen communication with motor drives and BMS
* ``imu`` - Data acquisition from IMU
* ``tof`` - Data acquisition from ToF camera, processing into 1D LaserScan
* ``ekf`` - Sensor fusion (EKF between IMU and wheel encoder data)
* ``teleop`` - CRSF remote control, telemetry, and killswitch

Additionally, the following profiles can be used to start optional nodes depending on the current
operation:

* ``mapping`` profile - Create a map of the environment: starts slam_toolbox in online mapping mode
* ``navigation`` profile - Navigate around the environment: starts AMCL localization and nav2 stack
* ``rviz`` profile - Start rviz GUI

Example commands
~~~~~~~~~~~~~~~~

Launch the entire base system::

    docker compose up

Restart a specific container (e.g. ``imu``)::

    docker compose up imu

Launch / stop a specific profile (e.g. ``mapping``)::

    docker compose up --profile mapping
    docker compose down --profile mapping

Adding new functionalities
--------------------------

You may edit the compose file and/or the bringup script to add your own containers and
functionalities.

.. todo:: Lifter node example
