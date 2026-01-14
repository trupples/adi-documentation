AD-R1M System Setup
===================

Base AD-R1M
-----------

.. todo::

    * Flash SD card image
    * Connect to WiFi and update AD-R1M software

**AD-R1M + Nvidia AGX Orin**
-----------------------------

This section describes the main steps required to flash an NVMe with JetPack 6.2.1 on the NVIDIA AGX Orin and to set up the environment for using Isaac ROS packages (cuVSLAM in this case). It also covers configuring RealSense cameras using an NVIDIA-provided Docker image and integrating cuVSLAM with the AGX Orin and AD-R1M navigation functionalities (Sections 1-7).

.. toctree:: 
    :maxdepth: 2

    agx_orin_and_cuvslam_setup/agx-orin-setup
    agx_orin_and_cuvslam_setup/setup-req-pkg-agx-orin
    agx_orin_and_cuvslam_setup/setup-isaac-ros-agx-orin
    agx_orin_and_cuvslam_setup/setup-isaac-ros-vslam
    agx_orin_and_cuvslam_setup/vslam-setup-and-usage
    agx_orin_and_cuvslam_setup/ad-r1m-jetson-orin-hardware-setup
    agx_orin_and_cuvslam_setup/ad-r1m-cuvslam-setup

Below are the necessary steps to configure the AD-R1M to use visual odometry from cuVSLAM in its state estimation and to build a NAV2-compliant map for autonomous navigation using the landmarks point cloud provided by cuVSLAM(Sections 8–9).

.. toctree::
    :maxdepth: 2

    agx_orin_and_cuvslam_setup/ad-r1m-and-nvidia-cuvslam
    agx_orin_and_cuvslam_setup/map-building-for-nav2

If you want to test this setup in a more controlled environment, we recommend using our Gazebo simulation, which includes the base AD-R1M robot (equipped with an IMU, ToF camera, and wheel encoders) plus an additional RealSense camera (Section 10).

.. toctree::
    :maxdepth: 2

    agx_orin_and_cuvslam_setup/ad-r1m-realsense-gazebo


Prototype AD-R1M (Legacy)
-------------------------

.. todo::

    Setup instructions for nonstandard development samples? Shouldn't this just be internal-only?
