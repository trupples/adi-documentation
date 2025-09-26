AD-R1M
======

Open Mobile Robot Platform Reference Design
"""""""""""""""""""""""""""""""""""""""""""

Introduction
------------

The Open Mobile Robot Platform Reference Design is a modular, extensible, and fully open-source framework developed to accelerate the design, prototyping, and deployment of autonomous mobile robots.

This reference design integrates key hardware and software components—including motor control, sensor fusion, localization, navigation, and communication—into a cohesive platform that supports rapid development and experimentation. 

Whether you're building a research prototype, an educational robot, or a commercial-grade autonomous system, this platform provides the flexibility and scalability needed to meet diverse application requirements.

It is designed to be compatible with popular development tools, middleware (such as ROS2), and embedded systems, making it ideal for both academic and industrial use.


Specifications
--------------

The scope of the project is to create a mobile robotic platform which should be used as a robotic platform for internal ADI development/advancement on mobile robotic infrastructure and broad market product. It will contains to following:

Software & Middleware
~~~~~~~~~~~~~~~~~~~~~~~~~~
- **ROS 2 Enabled** for:

  - Navigation
  - Motion Control
  - Sensor Integration
  - Battery management

- **Modular ROS 2 Nodes** for:

  - Localization
  - Navigation
  - Motion
  - BMS
- **Compatible with ROS 2 Humble Distribution**
- **Zephyr Support** for embedded firmware on motor, navigation and BMS boards
- **CAN** communication between platforms

Hardware Components
~~~~~~~~~~~~~~~~~~~

Processing Units
^^^^^^^^^^^^^^^^
- Raspberry Pi 5 (with ADI Kuiper2)
- NVIDIA Jetson AGX Orin

Motor & Motion Control
^^^^^^^^^^^^^^^^^^^^^^
- Motor Drivers with Encoder Feedback
- Motor Control Board with Zephyr Firmware

Power Management
^^^^^^^^^^^^^^^^
- Battery Management System board variants (BMS):

  - **3S BMS** with Zephyr Firmware or
  - **12S BMS** with Zephyr Firmware
- Power Management for Mobile Operation

Sensor Suite
^^^^^^^^^^^^
- ADI IMU for Localization
- ADI Time-of-Flight (ToF) Camera for Perception
- Intel RealSense Camera? 

Connectivity
^^^^^^^^^^^^
- USB-to-CAN Adapter
- Ethernet and Wi-Fi Interfaces


User Guides 
-----------

.. toctree::
    :maxdepth: 2

    quick-start-guide
    use-cases
    hardware-guide
    system-setup
    software-guide
    advanced-usage
    troubleshooting


Help and Support
----------------

To receive support regarding your issue, please initiate a discussion on the `Engineer Zone <https://ez.analog.com/>`__ community forum.
