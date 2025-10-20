AD-R1M Quick Start Guide
========================

.. figure:: ./figures/robot-hardware.png
   :alt: AD-R1M robot hardware architecture
   :align: center
   :width: 1000px

Use the side-panel buttons to power and initialize the robot electronics.

.. figure:: ./figures/robot-buttons.png
   :alt: Robot side panel (left to right): silver button = battery cutoff, blue LED = status, green button = BMS start
   :align: center
   :width: 500px

- **Silver button**: Press to enable the battery. It will latch/stay pressed. Press again to cut off battery. It should be pressed during charging and operation.
- **Blue LED**: Blinks status codes.
- **Green button**: Press to start up robot.

LED status codes:

================ ============= ==================================================
Time after start Blink pattern Meaning
================ ============= ==================================================
~ 5 s            ▄ ▄▄▄ ▄ ▄     Linux boot started
~ 45 s           ▄ ▄ ▄▄▄ ▄     Bringup script started (ROS 2 will launch shortly)
================ ============= ==================================================

Control robot using remote control
"""""""""""""""""""""""""""""""""""

.. figure:: ./figures/remote-control.png
   :alt: Remote control layout with KILLSWITCH (SB), lift (SC), power, and TELE buttons
   :align: center
   :width: 800px

- **Power on the RC**: press the On/Off button. The screen will show the model configuration and the RC battery state.
- **View robot battery**: press the ``TELE`` button (bottom-left). The display shows ``RxBt`` indicating the **robot battery level**.

.. caution::
   Ensure the robot battery does not drop below **9 V** during operation.

- **Enable motion**: move the **KILLSWITCH (SB)** to the **OFF** position (as shown in the figure). The robot will slightly shake to indicate it is enabled.
- **Drive**: use the indicated **CONTROL GIMBAL** to command the robot (forward/back/turn ~ up/down/left-right).

Control robot using keyboard teleop
-----------------------------------

MOTODO

Device Access
-------------
After connecting to the Wi‑Fi, you can access onboard computers as follows.

- SSH:

  .. code-block:: bash

   ssh analog@ad-r1m.local

- **Credentials**: user ``analog`` / password ``analog``

.. figure:: ./figures/ssh_rpi.png
   :alt: SSH to Raspberry Pi
   :align: center
   :width: 400px

..
        - VNC:
           - Install `VNC Viewer <https://www.realvnc.com/en/connect/download/viewer/>`_
           - Open Viewer → File → New Connection
           - **VNC Server**: ``ad-r1m.local``
           - **Credentials**: user ``analog`` / password ``analog``

        .. figure:: ./figures/vnc_connect.png
           :alt: VNC connection to Raspberry Pi
           :align: center
           :width: 400px

        .. note::
           Use VNC for GUI tasks like RViz, especially on the Raspberry Pi, SSH is sufficient for Portenta access and navigating the OS.

Raspberry Pi Runtime
--------------------

..
        Start docker container and RViz visualization
        ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        After connecting via VNC, run the start_rviz script to start the ROS 2 Docker container and visualize the robot.

        .. code-block:: bash

           # In the VNC session on Raspberry Pi
           ~/start_rviz.sh

        This launches RViz with the robot model, odometry, and camera laser scan visualization.

        .. figure:: ./figures/start_rviz.png
           :alt: RViz startup showing robot model and laser scan with odom fixed frame
           :align: center
           :width: 900px

        The RViz window starts with ``odom`` as the **Fixed Frame**, displaying the robot model and sensor data.

Mapping
~~~~~~~

To map the environment, run the mapping script and change the fixed frame to visualize the mapping process:

.. code-block:: bash

   # In the VNC session on Raspberry Pi
   ~/do_mapping.sh

.. figure:: ./figures/do_mapping.png
   :alt: RViz mapping view showing how to change fixed frame to map and mapping process
   :align: center
   :width: 900px

The figure shows how to change the fixed frame in RViz and observe the real-time mapping process.

Move the robot around the environment using the remote control to build the map, as shown in the animated demonstration below.

.. figure:: ./figures/do_mapping.gif
   :align: center
   :width: 1000px
   
   Robot mapping demonstration using remote control

The video shows the complete mapping process where the robot navigates the environment while building a real-time map using SLAM (Simultaneous Localization and Mapping).

Save the map
~~~~~~~~~~~~

After completing the mapping, open a new terminal and save the map to a file:

.. code-block:: bash

   # In the VNC session on Raspberry Pi
   ~/save_map.sh

This saves the map as ``office-map.pgm`` and ``office-map.yaml`` files in the ``~/ros_data/maps`` directory on the Raspberry Pi.

.. note::
   Save the map while the mapping node is still running and the map is being published. You can stop the mapping script after saving the map.


..
        Localization
        ~~~~~~~~~~~~

        To localize the robot on a previously saved map, run the localization script (make sure you stopped the ``do_mapping.sh`` script first):

        .. code-block:: bash

           # In the VNC session on Raspberry Pi
           ~/locate.sh

        This starts the **AMCL** (Adaptive Monte Carlo Localization) node to localize the robot on the saved map (``~/ros_data/maps/office-map.yaml``).

        .. note::
           After starting localization, press **2D Pose Estimate** in RViz and click on the map to set an approximate **initial pose** for the robot, this helps AMCL converge faster.
           You will see the robot's estimated position and orientation as a red arrow, and the covariance as a purple ellipse around it, see the localization demo. 

        .. figure:: ./figures/locate.gif
           :align: center
           :width: 1000px
           
           Robot localization demonstration

        .. caution::
           Stop ``do_mapping.sh`` before starting ``locate.sh``.

Navigation
~~~~~~~~~~~

.. code-block:: bash

   ~/do_navigation.sh

After running the navigation script, you will see the navigation nodes starting in the terminal, as shown in the figure below.

.. figure:: ./figures/navigate-sh.png
   :alt: Navigation terminal showing nodes starting
   :align: center
   :width: 600px

In RViz, you will see the **global costmap** and **local costmap** in light blue and purple.

.. figure:: ./figures/nav_view.png
   :alt: RViz navigation view showing global and local costmaps
   :align: center
   :width: 900px

**Sending a Navigation Goal**

To send a navigation goal, press **2D Nav Goal** in RViz and click on the map to set the desired destination for the robot, as shown in the video below.
You will see a blue arrow indicating the goal and the robot will start moving towards it.

.. figure:: ./figures/navigate.gif
   :align: center
   :width: 1000px
   
   Robot navigation demonstration using RViz
   
The video shows the robot navigating to the set goal using the navigation stack.

.. todo::

    * Brief overview of ROS 2 topics
