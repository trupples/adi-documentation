AD-AMR-DRV-SL User Guide
========================

Connect and configure a motor
-----------------------------

.. note::
	The current tuning procedure is admittedly crude. It will be simplified in the future.

Prerequisites:

* Install the latest version of TMCL-IDE
* Obtain a suitable UART probe (TODO *list of compatible probes*, custom *pinout*)

.. todo::
	* Stepper, BLDC, DC connection diagrams (tabs)
	* Encoder connector diagrams
	* TMCL-IDE tuning steps explained
	* Elaborate all points

1. Connect a UART probe to header P7
2. Open TMCL-IDE and connect to the corresponding serial port
3. Use the TMC9660 tuning wizard until the drive parameters are satisfactory
4. Disconnect in TMCL-IDE, and move the debug probe over to header P6
5. Open a serial terminal
6. Press Ctrl-U to erase junk that might have accumulated on UART
7. Type and run the ``tmc_params save`` command

All parameters configured by TMCL-IDE will be saved to the drive board's flash
and be loaded at each following boot.


Control through TMCL-IDE
------------------------

`TMCL-IDE
<https://www.analog.com/en/resources/evaluation-hardware-and-software/motor-motion-control-software/tmcl-ide.html>`_
is the main software package for Trinamic parts. The TMC9660 may be directly interfaced with from TMCL-IDE if connected via UART. The TMC9660 may be controlled via UART and by the MAX32662 at the same time, seamlessly. This usage is helpful for debugging and lower level access to the TMC9660 than the CANopen interface allows.

#. Connect a UART probe to header P7.
#. In TMCL-IDE, select the corresponding serial port from the *Connected devices* menu on the left side.
#. Use the following settings to initiate a connection:
	* 115200 baud
	* search IDs from 1 to 1

Once connected, the list on the left should expand, revealing a useful set of TMCL-IDE features.

TODO: how to reconcile the "control" and "tuning" via TMCL-IDE parts, given they start out the same, have the same prerequisites, but are two separate user stories / use cases?

Connect via CAN bus
-------------------

The AD-AMR-DRV-SL implements CANopen, with the CiA 402 profile for motor drives. The device is interoperable with other CANopen devices, but has limited applicability on a CAN bus that is not CANopen, unless carefully configured.

The boards' CANopen node IDs can be configured using the DIP switches *S2* (TODO: diagram). The node ID is ``0x10`` + the binary value of the DIP switches. Assigning node IDs via CANopen LSS is planned as a future feature. Most CAN messages sent and received by a specific device will have the lower 7 bits set to the node ID, allowing for easy identification when monitoring the bus.

.. table:: DIP switch, CANopen node ID, CAN frame ID correspondence

	========= ======== ================
	S2 config Node ID  CAN frame IDs
	========= ======== ================
	000       ``0x10`` ``x10``, ``x90``
	001       ``0x11`` ``x11``, ``x91``
	010       ``0x12`` ``x12``, ``x92``
	011       ``0x13`` ``x13``, ``x93``
	100       ``0x14`` ``x14``, ``x94``
	101       ``0x15`` ``x15``, ``x95``
	110       ``0x16`` ``x16``, ``x96``
	111       ``0x17`` ``x17``, ``x97``
	========= ======== ================

The LEDs on the board signal the CANopen status according to CiA 303-3.

.. table:: CiA 303-3 RUN LED patterns

	==================== ========================================
	RUN LED (Green, DS2) CANopen NMT state
	==================== ========================================
	Solid on             OPERATIONAL
	Blinking 1 Hz        PRE-OPERATIONAL
	Single flash         STOPPED
	Flickering 10 Hz     AutoBaud/LSS (not currently implemented)
	==================== ========================================

.. table:: CiA 303-3 ERR LED patterns

	================== ================================================
	ERR LED (Red, DS3) CANopen error state
	================== ================================================
	Off                No error
	Single flash       Warning limit reached: too many CAN error frames
	Double flash       Error Control Event (irrelevant)
	Triple flash       Sync timeout
	Flickering 10 Hz   AutoBaud/LSS (not currently implemented)
	Solid on           CAN bus off
	================== ================================================

Check that the board has started up correctly:

* The RUN LED (green) should be constantly on
* The FAULT LED (red) should be off

---


On the software side, CAN communication depends on the OS and used hardware interface. The following guide assumes a **Linux** machine. On Windows, this setup can be achieved in WSL with USB forwarding of CAN adapters.

.. todo:: Write or link to a WSL CAN setup guide

Install / load the appropriate kernel modules for your CAN adapter:

.. tabs::

	.. tab:: gs_can

		Many off-the-shelf adapters (TODO: list a handful) need the ``gs_can`` driver which is widely available, in many cases even already installed or built into the kernel.

		.. todo:: Elaborate instructions for gs_can

Configure and bring up the CAN interface (replace can0 with the name of the interface, if different)::

	$ ip link set can0 down
	$ ip link set can0 type can bitrate 500000
	$ ip link set can0 up

Additionally, the ``can-utils`` package has a useful set of tools which aid in bus monitoring and troubleshooting.

If connected to an AD-AMR-DRV-SL board, you should see regular heartbeat messages using `candump`::

	$ candump can0
	can0  716   [1]  05
	can0  716   [1]  05
	can0  716   [1]  05
	...

In the above snippet, ``716`` is the CAN message ID, and it corresponds to node ID ``0x16``. The following content of each line signifies a message length of 1 bytes and hexadecimal content ``05``. This is an CANopen NMT heartbeat message signaling the node is in the ``OPERATIONAL`` state.

To remotely reset all nodes on the bus, run::

	$ cansend can0 000#0081

To remotely reset a specific node, with ID xx, run (after replacing xx with the ID in hexadecimal)::

	$ cansend can0 000#xx81

The CiA 402 profile prescribes a standard set of CANopen objects and their function for motor drives. While hand-crafting compatible CAN messages is possible, it is recommended to use an implementation of the CANopen and CiA 402 stack that exposes a simpler API, such as the Python `canopen` package or the ROS2 `ros2_canopen` package, exemplified in the following sections.

Control through Python ``canopen``
----------------------------------

TODO

Control through ROS2 ``ros2_canopen``
-------------------------------------

TODO
