ADRD3161 Quick Start Guide
==========================

Connect a motor and encoder
---------------------------

Connect the motor windings to screw terminal block P3. The following tables exemplify typical connections based on the type of your motor.

.. note::
	Wire colors are not standard and vary between motors.
	These tables reflect the wiring of Trinamic motors.
	Check your motor's datasheet!

.. tab:: Stepper

        =========== ========== ========
        Motor phase Wire color Terminal
        =========== ========== ========
        A+          Black      UX1
        A-          Green      VX2
        B+          Red        WY1
        B-          Blue       Y2
        =========== ========== ========

.. tab:: Brushless DC

        =========== ========== ========
        Motor phase Wire color Terminal
        =========== ========== ========
        U           Yellow     UX1
        V           Red        VX2
        W           Black      WY1
        =========== ========== ========

.. tab:: Brushed DC

        =========== ========== ========
        Motor phase Wire color Terminal
        =========== ========== ========
        M+          Red        UX1
        M-          Black      VX2
        =========== ========== ========

Connect the motor's position encoder to terminal P16. If you need a custom encoder cable, check the hardware guide for a number of common connection diagrams.

Control using TMCL-IDE
----------------------

`TMCL-IDE
<https://www.analog.com/en/resources/evaluation-hardware-and-software/motor-motion-control-software/tmcl-ide.html>`_
is the main software package for Trinamic parts. The TMC9660 may be directly interfaced with from TMCL-IDE if connected via UART. The TMC9660 may be controlled via UART and by the MAX32662 at the same time, seamlessly. This usage is helpful for debugging and lower level access to the TMC9660 than the CANopen interface allows.

Prerequisites:

* Install the latest version of `TMCL-IDE <https://www.analog.com/en/resources/evaluation-hardware-and-software/motor-motion-control-software/tmcl-ide.html>`_
* Obtain a suitable UART probe (:adi:`MAX32625PICO` or other MAXDAP compatible)

Steps:

#. Connect a UART probe to header P7.
#. In TMCL-IDE, select the corresponding serial port from the *Connected devices* menu on the left side.
#. Use the following settings to initiate a connection:
	* 115200 baud
	* search IDs from 1 to 1

Initially, use the TMCL-IDE TMC9660 tuning wizard to obtain proper motor control parameters. Carefully go through each step, up to and including encoder configuration.

After you have found suitable motor control parameters, use the "Position/Velocity/Torque mode" tools from the menu to move it.

Control via CAN bus
-------------------

The ADRD3161 implements CANopen, with the CiA 402 profile for motor drives. The device is interoperable with other CANopen devices, but has limited applicability on a CAN bus that is not CANopen, unless carefully configured.

The boards' CANopen node IDs can be configured using the DIP switch S2. The CANopen node ID is ``0x10`` + the binary value of the DIP switches. Assigning node IDs via CANopen LSS is planned as a future feature. Most CAN messages sent and received by a specific device will have the lower 7 bits set to the node ID, allowing for easy identification when monitoring the bus:

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
	Blinking 2.5 Hz      PRE-OPERATIONAL
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

* The RUN LED (green) should be blinking at 2.5Hz
* The FAULT LED (red) should be off

Set up CAN adapter
''''''''''''''''''

On the software side, CAN communication depends on the OS and used hardware interface. The following guide assumes a **Linux** machine. On Windows, this setup can be achieved in WSL with USB forwarding of CAN adapters.

Install / load the appropriate kernel modules for your CAN adapter:

.. tab:: gs_can

        Many off-the-shelf USB-CAN adapters need the ``gs_usb`` driver which is provided by many common Linux distros.

        If the driver is loaded, you should automatically see a ``can0`` network interface being listed when you run ``ip link``.

.. tab:: slcan

        Adapters using the ``slcan`` protocol and driver, such as the :adi:`ADRD4161`, need the slcan daemon to run. On the ADRD4161, run::

                sudo slcand -o -c -f -t hw -s 6 -S 2000000 /dev/ttyAMA0 can0

        Other devices will need a different set of arguments to ``slcand``.

Configure and bring up the CAN interface (replace can0 with the name of the interface, if different)::

	$ ip link set can0 down
	$ ip link set can0 type can bitrate 500000
	$ ip link set can0 up

Additionally, the ``can-utils`` package has a useful set of tools which aid in bus monitoring and troubleshooting.

If connected to an ADRD3161 board, you should see regular heartbeat messages using `candump`::

	$ candump can0
	can0  717   [1]  7F
	can0  717   [1]  7F
	can0  717   [1]  7F
	...

In the above snippet, ``717`` is the CAN message ID, and it corresponds to node ID ``0x17``. The following content of each line signifies a message length of 1 bytes and hexadecimal content ``7F``. This is an CANopen NMT heartbeat message signaling the node is in the ``PRE-OPERATIONAL`` state.

To remotely reset all nodes on the bus, run::

	$ cansend can0 000#0081

To remotely reset a specific node, with ID xx, run (after replacing xx with the ID in hexadecimal)::

	$ cansend can0 000#xx81

Run a simple Python example
'''''''''''''''''''''''''''

The following example code uses the Python ``canopen`` library to communicate with the ADRD3161 and spin it in velocity mode.

.. code-block:: python3

   import canopen
   import time

   node_address = 0x17 # Change this to your node address

   net = canopen.Network()
   net.connect(channel='can0', interface='socketcan') # Change if using another adapter
   node = canopen.BaseNode402(node_address, 'adrd3161.eds')
   net.add_node(node)

   # Wait for node to say something, or block if there are communication problems
   node.nmt.wait_for_heartbeat()


   # Set up CiA 402 and set mode of operation to Cyclic Synchronous Velocity
   node.load_configuration()
   node.setup_402_state_machine()
   node.state = 'SWITCH ON DISABLED'
   node.sdo['Modes of Operation'] = 9

   time.sleep(1)
   node.nmt.state = 'OPERATIONAL'
   time.sleep(1)

   # Engage motor drive
   node.state = 'OPERATION ENABLED'
   time.sleep(2)

   # Send velocity commands
   RPM = 4000 # Scaling between internal units and real-life units
   node.sdo['Target Velocity'].raw = 10 * RPM
   time.sleep(1)
   node.sdo['Target Velocity'].raw = -20 * RPM
   time.sleep(1)
   node.sdo['Target Velocity'].raw = 30 * RPM
   time.sleep(1)
   node.sdo['Target Velocity'].raw = 0

   # Stop
   node.state = 'SWITCH ON DISABLED'

   node.nmt.state = 'PRE-OPERATIONAL'

