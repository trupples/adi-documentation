AD-AMR-DRV-SL (partno TBD)
==========================

Open Mobile Robot Motor Control module with CANopen CiA 402
"""""""""""""""""""""""""""""""""""""""""""""""""""""""""""

Introduction
------------

The AD???? is a versatile FOC motor driver based on the TMC9660, capable of
driving Stepper, BLDC, and brushed DC motors with optional quadrature (ABN)
and/or Hall encoders. It communicates via CANopen CiA 402 over 500 kbaud CAN.
For development, the TMC9660 may also be accessed directly via UART and thus
controlled / configured via TMCL-IDE.

Specifications
--------------

- 9 - 70 V DC supply
- 10 A phase output
- Isolated CAN bus communication, CANopen CiA 402
- TMC9660 motor driver IC
- MAX32662 microcontroller
- Supported motor types: 3-phase BLDC/PMSM, 2-phase bipolar stepper, brushed DC
- Supported encoder types: ABN (= ABZ, ABI, quadrature), Digital Hall
- Optional solderable brake chopper resistor
- Optional electromechanical brake in BLDC configuration

Connections:

- Power input: screw terminal
- Motor connection: screw terminal
- Debug ports (x2): SWD header
- Encoder interface, isolated: Custom header (8 pin)
- CAN, isolated: 2x Custom header (4 pin)

.. todo::
   Diagrams

.. todo::
   Custom connector docs 

Required Hardware
-----------------

- AD-AMR-DRV-SL
- Stepper / BLDC / DC motor. Documented output represents the `QSH5718-51-28-101-10k <https://www.analog.com/qsh5718>`_ Stepper and `TODO` BLDC, respectively.
- power supply (9 .. 70 VDC)
- CAN cable
- `MAX32625PICO <https://www.analog.com/max32625pico>`_ (or compatible) debug probe

System Setup
------------

TODO

Firmware updates
~~~~~~~~~~~~~~~~

The BOARDNAME firmware is based on Zephyr. The source code for the latest version may be found at https://github.com/analogdevicesinc/thisreponame .

To build and flash firmware, you will need a Zephyr setup and MSDK for target-specific flashing support.

Install MSDK following the `MSDK User Guide
<https://analogdevicesinc.github.io/msdk/USERGUIDE/#download>`_. On WSL, we
recommend installing it as root in ``/MaximSDK``, instead of the default
user home install path.

.. tabs::

   .. tab:: Create new west workspace

      If you do not have Zephyr SDK already set up, start by creating a Zephyr
      workspace, following the `Zephyr Getting Started
      <https://docs.zephyrproject.org/latest/develop/getting_started/index.html>`_
      tutorial. In short, the following commands should suffice:

      .. code-block:: bash

         # Set up virtualenv
         mkdir zephyrproject
         cd zephyrproject
         python3 -m venv .venv
         source .venv/bin/activate

         # Set up west, firmware repo, Zephyr SDK
         pip install west
         west init -m https://github.com/analogdevicesinc/thisreponame . # This might take a while - big download
         west update # This might take a while - big download
         west zephyr-export
         west packages pip --install
         cd zephyr
         west sdk install
         cd ..

   .. tab:: Use existing west workspace

      You may reuse a pre-existing West workspace. This is especially convenient if working on other boards in the ADRD family.

      .. code-block:: bash

         cd <path to west workspace>
         source .venv/bin/activate
         cd thisreponame

         west config manifest.path thisreponame
         west update

To build the firmware, run:

.. code-block:: bash

   cd <path to west workspace>
   source .venv/bin/activate
   cd thisreponame

   west build -p auto app

To flash the firmware (will build, if necessary), run:

.. code-block:: bash

   cd <path to west workspace>
   source .venv/bin/activate
   cd thisreponame

   # Replace /MaximSDK/ with the path to MSDK
   west flash --openocd-search /MaximSDK/Tools/OpenOCD/scripts/ --openocd /MaximSDK/Tools/OpenOCD/openocd

Motor Tuning Procedure
~~~~~~~~~~~~~~~~~~~~~~

.. note::
   The current motor tuning procedure is rather crude, unwieldy, and will be
   streamlined in the near future.

.. todo::
   Add diagrams / photos / screenshots for each step

1. Connect a UART probe to header P7
2. Open TMCL-IDE and connect to the corresponding serial port
3. Use the TMC9660 tuning wizard until the drive parameters are satisfactory
4. Disconnect in TMCL-IDE, and move the debug probe over to header P6
5. Open a serial terminal
6. Press Ctrl-U to erase junk that might have accumulated on UART
7. Type and run the ``tmc_params save`` command

All parameters configured by TMCL-IDE will be saved to the drive board's flash
and be loaded at each following boot.

User Guides
-----------

.. toctree::
    od

Help and Support
----------------

TODO
