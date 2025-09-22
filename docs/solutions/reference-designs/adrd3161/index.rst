ADRD3161
========

Motor Control module with CANopen CiA 402
"""""""""""""""""""""""""""""""""""""""""

Introduction
------------

The :adi:`ADRD3161` is an FOC motor driver board based on the :adi:`TMC9660`, capable of
driving Stepper, BLDC, and brushed DC motors with optional quadrature (ABN)
and/or Hall encoders. It communicates via CANopen CiA 402 over 500 kbaud CAN.
For development, the :adi:`TMC9660` may also be accessed directly via UART and thus
controlled / configured via TMCL-IDE.

Specifications
--------------

* 9 - 70 V DC supply
* 10 A max phase current (conservative)
* Supported motor types: 3-phase BLDC/PMSM, 2-phase bipolar stepper, brushed DC
* Supported encoder types: ABN (= ABZ, ABI, quadrature), Digital Hall
* Isolated CAN bus communication, CANopen CiA 402
* Option to solder a brake chopper resistor
* Option to use an electromechanical brake (only in the BLDC configuration)

Supporting hardware:

* :adi:`TMC9660` 70V Smart Gate Driver with Servo (FOC) Controller in HW and Buck Converter
* :adi:`MAX32662` Arm Cortex-M4 Processor with FPU-Based Microcontroller (MCU) with 256KB Flash and 80KB SRAM
* :adi:`ADM3053` Signal and Power Isolated CAN Transceiver with Integrated Isolated DC-to-DC Converter

Connections:

* Power input: screw terminal
* Motor connection: screw terminal
* Debug ports (x2): SWD header
* Encoder interface, isolated: Custom header (8 pin). Described in :ref:`adrd3161_cable_encoder`.
* CAN, isolated: 2x Custom header described below (4 pin). Described in :ref:`adrd3161_cable_can`.

Required Hardware
-----------------

* ADRD3161
* Stepper / BLDC / DC motor. Documented output represents the `QSH5718-51-28-101-10k <https://www.analog.com/qsh5718>`_ Stepper and `TODO` BLDC, respectively.
* DC power supply (9 .. 70 VDC)
* `MAX32625PICO <https://www.analog.com/max32625pico>`_ (or compatible) debug probe
* Cables described in the previous section

User Guides
-----------

.. toctree::

   quick-start-guide
   hardware-guide
   software-guide
   troubleshooting
   canopen

Help and Support
----------------

.. todo:: Support channel
