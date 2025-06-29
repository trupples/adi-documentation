AD-AMR-DRV-SL (partno TBD)
==========================

Open Mobile Robot Motor Control module with CANopen CiA 402
"""""""""""""""""""""""""""""""""""""""""""""""""""""""""""

Introduction
------------

The AD-AMR-DRV-SL is an FOC motor driver board based on the TMC9660, capable of
driving Stepper, BLDC, and brushed DC motors with optional quadrature (ABN)
and/or Hall encoders. It communicates via CANopen CiA 402 over 500 kbaud CAN.
For development, the TMC9660 may also be accessed directly via UART and thus
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
* Encoder interface, isolated: Custom header (8 pin)
* CAN, isolated: 2x Custom header (4 pin)

.. todo::
   Diagrams

.. todo::
   Custom connector docs 

Required Hardware
-----------------

* AD-AMR-DRV-SL
* Stepper / BLDC / DC motor. Documented output represents the `QSH5718-51-28-101-10k <https://www.analog.com/qsh5718>`_ Stepper and `TODO` BLDC, respectively.
* power supply (9 .. 70 VDC)
* CAN cable
* `MAX32625PICO <https://www.analog.com/max32625pico>`_ (or compatible) debug probe

User Guides
-----------

.. toctree::
    production-guide
    user-guide
    internals
    canopen

Help and Support
----------------

TODO
