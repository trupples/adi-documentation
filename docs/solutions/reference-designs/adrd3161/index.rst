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
* Encoder interface, isolated: Custom header described below (8 pin)
* CAN, isolated: 2x Custom header described below (4 pin)

Encoder cable
'''''''''''''

Header P16 provides 5V power to encoders and has isolated inputs corresponding to ABN and Digital Hall encoders.

Depending on the used motor and its wiring, you may need to customize the encoder cable. Some examples:

.. tab:: ADI Trinamic ABN (Default)

   Encoder cable for ADI Trinamic ABN encoders, such as the
   `ADI Trinamic TMCS <https://www.analog.com/en/product-category/motor-encoders.html>`_ series, and the builtin encoders on
   `ADI Trinamic QSH <https://www.analog.com/en/product-category/stepper-motors.html>`_ series stepper motors.
   
   .. graphviz:: res/cable-encoder-stepper.gv

.. tab:: BLDC with Hall
   
   Encoder cable for a generic BLDC with Digital Hall sensor configuration, with no special encoder connector, such as the
   `ADI Trinamic QBL <https://www.analog.com/en/product-category/bldc-brushless-dc-motors.html>`_ series.

   .. graphviz:: res/cable-encoder-bldc.gv

.. tab:: BLDC with Hall, VESC-like connector

   Encoder cable for using a Digital Hall sensor with a VESC-like encoder connector (6-pin JST PH), such as the
   `REV-21-1650 <https://revrobotics.eu/rev-21-1650/>`_.
   
   .. graphviz:: res/cable-encoder-bldc-vesc.gv

.. tab:: BLDC with Hall & ADI Trinamic ABN
   
   Encoder cable for using both an ADI Trinamic ABN encoder and a Digital Hall sensor.

   .. graphviz:: res/cable-encoder-bldc-abn.gv

CAN cable
'''''''''

The ADRDx161 board family communicates via CAN bus. The two headers P8, P9 allow for daisy-chaining CAN devices.

.. graphviz:: res/cable-can.gv


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
    production-guide
    user-guide
    internals
    canopen

Help and Support
----------------

TODO
