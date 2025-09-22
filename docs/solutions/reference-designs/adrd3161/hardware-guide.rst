ADRD3161 Hardware Guide
=======================

.. todo:: Board image with annotations

Switches
--------

.. todo:: RST_TMC, RST_MCU, address DIP switch

.. _adrd3161_cable_encoder:

Encoder cable
-------------

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


.. _adrd3161_cable_can:

CAN cable
---------

The ADRDx161 board family communicates via CAN bus. The two headers P8, P9 allow for daisy-chaining CAN devices.

.. graphviz:: res/cable-can.gv
