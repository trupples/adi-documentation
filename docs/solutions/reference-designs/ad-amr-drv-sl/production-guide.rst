AD-AMR-DRV-SL Production Guide
==============================

.. note::
	These steps are intended for when bringing up your own boards from scratch. These procedures are already done in the factory for boards sold by ADI.

Bootstrapping the TMC9660
-------------------------

.. warning::
	**USERS SHOULD NOT HAVE TO FOLLOW THE BOOTSTRAPPING SECTION.** These steps have already been done in the factory. Only follow them if you understand the TMC9660 bootstrap procedure. This permanently uses up one of the 4 configuration slots on the TMC9660. It cannot be undone and may only be overwritten at most 4 times on a given TMC9660 IC.

The :adi:`TMC9660` has reconfigurable I/O, clocking, LDO outputs, etc. Bootstrapping is the initial step of configuring these. Through bootstrapping, a configuration may be loaded in volatile memory, or "burned" to permanent nonvolatile memory to be loaded at power-up. These burn operations should be used sparingly, as they are limited to a total of 4 :abbr:`OTP (One Time Programmable)` configuration slots.

Configurations may be applied temporarily. If you want to try out a different configuration to the one provided as a default for this board, follow `AN-2601: TMC9660 Configuration and Bootstrapping
<https://www.analog.com/en/resources/app-notes/an-2601.html>`_ and reference the *board schematic* (TODO) and *default configuration* (TODO).

These instructions assume the TMC9660 is unconfigured (or has an invalid config, as described in the TMC9660 datasheet, section *Configuration Storage*) and thus enters bootstrap mode at startup.

Preparation:

* Download `UBLTools <https://www.analog.com/en/products/tmc9660.html#software-resources>`_
* Download bootstrap config file (TODO repo)
* Obtain a suitable UART probe (TODO *list of compatible probes*, custom *pinout*)

Steps:

#. Connect the UART probe to header P7 (*diagram*)
#. Take note of which serial interface the probe took. In the following commands, replace ``COM123`` with the actual serial port name.
#. Verify the TMC9660 is in bootstrap mode::

	> ublcli.exe --port COM123 inspect chip
	TODO: output

#. Upload the configuration and burn::

	> ublcli.exe --port COM123 write config --burn .\ad-amr-drv-bootstrap.toml
	TODO: output

Flashing the MAX32662
---------------------

The AD-AMR-DRV-SL firmware is based on Zephyr. The source code for the latest version may be found at https://github.com/analogdevicesinc/todoreponame .

Prerequisites:

* MSDK
* Zephyr workspace

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
         west init -m https://github.com/analogdevicesinc/todoreponame . # This might take a while - big download
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
         cd todoreponame

         west config manifest.path todoreponame
         west update

Enter the workspace and load the python virtual environment:

.. code-block:: bash

   cd <path to west workspace>
   source .venv/bin/activate
   cd todoreponame

Build the firmware:

.. code-block:: bash

   west build -p auto app

Flash the firmware (will build if necessary):

.. code-block:: bash

   # Replace /MaximSDK/ with the path to MSDK
   west flash --openocd-search /MaximSDK/Tools/OpenOCD/scripts/ --openocd /MaximSDK/Tools/OpenOCD/openocd

If your debug probe supports this, you may also flash the firmware by dragging and dropping the ``build/zephyr/zephyr.hex`` file onto the debug probe filesystem (?word?): (TODO example screenshots/gif)

Quick test
----------

TODO: How to quickly test a single board works

Test bench setup
----------------

TODO: How to set up an ADI production testing setup
