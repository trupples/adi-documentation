ADRD5161 Software Guide
=======================

Hardware Requirements
---------------------

* ADRD5161 board
* MAX32625PICO (or compatible) DAPLINK programmer
* battery pack or connection to USB-C supply (otherwise module is not powered)

Software Requirements
---------------------

* MaximSDK
* Zephyr Workspace

Install Tools
--------------

Install MSDK following the `MSDK User Guide
<https://analogdevicesinc.github.io/msdk/USERGUIDE/#download>`_. On WSL, we
recommend installing it as root in ``/MaximSDK``, instead of the default
user home install path.

If you do not have Zephyr SDK already set up, start by creating a Zephyr
workspace, following the `Zephyr Getting Started
<https://docs.zephyrproject.org/latest/develop/getting_started/index.html>`_
tutorial.

Setup Zephyr Workspace
----------------------

.. tab:: Create new west workspace
	      Set up virtualenv:

      .. code-block:: console

         $ mkdir zephyrproject
         $ cd zephyrproject
         $ python3 -m venv .venv
         $ source .venv/bin/activate

      Set up west workspace:

      .. code-block:: console

         $ pip install west
         $ west init -m https://github.com/analogdevicesinc/adrd5161-fw . # This might take a while - big download
         $ west update # This might take a while - big download
         $ west zephyr-export
         $ west packages pip --install
         $ cd zephyr
         $ west sdk install
         $ cd ..

.. tab:: Use existing west workspace

      You may reuse a pre-existing West workspace. This is especially convenient if working on other boards in the ADRD family.

      .. code-block:: console

         $ cd <path to west workspace>
         $ source .venv/bin/activate
         $ git clone https://github.com/analogdevicesinc/adrd5161-fw
         $ west config manifest.path adrd5161-fw
         $ west update

Enter the workspace and load the python virtual environment:

.. code-block:: console

   $ cd <path to west workspace>
   $ source .venv/bin/activate
   $ cd adrd5161-fw

Build and Flash the Firmware
----------------------------

The ADRD5161 firmware is based on Zephyr. The source code for the latest version: link.
The CiA 419 profile prescribes a standard set of CANopen objects and their function for BMS systems. While hand-crafting compatible CAN messages is possible, it is recommended to use an implementation of the CANopen and CiA 419 stack that exposes a simpler API, such as the Python ``canopen`` package or the ROS2 ``ros2_canopen`` package, exemplified in the following sections.

Build the firmware:

.. code-block:: console

   $ west build -p auto app

Flash the firmware (will build if necessary):

.. code-block:: console

   # Replace /MaximSDK/ with the path to MSDK
   $ west flash --openocd-search /MaximSDK/Tools/OpenOCD/scripts/ --openocd /MaximSDK/Tools/OpenOCD/openocd

Control through Python ``canopen``
----------------------------------
Install the python canopen package: 

.. code-block:: console
	pip install canopen

The code block below is a minimal example of accessing the BMS parameters through Python.

.. code-block:: python
	TODO: add python script or link ? 	


Control through ROS2 ``ros2_canopen``
-------------------------------------

TODO: implement ros wrapper
TODO: add images and proper refs
