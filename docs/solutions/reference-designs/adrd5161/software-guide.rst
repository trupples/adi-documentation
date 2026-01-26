ADRD5161 Software Guide
=======================

Hardware Requirements
---------------------

* ADRD5161 board
* MAX32625PICO (or compatible) DAPLINK programmer
* battery pack or connection to USB-C supply (otherwise module is not powered)
* CAN to USB adaptor to connect to the PC, to view data on CAN bus

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

	$ pip install canopen

Before running any examples, make sure the CAN network is properly set up.

.. code-block:: console

	$ ip link set can0 down
	$ ip link set can0 type can bitrate 500000
	$ ip link set can0 up

The code block below is a minimal example of accessing the BMS parameters through Python.

.. code-block:: python

	#!/usr/bin/env python3
	import canopen
	import time
	import sys
	import struct

	# Configuration
	CAN_INTERFACE = 'socketcan'
	CAN_CHANNEL = 'can0'
	CAN_BITRATE = 500000
	NODE_ID = 2


	def setup_network():
	"""Initialize CANopen network and node"""
	try:
		# Create network
		network = canopen.Network()

		# Connect to CAN bus
		network.connect(interface=CAN_INTERFACE, channel=CAN_CHANNEL, bitrate=CAN_BITRATE)
		print(f"Connected to {CAN_INTERFACE} on {CAN_CHANNEL} at {CAN_BITRATE} bit/s")

		# Add remote node
		node = canopen.RemoteNode(NODE_ID, object_dictionary=None)
		network.add_node(node)
		print(f"Added node {NODE_ID}")

		return network, node
	except Exception as e:
		print(f"Error setting up network: {e}")
		sys.exit(1)


	def read_sdo_auto(node, index, subindex=0, signed=False):
	"""Read a value via SDO and auto-detect size"""
	try:
		data = node.sdo.upload(index, subindex)
		data_len = len(data)

		if data_len == 1:
		fmt = '<b' if signed else '<B'
		elif data_len == 2:
		fmt = '<h' if signed else '<H'
		elif data_len == 4:
		fmt = '<i' if signed else '<I'
		elif data_len == 8:
		fmt = '<q' if signed else '<Q'
		else:
		return None, f"Unsupported data length: {data_len} bytes", data_len

		value = struct.unpack(fmt, data)[0]
		return value, None, data_len
	except Exception as e:
		return None, str(e), 0

	def charger_status_str(status):
	"""Convert charger status to string"""

	if status == 0:
		return "?"
	if status == 1:
		return "charging"
	if status == 2:
		return "fault"
	if status == 3:
		return "off"
	if status == 4:
		return "full"
	return None


	def read_od_values(node):
	"""Read all Object Dictionary values from the device"""

	print("\n" + "=" * 60)
	print("Reading Object Dictionary Values")
	print("=" * 60)

	# Battery Cell Voltages (0x2060, array of 4)
	print("\n[Battery Cell Voltages - 0x2060]")
	for i in range(4):
		voltage, error = read_sdo_auto(node, 0x2060, i + 1, signed=False)
		if error:
		print(f"  Cell {i + 1}: Error - {error}")
		else:
		print(f"  Cell {i + 1}: {voltage} mV ({voltage / 1000:.3f} V)")

	# Current (0x2071) - signed
	print("\n[Current - 0x2071]")
	current, error, size = read_sdo_auto(node, 0x2071, 0, signed=True)
	if error:
		print(f"  Current: Error - {error}")
	else:
		print(f"  Current: {current} uA ({current / 1000:.3f} mA)")

	# Battery Status (0x6000)
	print("\n[Battery Status - 0x6000]")
	status, error = read_sdo_auto(node, 0x6000, 0, signed=False)
	if error:
		print(f"  Battery Status: Error - {error}")
	else:
		print("Battery status: discharging" if status == 1 else "Battery status: charging")

	# Charger Status (0x6001)
	print("\n[Charger Status - 0x6001]")
	status, error = read_sdo_auto(node, 0x6001, 0, signed=False)
	if error:
		print(f"  Charger Status: Error - {error}")
	else:
		print(f"  Charger Status is {charger_status_str(status)}")

	# Temperature (0x6010) - signed
	print("\n[Temperature - 0x6010]")
	temp, error, size = read_sdo_auto(node, 0x6010, 0, signed=True)
	if error:
		print(f"  Temperature: Error - {error}")
	else:
		print(f"  Temperature: {temp} °C")

	# Ah Returned During Last Charge (0x6052)
	print("\n[Ah Returned During Last Charge - 0x6052]")
	ah, error = read_sdo_auto(node, 0x6052, 0, signed=False)
	if error:
		print(f"  Ah Returned: Error - {error}")
	else:
		print(f"  Ah Returned: {ah} mAh ({ah / 1000:.3f} Ah)")

	# Battery Voltage (0x6060)
	print("\n[Battery Voltage - 0x6060]")
	voltage, error = read_sdo_auto(node, 0x6060, 0, signed=False)
	if error:
		print(f"  Battery Voltage: Error - {error}")
	else:
		print(f"  Battery Voltage: {voltage} mV ({voltage / 1000:.3f} V)")

	# Battery State of Charge (0x6081)
	print("\n[Battery State of Charge - 0x6081]")
	soc, error = read_sdo_auto(node, 0x6081, 0, signed=False)
	if error:
		print(f"  Battery SOC: Error - {error}")
	else:
		print(f"  Battery SOC: {soc}%")

	print("\n" + "=" * 60)


	def main():

	print("ADRD5161 CAN Example \n")
	print("-" * 60)

	# Setup network
	print("Connect to CAN network\n")
	network, node = setup_network()

	try:
		read_od_values(node)
		print("No Errors. System test passed.\n")
	except Exception as e:
		print(f"\nError: {e}")
		import traceback
		traceback.print_exc()
	finally:
		# Cleanup
		print("\nDisconnecting...")
		network.disconnect()
		print("Disconnected")


	if __name__ == "__main__":
	main()



Control through ROS2 ``ros2_canopen``
-------------------------------------

TODO: implement ros wrapper
TODO: add images and proper refs
