ADRD5161
========

BMS module with CANopen CiA 419
"""""""""""""""""""""""""""""""

Overview
------------

	This board is a complete battery managment, charging and power delivery solution compatible with 3S Li-Ion battery packs, design for robotics, power tools and
	industrial equipment. It features and intuitive GUI for fast setup and monitoring, enabling out of the box operation with minimal configuration. The MAX17320 Fuel
	Gauge provides accurate state of charge and battery health data, while MAX77958 and MAX77961 enable customizable USB-C Charging up to 3A constant current. LTC4421
	Power Path controller prioritizes USB-C source over battery output and adds protection features. The system can deliver up to 240W (20A) output from battery and 
	integrates easily into larger systems via isolated CAN and isolated I2C. 

TODO: add high level info about software, add photo with the actual board

Features
--------------

	High Output Power Capability:

		- Delivers up to 240W (20A) continuous output power from the battery, suitable for driving motors, actuators and embedded control systems in robotics and industrial tools.
		
	USB Type-C Battery Pack CC/CV Charger - MAX77958EWV+ and MAX77961BEFV06+:
	
		- USB-C input power PDO negotiation (PD3.0 compliant)
		- Adaptive Input Current Limit
		- Thermal Regulation
		- Charging Constant Current (CC) up to 3A (default).

	Display and User Interface:

		- Equipped with display featuring a GUI and buttons for battery status and battery pack charging settings.

	Battery Compatibility:

		- Supports 3S Li-ion batteries default and 3S Li-polymer batteries.

	Fuel Gauge and Battery Pack Protector - MAX17320G20+:

		- Provides comprehensive protection including:
			- Overvoltage (temperature dependent)
			- Overcharge/Overdischarge/Short-Circuit Current
			- Over/Undertemperature
			- Undervoltage
			- Battery Internal Self-Discharge Detection
			- Ideal Diode Discharge During Charge Fault
			- Charging Prescriptions (JEITA)
			- Prequal Charge Control with CHG FET.
		- ModelGauge m5 EZ Algorithm (Percent, Capacity, Time-to-Empty/Full, Age, Cycle+™ Age Forecast).
		- Cell Balancing with Internal FETs.

	Output Powerpath Selector - LTC4421:

		- Automatically selects for the output power connector between USB-C (priority) source and Battery Pack
		- Adds to the system additional voltage, short circuit, and current limit protection for battery discharge.

	Microcontroller for system level control - MAX32662:

		- Arm Cortex-M4 Processor with FPU-Based Microcontroller featuring 256 KB Flash and 80KB SRAM.

	External communication:

		- Isolated CAN Transceiver - ADM3053
		- I2C isolator - ADUM1252.

	Additional screw terminal connector for adding external thermistors.

TODO: add high level info about software support

User Guides
-----------

.. toctree::
   :titlesonly:
   :maxdepth: 1

   */index
   hardware_configuration_page/hardware_configuration
   software_configuration_page/software_configuration
   system_setup_page/system_setup
   */

Additional Information and Useful Links
----------------
- :adi:`MAX77958 Product Page <MAX77958>`
- :adi:`MAX17320 Product Page <MAX17320>`
- :adi:`MAX77961B Product Page <MAX77961B>`
- :adi:`LTC4421 Product Page <LTC4421>`
- :adi:`MAX32662 Product Page <MAX32662>`
- :adi:`ADP5589 Product Page <ADP5589>`
- :adi:`ADM3053 Product Page <ADM3053>`
- :adi:`MAX31827 Product Page <MAX31827>`
- :adi:`MAX25231 Product Page <MAX25231>`
- :adi:`MAX4701 Product Page <MAX4701>`
- :adi:`ADUM1252 Product Page <ADUM1252>`


Help and Support
----------------

For questions and more information, please visit :ez:`/`.
