.. _mobile_base:

Mobile Robot
=============================



Mapping
-------

For creating a map, we use the `slam_toolbox <https://github.com/SteveMacenski/slam_toolbox>`_. 

Creating map
************

1. Stop all the running nodes with

.. code-block:: bash

	sudo systemctl stop ibt_ros2_bringup.service

2. Run the mapping bringup node, which enable only the mobile base, without navigation:

.. code-block:: bash

	sudo systemctl start ibt_ros2_mapping.service

.. warning:: Do not close this service until saving the map.

3. After starting the mapping, in the RViz or Foxglove, the initial stripes of the maps can be found

4. Navigate the robot around the environment with the joystick, in order to further build the map.

Saving map
**********

To save the map, use the following command

.. code-block:: bash

	ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap "name: data: '~/ibt'"

In the home dir will be appears
	1. ibt.pgm
	2. ibt.yaml

Restarting the navigation
**************************
After saving the map, you can restart the navigation stack with the following command:

1. Stop the mapping service:

.. code-block:: bash

	sudo systemctl stop ibt_ros2_mapping.service

2. Start the navigation service:

.. code-block:: bash

	sudo systemctl start ibt_ros2_bringup.service

It will be loaded the saved map with filename *ibt.yaml* and will start the navigation stack.

Battery state
-------------
The information about the battery state can be obtained from the topic `/battery_state` of type `sensor_msgs/msg/BatteryState`.
Data are represented in the following format:

.. code-block:: bash

	# Constants are chosen to match the enums in the linux kernel
	# defined in include/linux/power_supply.h as of version 3.7
	# The one difference is for style reasons the constants are
	# all uppercase not mixed case.

	# Power supply status constants
	uint8 POWER_SUPPLY_STATUS_UNKNOWN = 0
	uint8 POWER_SUPPLY_STATUS_CHARGING = 1
	uint8 POWER_SUPPLY_STATUS_DISCHARGING = 2
	uint8 POWER_SUPPLY_STATUS_NOT_CHARGING = 3
	uint8 POWER_SUPPLY_STATUS_FULL = 4

	# Power supply health constants
	uint8 POWER_SUPPLY_HEALTH_UNKNOWN = 0
	uint8 POWER_SUPPLY_HEALTH_GOOD = 1
	uint8 POWER_SUPPLY_HEALTH_OVERHEAT = 2
	uint8 POWER_SUPPLY_HEALTH_DEAD = 3
	uint8 POWER_SUPPLY_HEALTH_OVERVOLTAGE = 4
	uint8 POWER_SUPPLY_HEALTH_UNSPEC_FAILURE = 5
	uint8 POWER_SUPPLY_HEALTH_COLD = 6
	uint8 POWER_SUPPLY_HEALTH_WATCHDOG_TIMER_EXPIRE = 7
	uint8 POWER_SUPPLY_HEALTH_SAFETY_TIMER_EXPIRE = 8

	# Power supply technology (chemistry) constants
	uint8 POWER_SUPPLY_TECHNOLOGY_UNKNOWN = 0
	uint8 POWER_SUPPLY_TECHNOLOGY_NIMH = 1
	uint8 POWER_SUPPLY_TECHNOLOGY_LION = 2
	uint8 POWER_SUPPLY_TECHNOLOGY_LIPO = 3
	uint8 POWER_SUPPLY_TECHNOLOGY_LIFE = 4
	uint8 POWER_SUPPLY_TECHNOLOGY_NICD = 5
	uint8 POWER_SUPPLY_TECHNOLOGY_LIMN = 6

	std_msgs/Header  header
			builtin_interfaces/Time stamp
					int32 sec
					uint32 nanosec
			string frame_id
	float32 voltage          # Voltage in Volts (Mandatory)
	float32 temperature      # Temperature in Degrees Celsius (If unmeasured NaN)
	float32 current          # Negative when discharging (A)  (If unmeasured NaN)
	float32 charge           # Current charge in Ah  (If unmeasured NaN)
	float32 capacity         # Capacity in Ah (last full capacity)  (If unmeasured NaN)
	float32 design_capacity  # Capacity in Ah (design capacity)  (If unmeasured NaN)
	float32 percentage       # Charge percentage on 0 to 1 range  (If unmeasured NaN)
	uint8   power_supply_status     # The charging status as reported. Values defined above
	uint8   power_supply_health     # The battery health metric. Values defined above
	uint8   power_supply_technology # The battery chemistry. Values defined above
	bool    present          # True if the battery is present

	float32[] cell_voltage   # An array of individual cell voltages for each cell in the pack
							# If individual voltages unknown but number of cells known set each to NaN
	float32[] cell_temperature # An array of individual cell temperatures for each cell in the pack
							# If individual temperatures unknown but number of cells known set each to NaN
	string location          # The location into which the battery is inserted. (slot number or plug)
	string serial_number     # The best approximation of the battery serial number


Docking
-------

Work in progress

Safety areas
------------

You can select the safety areas in accordance with the environment, where the robot will be allowed to move.
To do this a proper interface is provided

.. code-block:: bash

	ros2 service call /sick/setOutput ibt_ros2_interfaces/srv/SetAttrAll "clas: 0x72
	instance: 1  
	data:
	- 255
	- 0
	- 0
	- 0
	- 0
	- 0
	- 0
	- 0
	- 0
	- 0"
