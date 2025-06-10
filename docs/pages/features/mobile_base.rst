.. _mobile_base:

Mobile Robot
=============================

Navigation
----------
The mobile base is equipped with the navigation stack, which allows the robot to navigate autonomously in the environment. 
The navigation stack is based on ROS 2 and uses the `nav2 <https://docs.nav2.org/>`_ framework.

You can control the robot's navigation using the following actions:

Actions:
	- /navigate_to_pose: `nav2_msgs/action/NavigateToPose <https://github.com/ros-navigation/navigation2/blob/humble/nav2_msgs/action/NavigateToPose.action>`_

.. warning:: Remember to be sure that the robot is located in the environment before proceeding with new goals

Localization
-------------
The first localization of robot in the map is required to use the naviation stack in proper way. 
It can be made by:

1. Publishing a pose on the topic

	- /initialpose : `geometry_msgs/msg/PoseWithCovarianceStamped <https://github.com/ros2/common_interfaces/blob/humble/geometry_msgs/msg/PoseWithCovarianceStamped.msg>`_

	.. image:: ../img/local.png
		:width: 400
		:align: center

2. Turn on the robot in the **origin** of the map, which is the point (0, 0, 0) in the map coordinates.

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

.. warning:: The default name of the map is *ibt*. If you want to save the map with a different name, change the `name` field in the launch file `ibt_ros2_bringup/launch/neo.launch.py` or simply do a replacement with the command above.

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
The information about the battery state can be obtained from 

Publisher
	- /battery_state: `sensor_msgs/msg/BatteryState <https://github.com/ros2/common_interfaces/blob/humble/sensor_msgs/msg/BatteryState.msg>`_

Docking
-------

Work in progress

Safety areas
------------

You can select the safety areas in accordance with the environment, where the robot will be allowed to move.

Services server
	- /sick/setOutput: `ibt_ros2_interfaces/srv/SetAttrAll <https://github.com/InnoboticsSRL/ibt_ros2_interfaces/blob/humble/srv/SetAttrAll.srv>`_