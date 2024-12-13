.. _ros2_nav2:

Nav2
=============================

Nav2 is the navigation stack for mobile robots designed for ROS 2. This package is considered both in research and as well as in industrial applications because of the modularity and the wide range of algorithms that it provides to the user. Nav2 is still considered as an evolving project and new features can be expected every fortnight. 

In this tutorial, we just explain on how Nav2 is used in our robots. We encourage the users to check out the detailed `official Nav2 documentation page <https://navigation.ros.org/>`_ for understanding the concepts surronding it.

Mapping
-------

Map is one of the essential component representing the environment based on which the planning algorithm will be able to plan the path for the robot.

For creating a map, we use the `slam_toolbox <https://github.com/SteveMacenski/slam_toolbox>`_. 

Creating map
************

.. note:: Depending on the requirement, the mapping parameters can be changed under ``~/your_workspace/src/neo_mpo_700/config/navigation/mapping.yaml``
1. Run the mapping bringup node, which enable only the mobile base, without navigation:
::
	ros2 launch awcombo_moveit_config map_bringup.launch.py

2. Start creating the map, use the following command
::
	ros2 launch neo_mpo_700-2 mapping.launch.py

.. warning:: Do not close the launch until saving the map.

3. After starting the mapping, in the RViz, the initial stripes of the maps can be found

4. Navigate the robot around the environment, in order to further build the map.

Saving map
**********

To save the map, use the following command

::
	ros2 run nav2_map_server map_saver_cli -f ~/your_workspace/src/neo_base/neo_mpo_700-2/configs/navigation/maps/

In the map folder, two essential files are created
	1. your_map_name.pgm
	2. your_map_name.yaml

Once the files are created, it's always essential to rebuild the package, in order for the maps to be installed and available during the run time. If not built, ROS will not be able to find the newly created map.


Docking
-------

More information on the docking can be found `here`.

Change parameters and algorithms
--------------------------------

There are different algorithms and corresponding parameters as part of the Navigation2 stack. These parameters can be found under ``neo_mpo_700/config/navigation/navigation.yaml`` 

All the home-brewed algorithms are explained in the section packages. More information about the options specified in the above table can be found in this `navigation plugins page <https://navigation.ros.org/plugins/>`_.

Change Behaviors
------------------

Behavior tree is one of the key core concepts that have been adapted into the Navigation 2 stack. Behavior trees in the Nav2 stack provides the user to easily develop and customize various behaviors that the robot should exhibit. Each behaviors are developed as an individual component and then is integrated into the behavior tree. Thus the behavior tree, provides us with a flexible development environment for solving a particular problem at hand. 

By default, all the robot packages carry behavior trees, that is customizable according to the application needs. The behavior trees can be found under ``neo_mpo_700/config/navigation/behavior_trees``.

``navigating_to_pose`` behavior tree is for all the tasks that requires the robot to navigate in free space from point A to point B. Whereas ``navigate_through_pose`` behavior tree is for the tasks that requires to robot to navigate through set of intermediary hard pose constraints in order to reach the goal localtion 

.. note:: More information on behavior trees can be `found here <https://navigation.ros.org/concepts/index.html#behavior-trees>`_. `Examples of behavior_trees can be found here <https://navigation.ros.org/behavior_trees/index.html>`_. 
