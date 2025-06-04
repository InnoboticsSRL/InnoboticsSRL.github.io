# ibt_ros2_driver
For controlling the movement of the robotic arm, a ROS2 package has been developed that interfaces with the Motorcortex web server. Within this package, an action server has been primarily implemented, whose task is to receive and manage movement requests from a user, executing them in a safe and reliable manner

## General Objectives of the Project

This section describes the main goals of the project at a conceptual level, without going into specific software implementation details.

The objective of the project is to develop a controller capable of receiving commands from a user to move a robotic arm, while ensuring the internal consistency of the system’s states. The controller provides the following functionalities:

1) **Joint coupling and error recovery**  
This functionality allows the user to couple the joints of the robot, an operation required to enable movement commands. It also makes it possible to bring the system back to an operational state in the event of execution errors.

2) **Canceling an ongoing operation**  
For safety or operational reasons, the user must be able to interrupt a movement currently in progress. The controller correctly handles the cancellation of the request.

3) **Publishing the robot’s status**  
The system provides in real time:  
- The current positions of the robotic arm joints  
- The current status of the robot, according to its internal state machine  

4) **Movement execution**  
The controller supports various types of movement:  
- **Joint Movement**: Movement is executed by specifying a list of coordinates in joint space.  
- **Point-To-Point (P2P) Movement**: The tool at the end of the arm reaches a specific position in Cartesian space.  
- **Circular Movement**: The arm’s tool executes a circular movement in space.  
- **Linear Movement**: The tool moves along a linear trajectory between two points in space.  

5) **Changing the dout state state**

6) **Publishing joint space positions and internal state of the arm**

---

## Configurable Variables at Node Startup

This section describes the variables that the user can modify before starting the node.  
The variables are defined in the launch file `ibt_ros2_driver.launch.py`.

| Variable       | Description                                           |
|----------------|-------------------------------------------------------|
| `namespace`    | Namespace to assign to the node                      |
| `url`          | WebSocket URL for the connection                     |
| `prefix`       | Prefix to use for the node                           |
| `timeout_ms`   | Connection timeout (in milliseconds)                 |
| `login`        | Username for WebSocket authentication                |
| `password`     | Password for WebSocket authentication                |

These variables can be passed as arguments when launching the node using the `ros2 launch` command, for example:

```bash
ros2 launch ibt_ros2_driver ibt_ros2_driver.launch.py url:=ws://localhost:8080
```

## How to Use the Package

This package provides interfaces to control a robotic arm. The main functionalities include:
```bash
/robofox/ibt_ros2_driver
  Subscribers:

  Publishers:
    /parameter_events: rcl_interfaces/msg/ParameterEvent
    /robofox/eef_pose: ibt_ros2_interfaces/msg/PoseRPY
    /robofox/joint_states: sensor_msgs/msg/JointState
    /robofox/logic_state: std_msgs/msg/String
    /rosout: rcl_interfaces/msg/Log
  Service Servers:
    /robofox/disarm: std_srvs/srv/Trigger
    /robofox/ibt_ros2_driver/describe_parameters: rcl_interfaces/srv/DescribeParameters
    /robofox/ibt_ros2_driver/get_parameter_types: rcl_interfaces/srv/GetParameterTypes
    /robofox/ibt_ros2_driver/get_parameters: rcl_interfaces/srv/GetParameters
    /robofox/ibt_ros2_driver/list_parameters: rcl_interfaces/srv/ListParameters
    /robofox/ibt_ros2_driver/set_parameters: rcl_interfaces/srv/SetParameters
    /robofox/ibt_ros2_driver/set_parameters_atomically: rcl_interfaces/srv/SetParametersAtomically
    /robofox/rearm: std_srvs/srv/Trigger
    /robofox/set_output: ibt_ros2_interfaces/srv/SetOutput
  Service Clients:

  Action Servers:
    /robofox/move_arm: ibt_ros2_interfaces/action/MoveArm
  Action Clients:

```

## Moving the Arm

To send commands to the move_arm action server, it is recommended to write a ROS 2 client. Below is a practical example.

```python
TODO
```