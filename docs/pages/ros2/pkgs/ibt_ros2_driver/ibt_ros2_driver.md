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
ros2 launch ibt_ros2_driver ibt_ros2_driver.launch.py url:=ws://localhost:8080 login:=user password:=pass
```

---

## ibt_ros2_interfaces

The necessary actions, messages, and services for the driver's operation are defined here.

### Messages

- **MoveReq.msg**: This message is used to construct the type of instruction to send to the server.

```
int32 move_type  # possible move to be executed
# enums 
int32 PTP=0     
int32 LIN=1
int32 JOINT=2
int32 CIRC=3
builtin_interfaces/Duration duration

PoseRPY[] pose         # list of Eulerian poses
float64[] positions    # list of joint positions

float64 scale_vel_factor
float64 scale_acc_factor
float64 angle_radius 
```

When constructing the message, the fields `pose` and `positions` must not be populated simultaneously, but only based on the selected move type.

**Pose** is used for the following movement types:
- LIN
- PTP
- CIRC: When this movement is selected, Motorcortex requires at least 3 non-aligned points to execute the correct circular motion of the arm. In this case, the field **angle_radius** is also filled with the angle (in radians) that the arm should travel.

**Positions** is used for JOINT

**PoseRPY.msg**: This message is used to define the Eulerian pose to be included in the `MoveReq` message.

```
# Eulerian pose
float64 x 
float64 y
float64 z 

float64 roll
float64 pitch
float64 yaw
```

### Actions

The action is defined as follows:

```
# Request
std_msgs/Header header  
MoveReq[] instructions  # list of instructions
---
# Result
int32 error_code        # error code
int32 SUCCESSFUL = 0
int32 INVALID_GOAL = -1

string error_str        # error message
---
# Feedback
string status           # status of the move
```

With this action, the user can send a request to execute a list of sequential movements by building a list of `MoveReq`, essentially creating a program for the robot to follow.

During motion execution, the client receives feedback corresponding to the status of the program's execution.

When the server completes the operation, the client receives a result code indicating the overall outcome of the request.

### Services

The services used are:

- **rearm**:  
  The rearm service is used to bring the robot out of various possible error states and re-enable the joints.  
  The required service type is already defined in the ROS ecosystem and is called [TRIGGER](https://docs.ros2.org/foxy/api/std_srvs/srv/Trigger.html).  
  This type of interface is used exclusively for activating the rearm functionality.

- **set_output**:

```
uint8 value
---
bool success
string message
```

This service allows control over the dout state states, and the enum defines the possible states to assign to the `value` field.

At the end of the operation, the server sends back to the client a result consisting of a boolean indicating the success of the request and a string message with a detailed error, if any occurred.


## How to Use the Package

This package provides interfaces to control a robotic arm. The main functionalities include:

* **Action Server**:

  * `move_arm`: sends motion commands to the robotic arm and cancels them.

* **Services**:

  * `rearm`: corrects the internal state of the arm.
  * `set_output`: manages the dout state.

* **Publishers**:

  * `joint_states`: publishes the position, velocity, and acceleration of the joints.
  * `logic_state`: publishes the internal logical state of the arm based on the state machine provided by Motorcortex.

---

## Visualizing Topics

Once the node is running, you can view the data published on `joint_states` and `logic_state` using standard ROS 2 commands:

```bash
ros2 topic echo <topic_name>
```

Esempi:

```bash
ros2 topic echo /robofox/joint_states
ros2 topic echo /robofox/logic_state
```
---

## Moving the Arm

To send commands to the move_arm action server, it is recommended to write a ROS 2 client. Below is a practical example.

### Esempio di client per il movimento del braccio

```python
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
import math
import numpy as np

from ibt_ros2_interfaces.action import MoveArm
from ibt_ros2_interfaces.msg import MoveReq, PoseRPY

class MoveArmClient(Node):

    def __init__(self):
        super().__init__('move_arm_client')
        self._client = ActionClient(self, MoveArm, '/robofox/move_arm')
        self.timeout_sec = 70.0

    def create_pose_msg(self, pose_vector) -> PoseRPY:
        pose = PoseRPY()
        pose.x, pose.y, pose.z, pose.roll, pose.pitch, pose.yaw = pose_vector
        return pose

    def send_goal(self, move_type='LIN'):
        self.get_logger().info(f'Waiting for server...')
        if not self._client.wait_for_server(timeout_sec=self.timeout_sec):
            self.get_logger().error('Action server not available.')
            return

        goal_msg = MoveArm.Goal()

        # Pose e joint di esempio
        pose = np.array([0.4, 0.2, 0.6, 0.0, math.pi, 0.0])
        pose1 = np.array([0.450, 0.0, 0.450, math.radians(180), 0.0, math.radians(180)])
        j_pos1 = [math.radians(0.5), math.radians(-15.9), math.radians(117.5), math.radians(0), math.radians(74.4), math.radians(0.5)]

        # Comando di homing
        req_homing = MoveReq()
        req_homing.move_type = MoveReq.JOINT
        req_homing.positions = [math.radians(0), math.radians(0.0), math.radians(90.0), math.radians(0), math.radians(90.0), math.radians(0)]
        req_homing.scale_vel_factor = 0.1
        req_homing.scale_acc_factor = 0.1

        if move_type == 'LIN':
            req = MoveReq()
            req.move_type = MoveReq.LIN
            req.pose = [self.create_pose_msg(pose.tolist())]
            req.scale_vel_factor = 0.1
            req.scale_acc_factor = 0.1
            goal_msg.instructions = [req_homing, req]

        elif move_type == 'PTP':
            req = MoveReq()
            req.move_type = MoveReq.PTP
            req.pose = [self.create_pose_msg(pose.tolist())]
            req.scale_vel_factor = 0.5
            req.scale_acc_factor = 0.5
            goal_msg.instructions = [req_homing, req]

        elif move_type == 'CIRC':
            cart_pos_1 = [0.675, 0.0, 0.4739, math.pi, 0.0, math.pi]
            cart_pos_2 = [0.850, 0.0, 0.4739, math.pi, 0.0, math.pi]
            cart_pos_3 = [0.675, 0.2, 0.4739, math.pi, 0.0, math.pi]

            req = MoveReq()
            req.move_type = MoveReq.CIRC
            req.angle_radius = 3 * math.pi
            req.pose = [
                self.create_pose_msg(cart_pos_1),
                self.create_pose_msg(cart_pos_2),
                self.create_pose_msg(cart_pos_3)
            ]
            req.scale_vel_factor = 0.1
            req.scale_acc_factor = 0.1
            goal_msg.instructions = [req]

        elif move_type == 'JOINT':
            req = MoveReq()
            req.move_type = MoveReq.JOINT
            req.positions = j_pos1
            req.scale_vel_factor = 0.1
            req.scale_acc_factor = 0.1
            goal_msg.instructions = [req_homing]

        else:
            self.get_logger().error(f'Unknown move_type: {move_type}')
            return

        self.get_logger().info(f'Sending goal: {move_type}')
        future = self._client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        rclpy.spin_until_future_complete(self, future, timeout_sec=self.timeout_sec)
        goal_handle = future.result()

        if not goal_handle or not goal_handle.accepted:
            self.get_logger().error('Goal was rejected or failed to send.')
            return

        self.get_logger().info('Goal accepted, waiting for result...')
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=self.timeout_sec)
        result = result_future.result().result

        self.get_logger().info(f'Result: {result.error_str} (Code: {result.error_code})')

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(f'Feedback received: {feedback}')

def main(args=None):
    rclpy.init(args=args)
    node = MoveArmClient()

    node.send_goal('PTP')
    node.send_goal('LIN')
    node.send_goal('CIRC')
    node.send_goal('JOINT')

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

---

**Explanation:**
The action server expects a message of type MoveArm.Goal, which contains a list of instructions of type MoveReq.
Each MoveReq defines:

 - the type of movement (JOINT, LIN, PTP, CIRC);

 - the targets (joint positions or Cartesian poses PoseRPY);

 - velocity and acceleration scaling factors (and the angle for circular motion).

**More details:**

The function `create_pose_msg()` is a utility to convert a list of 6 values into a properly structured `PoseRPY` message.


## Cancelling a Request: 

This section shows how to cancel a motion request. The example is taken from a pytest file.

```python

    def test_cancel_goal(rclpy_init_shutdown, node, action_client):
    assert action_client.wait_for_server(timeout_sec=timeout_sec)

    goal = MoveArm.Goal()
    req = MoveReq()
    req.move_type = MoveReq.JOINT
    req.scale_vel_factor = 1.0
    req.positions = j_pos
    goal.instructions = [req_zero, req_homing, req]

    future = action_client.send_goal_async(goal, feedback_callback=feedback_callback)
    rclpy.spin_until_future_complete(node, future, timeout_sec=timeout_sec)
    goal_handle = future.result()
     
    assert goal_handle is not None
    assert goal_handle.accepted


    cancel_future = goal_handle.cancel_goal_async()
    rclpy.spin_until_future_complete(node, cancel_future, timeout_sec=timeout_sec)
    assert goal_handle.accepted
    cancel_response = cancel_future.result()
    node.get_logger().info("cancel goal is sent")
    assert cancel_response.return_code == CancelGoal.Response.ERROR_NONE
    assert len(cancel_response.goals_canceling) == 1
    assert cancel_response.goals_canceling[0].goal_id == goal_handle.goal_id

```

## Sending Service Requests: 

This section shows how to send service requests.

**rearm :**
```python

   def test_rearm_service(rclpy_init_shutdown, node, service_client):
    assert service_client.wait_for_service(timeout_sec=10.0)
    
    request = Trigger.Request()
    future = service_client.call_async(request)
    
    rclpy.spin_until_future_complete(node, future, timeout_sec=10.0)
    
    assert future.result() is not None
    assert future.result().success
    assert future.result().message == "Rearmed successfully"

```