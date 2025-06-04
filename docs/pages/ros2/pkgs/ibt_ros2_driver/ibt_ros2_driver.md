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