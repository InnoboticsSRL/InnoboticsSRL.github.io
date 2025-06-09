.. _manipulator:

Manipulator
============

Arm
-------

Topics
    - /robofox/joint_states: sensor_msgs/msg/JointState
    - /robofox/logic_state: std_msgs/msg/String
Services
    - /robofox/disarm: std_srvs/srv/Trigger
    - /robofox/rearm: std_srvs/srv/Trigger
    - /robofox/set_output: ibt_ros2_interfaces/srv/SetOutput
    - /robofox/get_input: ibt_ros2_interfaces/srv/GetInput
Actions
    - /robofox/move_arm: ibt_ros2_interfaces/action/MoveArm

The arm is the main component of the manipulator, consisting of multiple links and joints that allow it to move in various directions.
The controller supports various types of movement:  
    - **Joint Movement**: Movement is executed by specifying a list of coordinates in joint space.  
    - **Point-To-Point (P2P) Movement**: The tool at the end of the arm reaches a specific position in Cartesian space.  
    - **Linear Movement**: The tool moves along a linear trajectory between two points in space.  

Example:

.. code-block:: python

    import rclpy
    from rclpy.node import Node
    from rclpy.action import ActionClient
    import math
    import time
    from std_srvs.srv import Trigger

    from ibt_ros2_interfaces.action import MoveArm
    from ibt_ros2_interfaces.msg import MoveReq, PoseRPY, Waypoint

    RED = '\033[91m'
    GREEN = '\033[92m'
    BLUE = '\033[94m'
    YELLOW = '\033[93m'
    RESET = '\033[0m'

    class TESTMove(Node):
        def __init__(self):
            super().__init__('test_move_arm')
            self.get_logger().info(f'{YELLOW}Starting TESTMove...{RESET}')

            self.rearm_client = self.create_client(Trigger, '/robofox/rearm')
            self.disarm_client = self.create_client(Trigger, '/robofox/disarm')
            self.move_client = ActionClient(self, MoveArm, '/robofox/move_arm')
            self.timeout_sec = 60.0

            self.wp_home = Waypoint()
            self.wp_home.pose = [math.radians(0), math.radians(0.0), math.radians(90.0), math.radians(0), math.radians(90.0), math.radians(0)]
            self.wp_home.smoothing_factor = 0.1
            self.wp_home.next_segment_velocity_factor = 0.1

            self.req_home = MoveReq()
            self.req_home.move_type = MoveReq.JOINT
            self.req_home.velocity = 0.5
            self.req_home.acceleration = 0.3
            self.req_home.rotational_velocity = 0.5
            self.req_home.rotational_acceleration = 0.3
            self.req_home.waypoints = [self.wp_home]

            self.wp_default = Waypoint()
            # in joint
            self.wp_default.pose = [-1.361, -0.578, 1.607, 0.038, 0.384, 0.000]
            self.wp_default.smoothing_factor = 0.1
            self.wp_default.next_segment_velocity_factor = 0.1

            self.req_default = MoveReq()
            self.req_default.move_type = MoveReq.JOINT
            self.req_home.velocity = 0.5
            self.req_home.acceleration = 0.3
            self.req_home.rotational_velocity = 0.5
            self.req_home.rotational_acceleration = 0.3
            self.req_default.waypoints = [self.wp_default]

            # Detection poses in xyzRPY format wrt base_link
            self.detection_poses = [ [0.60,0.0,0.0,math.radians(180),0.0,math.radians(180)] ]

        def call_rearm_service(self):
            if not self.rearm_client.wait_for_service(timeout_sec=2.0):
                self.get_logger().error(f'{RED}Rearm service not available.{RESET}')
                return False

            request = Trigger.Request()
            future = self.rearm_client.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=self.timeout_sec)
            res = future.result()

            if res is not None and res.success:
                self.get_logger().info(f'{GREEN}Rearm service called successfully.{RESET}')
                # Timeout of 5 seconds after rearm to ensure the arm is ready
                time.sleep(5.0)
                return True
            else:
                self.get_logger().error(f'{RED}Failed to call rearm service.{RESET}')
                return False

        def call_disarm_service(self):
            if not self.disarm_client.wait_for_service(timeout_sec=2.0):
                self.get_logger().error(f'{RED}Disarm service not available.{RESET}')
                return False

            request = Trigger.Request()
            future = self.disarm_client.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=self.timeout_sec)
            res = future.result()

            if res is not None and res.success:
                self.get_logger().info(f'{GREEN}Disarm service called successfully.{RESET}')
                # Timeout of 1 second after disarm to ensure the arm is not engaged
                time.sleep(1.0)
                return True
            else:
                self.get_logger().error(f'{RED}Failed to call disarm service.{RESET}')
                return False

        def create_pose_msg(self, pose_vector) -> PoseRPY:
            pose = PoseRPY()
            pose.x, pose.y, pose.z, pose.roll, pose.pitch, pose.yaw = pose_vector
            return pose

        def feedback_callback(self, feedback_msg):
            self.get_logger().info(f'{YELLOW} Feedback: {feedback_msg.feedback.status}{RESET}')

        def move_to_req_pose(self, req: MoveReq):
            self.get_logger().info(f'{BLUE} [Trying to move to home pose...{RESET}')
            goal_msg = MoveArm.Goal()
            goal_msg.requests = [req]

            # First way to send MoveArm goal
            self.get_logger().info(f'{BLUE}Sending goal: {req.move_type}{RESET}')
            future = self.move_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
            rclpy.spin_until_future_complete(self, future, timeout_sec=self.timeout_sec)
            goal_handle = future.result()

            if not goal_handle or not goal_handle.accepted:
                self.get_logger().error(f'{RED}Goal was rejected or failed to send.{RESET}')
                return False

            self.get_logger().info(f'{BLUE}Goal accepted, waiting for result...{RESET}')
            result_future = goal_handle.get_result_async()
            rclpy.spin_until_future_complete(self, result_future, timeout_sec=self.timeout_sec)
            result = result_future.result().result

            self.get_logger().info(f'{YELLOW}Move result: {result.error_str} (Code: {result.error_code}){RESET}')
            return True

        def move_to_pose_rpy(self, rpy: PoseRPY):
            goal = MoveArm.Goal()

            wp = Waypoint()
            wp.pose = [rpy.x, rpy.y, rpy.z, rpy.roll, rpy.pitch, rpy.yaw]
            wp.smoothing_factor = 0.1
            wp.next_segment_velocity_factor = 0.1

            req = MoveReq()
            req.move_type = MoveReq.PTP
            req.velocity = 0.5
            req.acceleration = 0.3
            req.rotational_velocity = 0.5
            req.rotational_acceleration = 0.3
            req.waypoints = [wp]
            goal.requests = [req]

            self.get_logger().info(f'{BLUE}Sending goal: {req.move_type}{RESET}')
            future = self.move_client.send_goal_async(goal, feedback_callback=self.feedback_callback)
            rclpy.spin_until_future_complete(self, future, timeout_sec=self.timeout_sec)
            goal_handle = future.result()

            if not goal_handle or not goal_handle.accepted:
                self.get_logger().error(f'{RED}Goal was rejected or failed to send.{RESET}')
                return False

            self.get_logger().info(f'{BLUE}Goal accepted, waiting for result...{RESET}')
            result_future = goal_handle.get_result_async()
            rclpy.spin_until_future_complete(self, result_future, timeout_sec=self.timeout_sec)
            result = result_future.result().result

            self.get_logger().info(f'{YELLOW}Move result: {result.error_str} (Code: {result.error_code}){RESET}')
            return True

        def execute_pipeline(self):
            if not self.call_rearm_service():
                self.get_logger().error(f'{RED}Rearm service call failed. Exiting...{RESET}')
                return

            self.get_logger().info(f'{YELLOW} Waiting for action servers...{RESET}')
            self.move_client.wait_for_server()
            self.get_logger().info(f'{GREEN} Move arm server active...{RESET}')

            if not self.move_to_req_pose(self.req_default):
                self.get_logger().error(f'{RED}Pipeline aborted: failed to reach default pose.{RESET}')
                return

            if not self.move_to_req_pose(self.req_home):
                self.get_logger().error(f'{RED}Pipeline aborted: failed to reach home pose.{RESET}')
                return

                
            for i, pose_vec in enumerate(self.detection_poses):
                self.get_logger().info(f'{YELLOW} [Try {i+1}] Moving to detection pose...{RESET}')
                
                if not self.move_to_pose_rpy(self.create_pose_msg(pose_vec)):
                    self.get_logger().error(f'{RED}Failed to move to detection pose {i+1}. Exiting...{RESET}')
                    return


            if not self.call_disarm_service():
                self.get_logger().error(f'{RED}Disarm service call failed. Exiting...{RESET}')
                return


End effector
-------------
The end effector is the part of the manipulator that interacts with the environment. It can be a gripper, a tool, or any other device that performs a specific task.
It can be controlled using the services `/robofox/set_output` and `/robofox/get_input` provided by the `ibt_ros2_interfaces` package.

Minimal example

.. code-block:: python

    def test_set_output_service(rclpy_init_shutdown, node, set_output_client):
        assert set_output_client.wait_for_service(timeout_sec=10.0)

        request = SetOutput.Request()
        request.data = [False]
        future = set_output_client.call_async(request)
        
        rclpy.spin_until_future_complete(node, future, timeout_sec=10.0)
        
        assert future.result() is not None
        assert future.result().success

    def test_get_input_service(rclpy_init_shutdown, node, get_input_client):
        assert get_input_client.wait_for_service(timeout_sec=10.0)

        request = GetInput.Request()
        future = get_input_client.call_async(request)
        
        rclpy.spin_until_future_complete(node, future, timeout_sec=10.0)
        
        assert future.result() is not None
        assert future.result().success
        assert len(future.result().data) > 0

