#!/usr/bin/env python3
import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.node import Node
from ro36_interfaces.action import Goto
from roboter_interfaces.msg import RoboterInfo
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import time 
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
import threading
from move_node.move import SimpleRobotMover
import numpy as np
from std_msgs.msg import Bool

class Action_Server(Node):
    def __init__(self):
        """
        Initializes the Action_Server node.
        """
        super().__init__("ro36_simple_mover_server")
        
        self._goal_handle = None
        self._last_pose_x = None
        self._last_pose_y = None
        self._last_pose_theta = None
        self._roboter_current_pose_x = None
        self._roboter_current_pose_y = None
        self._roboter_current_pose_theta = None
        self.current_robo_x = None
        self.current_robo_y = None
        self.current_robo_theta = None
        
        # Create an ActionServer for the Goto action
        self.action_server = ActionServer(
            self, 
            Goto,
            "action_server", 
            execute_callback=self._execute_callback, 
            handle_accepted_callback=self._handle_accepted_callback,
            cancel_callback=self._cancel_callback,
            goal_callback=self._goal_callback,
            callback_group=ReentrantCallbackGroup()
        )
        self._goal_handle_lock = threading.Lock()
        
        # Create a subscription to the Odometry topic
        self.odom_sub = self.create_subscription(Odometry, 'odom', self._odom_callback, 10)
        self.sub_roboterInfo = self.create_subscription(RoboterInfo, 'roboter_info', self.roboter_callback, 10)
        # Create a publisher for the Twist command
        self._cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self._result_pub = self.create_publisher(Bool, 'result_move', 10)
        
    def roboter_callback(self, msg):
        self.current_robo_x = msg.roboter_x
        self.current_robo_y = msg.roboter_y
        self.current_robo_theta = msg.roboter_theta
        
    def _goal_callback(self, goal_request):
        """
        Handles the goal callback when a new goal is received.

        Args:
            goal_request: The goal request.

        Returns:
            GoalResponse: ACCEPT to accept the goal.
        """
        self.get_logger().info("Received goal request with target pose " + _pose_as_string(goal_request.pose)) 
        self._result_pub.publish(Bool(data = True))
        return GoalResponse.ACCEPT

    def _determine_action_result(self, goal_handle):
        """
        Determines the action result based on the current state.

        Args:
            goal_handle: The goal handle.

        Returns:
            Goto.Result: The action result.
        """
        result = Goto.Result()
        if goal_handle.is_active and self._dist_to_goal(goal_handle) <= 0.1:
            self.get_logger().info("Move to pose " + _pose_as_string(goal_handle.request.pose) + " succeeded")
            goal_handle.succeed()
            result.reached = True
            self._result_pub.publish(Bool(data = False))
            
        elif goal_handle.is_cancel_requested:
            goal_handle.canceled()
            self.get_logger().info("Move to pose " + _pose_as_string(goal_handle.request.pose) + " was cancelled")
            self.result_move = False
        else:
            if goal_handle.is_active:
                goal_handle.abort() 
                self.result_move = False
            self.get_logger().info("Move to pose " + _pose_as_string(goal_handle.request.pose) + " was aborted")
            self.result_move = False
            
        return result
    
    def _publish_feedback(self, goal_handle):
        """
        Publishes feedback based on the current state.

        Args:
            goal_handle: The goal handle.
        """
        feedback_msg = Goto.Feedback()
        feedback_msg.dist_to_goal = self._dist_to_goal(goal_handle)
        goal_handle.publish_feedback(feedback_msg)
        time.sleep(1)

    def _dist_to_goal(self, goal_handle):
        """
        Calculates the distance to the goal.

        Args:
            goal_handle: The goal handle.

        Returns:
            float: The distance to the goal.
        """
        return np.sqrt(
            np.square(goal_handle.request.pose.x - self._last_pose_x) +
            np.square(goal_handle.request.pose.y - self._last_pose_y)
        )
        
    def _handle_accepted_callback(self, goal_handle):
        """
        Handles the accepted callback when a new goal is accepted.

        Args:
            goal_handle: The goal handle.
        """
        with self._goal_handle_lock: 
            if self._goal_handle is not None and self._goal_handle.is_active:
                self.get_logger().info("Replacing active goal with new goal. ")
                self._goal_handle.abort()
            self._goal_handle = goal_handle
        goal_handle.execute()
    
    def _cancel_callback(self, goal_handle):
        """
        Handles the cancel callback when a goal is cancelled.

        Args:
            goal_handle: The goal handle.

        Returns:
            CancelResponse: ACCEPT to accept the cancellation.
        """
        self.get_logger().info("Canceling move to pose " + _pose_as_string(goal_handle.request.pose)) 
        return CancelResponse.ACCEPT
    
    def _execute_callback(self, goal_handle):
        """
        Executes the action to move to the specified pose.

        Args:
            goal_handle: The goal handle.

        Returns:
            Goto.Result: The action result.
        """
        self.get_logger().info("Executing move to pose " + _pose_as_string(goal_handle.request.pose)) 
        mover = SimpleRobotMover()
        mover.set_target_pose(goal_handle.request.pose.x, goal_handle.request.pose.y, goal_handle.request.pose.theta)
        
        vel = mover.get_velocity_command(self._last_pose_x, self._last_pose_y, self._last_pose_theta)

        while vel is not None:
            vel = mover.get_velocity_command(self._last_pose_x, self._last_pose_y, self._last_pose_theta)
            self._publish_vel(vel)
            self._publish_feedback(goal_handle)
            time.sleep(0.05) 

        self._publish_vel(None)
        return self._determine_action_result(goal_handle)
    
    def _publish_vel(self, vel):
        """
        Publishes the velocity command.

        Args:
            vel: The velocity command.
        """
        vel_msg = Twist()
        if vel is not None:
            vel_msg.angular.z = vel[1]
            vel_msg.linear.x = vel[0]
        self._cmd_pub.publish(vel_msg)
              
    def _odom_callback(self, msg):
        """
        Callback for the Odometry message. Updates the robot's current pose.

        Args:
            msg: The Odometry message.
        """
        self._last_pose_x = msg.pose.pose.position.x
        self._last_pose_y = msg.pose.pose.position.y
        _, _, self._last_pose_theta = quaternion([
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w,   
        ])
        
def _pose_as_string(pose):
    """
    Converts a pose to a string.

    Args:
        pose: The pose.

    Returns:
        str: The pose as a string.
    """
    return "x ={}, y={} , theta={}".format(pose.x, pose.y, pose.theta) 

def quaternion(quaternion):
    """
    Converts quaternion (w in last place) to euler roll, pitch, yaw.

    Args:
        quaternion (list): The quaternion [x, y, z, w].

    Returns:
        tuple: Euler angles (roll, pitch, yaw).
    """
    x, y, z, w = quaternion

    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = np.arctan2(sinr_cosp, cosr_cosp)

    sinp = 2 * (w * y - z * x)
    pitch = np.arcsin(sinp)

    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = np.arctan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw  

def main(args=None):
    """
    Main entry point of the script.

    Args:
        args: Command line arguments.
    """
    rclpy.init(args=args)
    robotMoverExecutor = MultiThreadedExecutor(num_threads=2)
    action_server = Action_Server()
    rclpy.spin(node=action_server, executor=robotMoverExecutor)  
    action_server.destroy_node()
    rclpy.shutdown()      
        
if __name__ == '__main__':
    main()
