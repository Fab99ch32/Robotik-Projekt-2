import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from geometry_msgs.msg import Pose2D
from ro36_interfaces.action import Goto
from nav_msgs.msg import Odometry
import numpy as np
import math

class Action_Client(Node):
    def __init__(self):
        """
        Initializes the Action Client node.
        """
        super().__init__("action_client")
        # Creates an Action Client for the Goto Action Server
        self._action_client = ActionClient(self, Goto, "action_server")
        # Creates a subscription for Odometry messages
        self._odometry_subscription = self.create_subscription(Odometry, 'odom', self.odometry_callback, 10)
        # Current position of the robot
        self._current_x = 0.0
        self._current_y = 0.0

    def odometry_callback(self, msg):
        """
        Callback function for Odometry messages. Updates the current position of the robot.
        """
        self._current_x = msg.pose.pose.position.x
        self._current_y = msg.pose.pose.position.y
        
        
    def send_goal(self):
        """
        Sends a goal to the Goto Action Server.
        """
        # Creates a goal for the Goto Action
        goal_msg = Goto.Goal()
        goal_msg.pose = Pose2D(x=self._current_x, y=self._current_y, theta=get_quaternion_from_euler(0.0, 0.0, np.pi)[3])  

        # Waits for the Action Server to become available
        self._action_client.wait_for_server()

        # Sends the goal asynchronously
        return self._action_client.send_goal_async(goal_msg)


def get_quaternion_from_euler(roll, pitch, yaw):
  """
  Convert an Euler angle to a quaternion.
   
  Input
    :param roll: The roll (rotation around x-axis) angle in radians.
    :param pitch: The pitch (rotation around y-axis) angle in radians.
    :param yaw: The yaw (rotation around z-axis) angle in radians.
 
  Output
    :return qx, qy, qz, qw: The orientation in quaternion [x,y,z,w] format
  """
  qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
  qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
  qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
  qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
 
  return [qx, qy, qz, qw]

def main(args=None):
    """
    Main function to initialize and run the Action Client.
    """
    rclpy.init(args=args)

    action_client = Action_Client()

    rclpy.spin_once(action_client)
   
    # Sends a goal (90 degrees rotation)
    future = action_client.send_goal()

    # Waits until the action is completed
    rclpy.spin_until_future_complete(action_client, future)

    rclpy.shutdown()

if __name__ == '__main__':
    main()
