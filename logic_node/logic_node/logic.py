import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose2D, Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from roboter_interfaces.msg import MakerDistanceID
from ro36_interfaces.action import Goto
from rclpy.action import ActionClient
from math import pi 
import numpy as np
import time


class LogicNode(Node):

    def __init__(self):
        super().__init__('logic_node')
    
        self._action_client = ActionClient(self, Goto, 'action_server')
        self.sub_markerInfo = self.create_subscription(MakerDistanceID, 'aruco_marker_info', self.aruco_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, 'odom',self._odom_callback, 10)
        self.sub_rfidInfo = self.create_subscription(String, 'rfid_tag', self.rfid_callback, 10)
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
          
        self.id_front = 1
        self.id_back = 23
        self.id_goal = 37
        self.last_marker_id = None 
        self.marker_id =None
        self.marker_distance = None
        self.marker_x = None
        self.marker_y = None
        self.marker_theta = None
        self.rfid = None
        self.last_rfid = None
        self.last_pose_x = None
        self.last_pose_y = None
        self.last_pose_theta = None
        self.is_here = False
        self.start_x =  None
        self.start_y = None
        self.last_call_time = 0
        self.rotate_time = 10
        self.linear_x = 0.1
        self.stop_distance = 0.35
        self.start_time=None
        self.time_required = 0.0
        self.is_rotate= False
        self.pose_reached = False
        
    def aruco_callback(self, msg):
        self.marker_id = msg.id.data
        self.marker_distance = msg.distance
        self.get_logger().info("Erhaltene Aruco-Markerinformationen: ID={}, Entfernung={}".format(
            self.marker_id, self.marker_distance))
        
        self.handle_function()
           
               
    def handle_function(self):
        if self.marker_id == self.id_back:
            self.handle_back_marker()
        elif self.marker_id == self.id_front:
            self.handle_front_marker()
        elif self.marker_id == self.id_goal:
            self.handle_goal_marker()
        else:
            self.get_logger().warning("Unrecognized marker ID: {}".format(self.marker_id))
        
    def handle_back_marker(self):
        self.get_logger().info("Roboter wurde von hinten in einer Abstand von {:.2f} m erkannt".format(self.marker_distance))
        self.execute_avoidance()
        self.last_marker_id = self.marker_id
        
    def handle_front_marker(self):
        self.get_logger().info("Roboter wurde von vorne in einer Abstand von {:.2f} m erkannt".format(self.marker_distance))
        self.execute_avoidance()
        self.last_marker_id = self.marker_id
        
    def handle_goal_marker(self):
        self.get_logger().info("Ziel Position in einer Abstand von {:.2f} m erkannt".format(self.marker_distance))
        
        distance = self.marker_distance 
        if distance is not None and distance > self.stop_distance:
            if self.is_rotate is False:
                self.pose_reached  = False
                self.get_logger().info("Roboter läuft !")
        else:
            self.pose_reached  = True
            self.get_logger().info("Ziel erreicht!")

            # Perform rotation
            rotate_theta = get_quaternion_from_euler(0.0, 0.0, (np.pi+self.last_pose_theta))[3]
            self.publish_goal_info(x=self.last_pose_x, y=self.last_pose_y, theta=rotate_theta)
            self.get_logger().info("Wenden läuft!")
            self.is_rotate = True
            
    
    def start_pose_robot(self, x, y):
        if self.is_here is False:
            self.is_here= True
            self.start_x = x
            self.start_y = y
            
    def _odom_callback(self,msg):
        self.last_pose_x = msg.pose.pose.position.x
        self.last_pose_y = msg.pose.pose.position.y
        self.start_pose_robot(self.last_pose_x, self.last_pose_y)
        _,_,self.last_pose_theta = quaternion(
            [
                msg.pose.pose.orientation.x,
                msg.pose.pose.orientation.y,
                msg.pose.pose.orientation.z,
                msg.pose.pose.orientation.w,   
            ]
        )

    def publish_goal_info(self, x, y, theta):
        goal_msg = Goto.Goal()
        goal_msg.pose = Pose2D(x=x , y=y , theta=theta)
        self.is_rotate = False
        return self._action_client.send_goal_async(goal_msg)

    def execute_avoidance(self):
        avoidance_theta = get_quaternion_from_euler(0.0, 0.0, (np.pi/4))[3] # 90 Grad nach rechts drehen
        self.publish_goal_info(x=self.last_pose_x, y=self.last_pose_y, theta=avoidance_theta)
        self.get_logger().info("Ausweichbewegung nach rechts gestartet: x={}, y={}, Theta={}".format(self.last_pose_x, self.last_pose_y, avoidance_theta))
        
    def are_variables_set(self):
        return all(
            var is not None
            for var in [
                self.marker_distance,
                self.marker_x,
                self.marker_y,
                self.marker_theta
            ]
        )
    
    
    def send_twist_message(self, linear_x, angular_z):
        twist_msg = Twist()
        twist_msg.linear.x = linear_x
        twist_msg.angular.z = angular_z
        self.publisher.publish(twist_msg)


def get_quaternion_from_euler(roll, pitch, yaw):
    qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
    qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
    qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
 
    return [qx, qy, qz, qw]

def quaternion(quaternion):
    x = quaternion[0]
    y = quaternion[1]
    z = quaternion[2]
    w = quaternion[3]

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
    rclpy.init(args=args)
    logic_node = LogicNode()
    rclpy.spin(logic_node)
    logic_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
