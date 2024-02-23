from time import sleep
from std_msgs.msg import Float64, String, Bool
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose2D, Twist
from nav_msgs.msg import Odometry
from roboter_interfaces.msg import MakerDistanceID
from ro36_interfaces.action import Goto
from rclpy.action import ActionClient
from math import pi 
import numpy as np
import time

class PI_Regler:
    def __init__(self, kp, ki):
        self.kp = kp
        self.ki = ki
        self.previous_error = 0
        self.integral = 0
        self.bool_rfid = False


    def ubertragungsfunktion(self, ist_winkel, ist_abstand):
        soll_winkel = 90  # Zielwinkel
        soll_abstand = 0  # Zielabstand
        last_abstand = 0
        last_winkel = 90

        # Fehler berechnen
        error_winkel = -0.1 * (soll_winkel - ist_winkel)
        error_abstand = 5 * (soll_abstand - ist_abstand)

        # Proportional-Integral-Regelung
        self.integral += error_abstand
        drehgeschwindigkeit = -0.01 * (self.ki * error_abstand + self.kp * error_winkel)

        last_abstand = ist_abstand

        return drehgeschwindigkeit

class ControlNode(Node):
    def __init__(self):
        super().__init__('control_node')

        # Abonnieren von Distanz und Winkel
        self.distance_sub = self.create_subscription(Float64, 'distance_to_optical_axis', self.distance_callback, 10)
        self.angle_sub = self.create_subscription(Float64, 'line_angle', self.angle_callback, 10)
        self._action_client = ActionClient(self, Goto, 'action_server')
        self.sub_markerInfo = self.create_subscription(MakerDistanceID, 'aruco_marker_info', self.aruco_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, 'odom',self._odom_callback, 10)
        # Publisher für Winkelgeschwindigkeit (Float64)
        self.angular_velocity_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        self.ist_abstand = 0
        self.regler = PI_Regler(kp=2.7, ki=0.01)
                  
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
        self.stop_distance = 0.50
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
        
    def handle_front_marker(self):
        self.get_logger().info("Roboter wurde von vorne in einer Abstand von {:.2f} m erkannt".format(self.marker_distance))
        
    def handle_goal_marker(self):
        self.get_logger().info("Ziel Position in einer Abstand von {:.2f} m erkannt".format(self.marker_distance))
        
        distance = self.marker_distance 
        if distance is not None and distance > self.stop_distance:
            if self.is_rotate is False:
                self.pose_reached = False 
                self.get_logger().info("Roboter läuft !")
        else:
            self.get_logger().info("Ziel erreicht!")
            self.pose_reached = True
    
                
    def _odom_callback(self,msg):
        self.last_pose_x = msg.pose.pose.position.x
        self.last_pose_y = msg.pose.pose.position.y
        _,_,self.last_pose_theta = quaternion(
            [
                msg.pose.pose.orientation.x,
                msg.pose.pose.orientation.y,
                msg.pose.pose.orientation.z,
                msg.pose.pose.orientation.w,   
            ]
        )
        
    def distance_callback(self, data):
        # Aktualisieren Sie den Wert von ist_abstand
        self.ist_abstand = data.data

    def angle_callback(self, data):
        # Empfangene Winkelgeschwindigkeit
        ist_winkel = data.data  # Beachten Sie das Minuszeichen
        if (ist_winkel is not None):

            # Verwenden Sie die übertragungsfunktion, um die Winkelgeschwindigkeit zu berechnen
            angular_velocity = self.regler.ubertragungsfunktion(ist_winkel, self.ist_abstand)
            

            # Erstelle Twist-Nachricht mit konstanter linearer Geschwindigkeit (0.1)
            if self.pose_reached == False:
                if self.is_rotate == True:
                    twist_msg = Twist()
                    twist_msg.linear.x = 0.0
                    twist_msg.angular.z = angular_velocity
                else :
                    twist_msg = Twist()
                    twist_msg.linear.x = 0.1  
                    twist_msg.angular.z = angular_velocity
            else:
                start_time = time.time() 
                while time.time() - start_time < 4.1: 
                    twist_msg = Twist()
                    twist_msg.linear.x = 0.0
                    twist_msg.angular.z = 0.6
                    self.angular_velocity_pub.publish(twist_msg)

                    # # Perform rotation
                    # rotate_theta = get_quaternion_from_euler(0.0, 0.0, (np.pi+self.last_pose_theta))[3]
                    # self.publish_goal_info(x=self.last_pose_x, y=self.last_pose_y, theta=(np.pi+self.last_pose_theta))
                    
                    self.get_logger().info("Wenden läuft!")
                    
                self.is_rotate = True
                self.pose_reached = False
                twist_msg.linear.x = 0.0
                twist_msg.angular.z = 0.0
            
            self.get_logger().info(f'Drehgeschwindigkeit: {angular_velocity}')
            self.get_logger().info(f'Winkel empfangen: {ist_winkel}')
            self.get_logger().info(f'Abstand empfangen: {self.ist_abstand}')


            # Veröffentliche die Twist-Nachricht
            self.angular_velocity_pub.publish(twist_msg)
    
    def publish_goal_info(self, x, y, theta):
        goal_msg = Goto.Goal()
        goal_msg.pose = Pose2D(x=x , y=y , theta=theta)
        self.is_rotate = False
        return self._action_client.send_goal_async(goal_msg)
    
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
        node = ControlNode()
        rclpy.spin(node)
        rclpy.shutdown()
    
if __name__ == '__main__':
    main()