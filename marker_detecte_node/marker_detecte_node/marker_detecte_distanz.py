import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from std_msgs.msg import Int32, Int64
import cv2
import numpy as np
from marker_detecte_node.marker_detection import *
from roboter_interfaces.msg import MakerDistanceID, RoboterInfo
import time

class ArucoMarkerNode(Node):
    def __init__(self):
        """
        Initializes the ArucoMarkerNode.

        This constructor sets up the ArucoMarkerNode with subscribers, publishers, and necessary parameters.
        """
        super().__init__('aruco_marker_node')
        self.bridge = CvBridge()
        
        self.subscriber = self.create_subscription(Image, 'camera_image', self.image_callback, 10)
        self.publisher_marker_info = self.create_publisher(MakerDistanceID, 'aruco_marker_info', 10)
        #self.publisher_roboter_info = self.create_publisher(RoboterInfo, "roboter_info", 10)
        self.x_goal = None
        self.y_goal = None
        self.theta_goal = None
        self.len_marker_front = 0.01  # in meters
        self.len_marker_back_and_goal = 0.1  # in meters
        
    def image_callback(self, msg):
        """
        Callback function for processing camera images.

        This function is called when a camera image is received. It detects Aruco markers in the
        image, calculates their positions, and publishes information about detected markers and the
        robot's position.

        Args:
            msg (Image): The received camera image message.
        """
        frame = self.bridge.imgmsg_to_cv2(msg)
        corners, ids = detect_aruco_markers(frame)
        
        # Extracts the time information from the 'Image' message
        time_sec = msg.header.stamp.sec
        time_nsec = msg.header.stamp.nanosec
        time = float(time_sec) + float(time_nsec) * 1e-9
        
        if ids is not None:
            for i in range(len(ids)):
                
                if len(ids) == 1:
                    distance = calculate_distance(corners, i, self.len_marker_back_and_goal)
                    self.publish_marker_info(ids[i], distance)
                else:
                    distance = calculate_distance(corners, i, self.len_marker_front)
                    self.publish_marker_info(ids[i], distance)  
        else: 
            self.get_logger().info("No Aruco markers found")
            
    def publish_marker_info(self, marker_id, distance):
        """
        Publishes information about detected markers.

        Args:
            marker_id (int): The ID of the detected Aruco marker.
            distance (float): The distance from the camera to the marker.
            x (float): X-coordinate of the marker in the camera frame.
            y (float): Y-coordinate of the marker in the camera frame.
            theta (float): Yaw angle of the marker in the camera frame.
            time (float): Timestamp when the marker information was processed.
        """
        if all(val is not None for val in [marker_id, distance]):
            marker_info_msg = MakerDistanceID()
            marker_info_msg.id = Int32(data=int(marker_id))
            marker_info_msg.distance = distance
            self.publisher_marker_info.publish(marker_info_msg)
        else:
            self.get_logger().error("Condition to send marker information not met")

def main(args=None):
    """
    Main function to execute the ArucoMarkerNode.

    This function initializes the ArucoMarkerNode, executes it using rclpy.spin(), and ensures
    proper cleanup on shutdown.
    """
    rclpy.init(args=args)    
    aruco_node = ArucoMarkerNode()
    rclpy.spin(aruco_node)
    aruco_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


# import cv2
# import rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import Image
# from geometry_msgs.msg import Twist
# from cv_bridge import CvBridge
# import numpy as np
# import time

# camera_matrix = [
#     [2.43442033e+03, 0.00000000e+00, 1.32928359e+03],
#     [0.00000000e+00, 2.42670731e+03, 9.60008894e+02],
#     [0.00000000e+00, 0.00000000e+00, 1.00000000e+00]
# ]

# class ArUcoMarkerNode(Node):
#     def __init__(self):
#         super().__init__('aruco_marker_node')
#         self.subscription = self.create_subscription(Image, 'camera_image', self.image_callback, 10)
#         self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
#         self.bridge = CvBridge()

#     def send_twist_message(self, linear_x, angular_z):
#         twist_msg = Twist()
#         twist_msg.linear.x = linear_x
#         twist_msg.angular.z = angular_z
#         self.publisher.publish(twist_msg)

#     def calculate_distance(self, corners, i, marker_size):
#         """
#         Calculates the distance from the camera to an Aruco marker.

#         Args:
#             corners (list): List of corner points of the detected marker.
#             i (int): Index of the marker in the corners list.
#             marker_size (float): The real-world size of the Aruco marker.

#         Returns:
#             float: Calculated distance from the camera to the marker.
#         """
#         # Convert the camera_matrix to a numpy array
#         camera_matrix_np = np.array(camera_matrix)

#         marker_size_in_pixels = np.linalg.norm(corners[i][0][0] - corners[i][0][1])
#         distance = marker_size * camera_matrix_np[0, 0] / marker_size_in_pixels
#         return distance

#     def calculate_marker_angle(self, corners, image_shape):
#         marker_center = np.mean(corners[0][0], axis=0)
#         camera_center = np.array(image_shape) / 2
#         vector_to_marker = marker_center - camera_center
#         angle_to_marker = np.arctan2(vector_to_marker[1], vector_to_marker[0])
#         return angle_to_marker

#     def calculate_distance_once(self, corners, idx, length):
#         marker_size = 0.1  # You may adjust the marker size
#         distance = self.calculate_distance(corners, idx, marker_size)
#         return distance

#     def image_callback(self, msg):
#         try:
#             cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
#             gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
#             dictionary = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
#             parameters = cv2.aruco.DetectorParameters_create()
#             corners, ids, rejected_img_points = cv2.aruco.detectMarkers(gray, dictionary, parameters=parameters)

#             if ids is not None and len(ids) > 0:
#                 distance = self.calculate_distance_once(corners, 0, 0.1)
#                 stop_distance = 0.1

#                 if distance is not None and distance > stop_distance:
#                     # Calculate linear velocity based on distance
#                     linear_x = 0.1

#                     # Send Twist message with linear velocity
#                     self.send_twist_message(linear_x=linear_x, angular_z=0.0)
#                     self.get_logger().info(f"Linear Velocity: {linear_x}")
#                 else:
#                     self.send_twist_message(linear_x=0.0, angular_z=0.0)
#                     self.get_logger().info("ArUco markers ")
                    
#             else:
#                 # No markers found, stop the robot
#                 self.send_twist_message(linear_x=0.0, angular_z=0.0)
#                 self.get_logger().info("No ArUco markers found")

#         except Exception as e:
#             self.get_logger().error(f"Error in image_callback: {str(e)}")

# def main(args=None):
#     rclpy.init(args=args)
#     aruco_marker_node = ArUcoMarkerNode()
#     rclpy.spin(aruco_marker_node)
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()
