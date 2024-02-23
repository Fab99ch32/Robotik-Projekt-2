import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from time import time, sleep
import numpy as np

class CameraNode(Node):
    def __init__(self, camera_index=0, framerate=10):
        """
        Initializes the CameraNode.

        Args:
            camera_index (int): Index of the camera to use (default is 0).
            framerate (int): Framerate for image capturing (default is 10).
        """
        super().__init__('camera_node')
        self.__camera = cv2.VideoCapture(camera_index)
        self.__framerate = framerate
        self.bridge = CvBridge()

        self.camera_matrix = [
            [2.43442033e+03, 0.00000000e+00, 1.32928359e+03],
            [0.00000000e+00, 2.42670731e+03, 9.60008894e+02],
            [0.00000000e+00, 0.00000000e+00, 1.00000000e+00]
        ]

        self.distortion_coefficients = [
            [0.09842787, -0.34590361, -0.00035382, 0.00109482, 0.295309]
        ]

        # Create a publisher for the grayscale image
        self.publisher = self.create_publisher(Image, 'camera_image', 10)

    def capture_and_publish(self):
        """
        Captures an image from the camera, performs undistortion, converts it to grayscale, and publishes it.

        The undistorted and grayscale image is published as a ROS Image message.
        """
        ret, original = self.__camera.read()
        timestamp = time()
        if not ret:
            self.get_logger().error('Image capturing failed.')
            sleep(1)
            return
        else:
            self.get_logger().info("Image capturing succeeded.")

        undistorted_image = self.undistort_image(original)
        gray_image = cv2.cvtColor(undistorted_image, cv2.COLOR_BGR2GRAY)

        # Convert the OpenCV image to a ROS Image message
        ros_image = self.bridge.cv2_to_imgmsg(gray_image, 'mono8')

        ros_image.header.stamp.sec = int(timestamp)
        ros_image.header.stamp.nanosec = int((timestamp - int(timestamp)) * 1000000000)
        # Publish the ROS Image message
        self.publisher.publish(ros_image)

    def undistort_image(self, image):
        """
        Undistorts the input image using camera matrix and distortion coefficients.

        Args:
            image (numpy.ndarray): Input image.

        Returns:
            numpy.ndarray: Undistorted image.
        """
        if self.camera_matrix is not None and self.distortion_coefficients is not None:
            # Convert the camera_matrix and distortion_coefficients to numpy arrays
            camera_matrix_np = np.array(self.camera_matrix)
            distortion_coefficients_np = np.array(self.distortion_coefficients)

            # Ensure they are of the correct data type
            camera_matrix_np = camera_matrix_np.astype('float64')
            distortion_coefficients_np = distortion_coefficients_np.astype('float64')

            # Perform undistortion
            undistorted_image = cv2.undistort(image, camera_matrix_np, distortion_coefficients_np)
            return undistorted_image
        else:
            return image


def main(args=None):
    rclpy.init(args=args)
    camera_node = CameraNode()

    try:
        # Use a timer to call the capture_and_publish method at a specified frequency
        timer_period = 1.0 / 10
        timer = camera_node.create_timer(timer_period, camera_node.capture_and_publish)
        rclpy.spin(camera_node)
    except KeyboardInterrupt:
        # Handle Ctrl+C gracefully
        pass
    finally:
        camera_node.__camera.release()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
