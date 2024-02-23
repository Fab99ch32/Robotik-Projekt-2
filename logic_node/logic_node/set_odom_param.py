import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
from roboter_interfaces.msg import RoboterInfo

class OdometryOverride(Node):
    def __init__(self):
        """
        Initializes the OdometryOverride class.

        This constructor sets up the OdometryOverride node with a publisher for odometry data and a
        subscription to receive robot information.
        """
        super().__init__('odometry_override_node')
        self.odom_publisher = self.create_publisher(PoseWithCovarianceStamped, 'odom', 10)
        self.sub_roboterInfo = self.create_subscription(RoboterInfo, "roboter_info", self.set_odometry, 10)
        
    def set_odometry(self, msg):
        """
        Callback function for setting odometry data.

        This function is called when robot information is received. It extracts relevant information
        and publishes it as odometry data.

        Args:
            msg (RoboterInfo): The received robot information message.
        """
        x_roboter = msg.roboter_x
        y_roboter = msg.roboter_y
        theta_roboter = msg.roboter_theta
        
        odom_msg = PoseWithCovarianceStamped()
        odom_msg.pose.pose.position.x = x_roboter
        odom_msg.pose.pose.position.y = y_roboter
        odom_msg.pose.pose.orientation.z = theta_roboter

        # Publish the odometry data
        self.odom_publisher.publish(odom_msg)

def main(args=None):
    """
    Main function to execute the Odometry Override Node.

    This function initializes the OdometryOverride node, executes it once using rclpy.spin_once(),
    and prints a success message. It handles keyboard interrupts gracefully and ensures the node
    is properly cleaned up.
    """
    rclpy.init(args=args)
    odometry_override_node = OdometryOverride()

    try:
        rclpy.spin_once(odometry_override_node)
        print("Odometry Override Node has been successfully executed.")
    except KeyboardInterrupt:
        pass
    finally:
        odometry_override_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
