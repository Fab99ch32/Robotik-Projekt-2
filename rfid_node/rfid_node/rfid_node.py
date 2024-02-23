import RPi.GPIO as GPIO
from mfrc522 import SimpleMFRC522
import rclpy
from std_msgs.msg import Int32, String
from rclpy.node import Node
from roboter_interfaces.msg import RfidTag
class RfidNode(Node):
    def __init__(self):
        """
        Initializes the RFID Node.
        """
        super().__init__('rfid_node')
        # Creates an RFID reader
        self._rfid_reader = SimpleMFRC522()
        # Creates a publisher for RFID tags
        self._rfid_publisher = self.create_publisher(String, 'rfid_tag', 10)

    def spin(self):
        """
        Main loop of the RFID Node, continuously reads RFID tags and publishes them.
        """
        while rclpy.ok():
            print("RfidNode running")
            _, text = self._rfid_reader.read()
            msg = String()
            msg.data = str(text)  
            self._rfid_publisher.publish(msg)
            print(f"Published RFID tag: {msg.data}")

    def destroy(self):
        """
        Cleans up GPIO resources when shutting down the node.
        """
        GPIO.cleanup()
        super().destroy_node()

def main():
    """
    Main function to initialize and run the RFID Node.
    """
    rclpy.init()
    rfid_node = RfidNode()
    try:
        rfid_node.spin()
    finally:
        rfid_node.destroy()
        rclpy.shutdown()

if __name__ == '__main__':
    main()