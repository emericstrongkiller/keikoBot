import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge, CvBridgeError
import cv2

class QRCodeDetector(Node):
    def __init__(self):
        super().__init__('qr_code_detector')

        # Initialisation
        self.bridge = CvBridge()
        self.qr_decoder = cv2.QRCodeDetector()

        # Publisher for detection status
        self.qr_status_publisher = self.create_publisher(String, '/qr_code_detection/status', 10)

        # Subscriber for image data
        self.create_subscription(
            Image,
            '/aquabot/sensors/cameras/main_camera_sensor/image_raw',
            self.image_callback,
            10
        )

        self.get_logger().info("QR Code Detection Node Initialized.")

    def image_callback(self, msg):
        try:
            # Convert ROS image to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            self.get_logger().error(f"Failed to convert image: {e}")
            return

        # Detect QR code (bbox is not None if a QR code is detected)
        _, bbox, _ = self.qr_decoder.detectAndDecode(cv_image)

        if bbox is not None:
            self.get_logger().info("QR Code Detected.")
            self.qr_status_publisher.publish(String(data="QR Code Detected"))
        else:
            self.get_logger().info("No QR Code Detected.")
            self.qr_status_publisher.publish(String(data="No QR Code Detected"))


def main(args=None):
    rclpy.init(args=args)
    node = QRCodeDetector()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down QR Code Detector Node...')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
