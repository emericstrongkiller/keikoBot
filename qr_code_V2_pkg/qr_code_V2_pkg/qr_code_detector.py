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
        self.qr_code_raw_publisher = self.create_publisher(String, '/qr_code_raw', 10)

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
        # Convertir l'image ROS en image OpenCV
        cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
    except CvBridgeError as e:
        self.get_logger().error(f"Failed to convert image: {e}")
        return

    # Détection et décodage du QR code
    data, bbox, _ = self.qr_decoder.detectAndDecode(cv_image)

    if data:
        self.get_logger().info(f"QR Code Detected")
        # Publier les données brutes du QR code pour le script `qr_code_reader`
        self.qr_code_raw_publisher.publish(String(data=data))
    else:
        self.get_logger().info("No QR Code Detected.")

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
