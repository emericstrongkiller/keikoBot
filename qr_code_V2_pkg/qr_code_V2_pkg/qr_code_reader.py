import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class QRCodeReader(Node):
    def __init__(self):
        super().__init__('qr_code_reader')

        # Publisher pour publier les informations décodées du QR code
        self.qr_code_publisher = self.create_publisher(String, '/qr_code_data', 10)

        # Subscriber pour recevoir les QR codes détectés (par exemple, sous forme brute ou données intermédiaires)
        self.create_subscription(
            String,  # Suppose que `qr_code_detector` publie les données brutes du QR code
            '/qr_code_raw',  # Topic où `qr_code_detector` publie les informations du QR code
            self.qr_code_callback,
            10
        )

        self.get_logger().info("QR Code Reader Node Initialized.")

    def qr_code_callback(self, msg):
        # Récupère les données du QR code publiées par `qr_code_detector`
        qr_code_raw = msg.data

        # Traitement ou extraction des informations du QR code (si nécessaire)
        qr_code_data = f"Decoded QR Code: {qr_code_raw}"

        # Log des informations pour vérification
        self.get_logger().info(qr_code_data)

        # Publie les données décodées sur un autre topic
        #self.qr_code_publisher.publish(String(data=qr_code_data))


def main(args=None):
    rclpy.init(args=args)
    node = QRCodeReader()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down QR Code Reader Node...')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
