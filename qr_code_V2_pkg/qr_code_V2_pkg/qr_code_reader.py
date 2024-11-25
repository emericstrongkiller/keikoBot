import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json


class QRCodeReader(Node):
    def __init__(self):
        super().__init__('qr_code_reader')

        # Publisher for formatted QR code data to be used by wind_turbine_inspection
        self.wind_turbine_report_publisher = self.create_publisher(
            String,
            '/vrx/windturbineinspection/windturbine_checkup',
            10
        )
        # Subscriber to receive raw QR code data
        self.create_subscription(
            String,  # qr_code_detector publishes raw QR code data
            '/qr_code_raw',  # Topic where qr_code_detector publishes raw QR code data
            self.qr_code_callback,
            10
        )

        self.get_logger().info("QR Code Reader Node Initialized.")

    def qr_code_callback(self, msg):
        # Get raw QR code data from qr_code_detector
        qr_code_data = msg.data

        try:
            # Analyse and format the raw JSON QR code data
            parsed_data = json.loads(qr_code_data)
            report = f"ID: {parsed_data['id']}\n" \
                     f"Status: {parsed_data['state']}\n"

            # Publish formatted QR code data to topic
            self.wind_turbine_report_publisher.publish(String(data=report))
            self.get_logger().info("Published QR code data to wind turbine report topic.")

            #Shutdown qr_code_detector
            self.get_logger().info("Shutting down QR Code Detector Node...")
            self.qr_code_detector_process.terminate()

            # Shutdown the node after publishing
            self.get_logger().info("Shutting down QR Code Reader Node after publishing.")
            self.destroy_node()
            rclpy.shutdown()

        # If data is not in JSON format
        except json.JSONDecodeError as e:
            error_message = f"Invalid JSON received: {qr_code_data}"
            self.get_logger().error(error_message)


def main(args=None):
    rclpy.init(args=args)
    node = QRCodeReader()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down QR Code Reader Node...')
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
