import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import PoseArray
from std_msgs.msg import String
import math
import subprocess


class WindTurbinePerimeterDetector(Node):
    def __init__(self):
        super().__init__('wind_turbine_perimeter_detector')

        # Constants for GPS to Cartesian conversion
        self.earth_radius = 6371000.0  # Earth's average radius in meters
        self.ref_lat = None  # Reference latitude (initialized with the first GPS message)
        self.ref_lon = None  # Reference longitude (initialized with the first GPS message)

        # Subscriber for wind turbine positions
        self.create_subscription(
            PoseArray,
            '/aquabot/ais_sensor/windturbines_positions',
            self.wind_turbines_callback,
            10
        )

        # Subscriber for boat GPS position
        self.create_subscription(
            NavSatFix,
            '/aquabot/sensors/gps/gps/fix',
            self.boat_gps_callback,
            10
        )

        # Publisher to indicate whether the boat is within the perimeter
        self.perimeter_status_publisher = self.create_publisher(
            String,
            '/boat_in_perimeter',
            10
        )

        self.wind_turbines = []  # List of wind turbine positions [(x, y)]
        self.boat_position = None  # Current position of the boat (x, y)
        

        self.get_logger().info("Wind Turbine Perimeter Detector Node Initialized.")

    def wind_turbines_callback(self, msg):

        if self.ref_lat is None or self.ref_lon is None:
            self.get_logger().info("Waiting for GPS reference to initialize wind turbine positions.")
            return

        # Convert wind turbine positions to Cartesian coordinates
        self.wind_turbines = []
        for pose in msg.poses:
            latitude = pose.position.x
            longitude = pose.position.y
            x, y = self.convert_gps_to_cartesian(latitude, longitude)
            self.wind_turbines.append((x, y))

    def boat_gps_callback(self, msg):

        # Initialize GPS reference if necessary
        if self.ref_lat is None or self.ref_lon is None:
            self.ref_lat = msg.latitude
            self.ref_lon = msg.longitude
            return

        # Convert the boat's GPS position to Cartesian coordinates
        self.boat_position = self.convert_gps_to_cartesian(msg.latitude, msg.longitude)
        self.get_logger().info(f"Boat Cartesian Position: {self.boat_position}")

        # Check if wind turbine list is empty
        if not self.wind_turbines:
            self.get_logger().warning("No wind turbines available for distance calculations.")
            self.publish_status("OUTSIDE_PERIMETER", None)
            return

        # Check if the boat is within the perimeter of any wind turbine
        in_perimeter = False
        for idx, (turbine_x, turbine_y) in enumerate(self.wind_turbines):
            distance = self.calculate_distance(self.boat_position, (turbine_x, turbine_y))
            self.get_logger().info(f"Distance to Turbine {idx}: {distance:.2f}m")

            if distance < 15.0:
                self.publish_status("IN_PERIMETER", idx)
                in_perimeter = True
                break

        if not in_perimeter:
            self.publish_status("OUTSIDE_PERIMETER", -1)

    def calculate_distance(self, pos1, pos2):
        """
        Calculate the Euclidean distance between two Cartesian positions (x, y).
        """
        return math.sqrt((pos2[0] - pos1[0]) ** 2 + (pos2[1] - pos1[1]) ** 2)

    def convert_gps_to_cartesian(self, latitude, longitude):
        """
        Convert GPS coordinates (latitude, longitude) to Cartesian coordinates (x, y).
        """
        lat_rad = math.radians(latitude)
        lon_rad = math.radians(longitude)
        ref_lat_rad = math.radians(self.ref_lat)
        ref_lon_rad = math.radians(self.ref_lon)

        # Compute Cartesian coordinates
        x = self.earth_radius * (lon_rad - ref_lon_rad) * math.cos(ref_lat_rad)
        y = self.earth_radius * (lat_rad - ref_lat_rad)

        return x, y

    def publish_status(self, status, turbine_id):
        """
        Publish the status (IN_PERIMETER or OUTSIDE_PERIMETER) and the ID of the wind turbine.
        """
        message = f"{status},{turbine_id}"
        self.perimeter_status_publisher.publish(String(data=message))
        self.get_logger().info(f"Published status: {message}")
        
        if status == "IN_PERIMETER":
            self.get_logger().info("Activating QR Code Reader.")
            self.qr_code_detector_process = subprocess.Popen(
                ['ros2', 'run', 'qr_code_V2_pkg', 'qr_code_detector']
            )


def main(args=None):
    rclpy.init(args=args)
    node = WindTurbinePerimeterDetector()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down Wind Turbine Perimeter Detector Node...")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
