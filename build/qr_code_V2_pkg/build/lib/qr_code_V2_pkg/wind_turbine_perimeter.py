import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import PoseArray
from std_msgs.msg import String
import math


class WindTurbinePerimeterDetector(Node):
    def __init__(self):
        super().__init__('wind_turbine_perimeter_detector')

        # Subscriber pour les positions des éoliennes
        self.create_subscription(
            PoseArray,
            '/aquabot/ais_sensor/windturbines_positions',
            self.wind_turbines_callback,
            10
        )

        # Subscriber pour la position GPS du robot
        self.create_subscription(
            NavSatFix,
            '/aquabot/sensors/gps/gps/fix',
            self.boat_gps_callback,
            10
        )

        # Publisher pour indiquer si le robot est dans le périmètre
        self.perimeter_status_publisher = self.create_publisher(
            String,
            '/boat_in_perimeter',
            10
        )

        self.wind_turbines = []  # Liste des positions des éoliennes [(id, x, y)]
        self.boat_position = None  # Position actuelle du robot (x, y)

        self.get_logger().info("Wind Turbine Perimeter Detector Node Initialized.")

    def wind_turbines_callback(self, msg):
        # Récupérer les positions des éoliennes depuis PoseArray
        self.wind_turbines = [
            (i, pose.position.x, pose.position.y)  # Ajouter un ID unique pour chaque éolienne
            for i, pose in enumerate(msg.poses)
        ]
        self.get_logger().info(f"Received wind turbine positions: {self.wind_turbines}")

    def boat_gps_callback(self, msg):
        # Récupérer la position GPS du robot
        self.boat_position = (msg.latitude, msg.longitude)
        self.get_logger().info(f"Boat GPS Position: {self.boat_position}")

        if not self.wind_turbines:
            # Pas d'éoliennes connues pour le moment
            self.publish_status("OUTSIDE_PERIMETER", "none")
            return

        # Trouver l'éolienne la plus proche dans le périmètre
        closest_turbine_id, closest_distance = self.find_closest_turbine()

        if closest_distance < 15.0:
            # Robot est dans le périmètre de l'éolienne la plus proche
            self.publish_status("IN_PERIMETER", str(closest_turbine_id))
        else:
            # Robot est en dehors du périmètre
            self.publish_status("OUTSIDE_PERIMETER", "none")

    def find_closest_turbine(self):
        """
        Trouve l'éolienne la plus proche parmi celles connues.
        Retourne (id, distance_min).
        """
        closest_id = None
        closest_distance = float('inf')  # Initialiser à une valeur très grande

        for turbine_id, turbine_x, turbine_y in self.wind_turbines:
            distance = self.calculate_distance(self.boat_position, (turbine_x, turbine_y))
            if distance < closest_distance:
                closest_id = turbine_id
                closest_distance = distance

        return closest_id, closest_distance

    def calculate_distance(self, pos1, pos2):
        """
        Calcule la distance Euclidienne entre deux positions GPS.
        pos1 et pos2 sont des tuples (latitude, longitude).
        """
        return math.sqrt((pos2[0] - pos1[0]) ** 2 + (pos2[1] - pos1[1]) ** 2)

    def publish_status(self, status, turbine_id):
        """
        Publie le statut (IN_PERIMETER ou OUTSIDE_PERIMETER) et l'ID de l'éolienne la plus proche.
        """
        message = f"{status},{turbine_id}"
        self.perimeter_status_publisher.publish(String(data=message))
        self.get_logger().info(f"Published status: {message}")


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
