import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from rclpy.qos import qos_profile_sensor_data
from rclpy.qos import QoSProfile
import math
import time
from go_home import GoHome  # Importer le fichier go_home.py

class FollowMe(Node):

    def __init__(self):
        super().__init__('follow_me')

        # Quality of Service
        qos = QoSProfile(depth=10)

        # Variables d'initialisation
        self.range_min = 0
        self.range_max = 3.5
        self.mid_range = (self.range_max + self.range_min) / 2
        self.measure_angle_deg = 50
        self.linear_velocity = 0.0
        self.angular_velocity = 0.0
        self.k_linear = 1
        self.k_angular = 1
        self.max_linear_velocity = 0.5
        self.max_angular_velocity = 0.5
        self.was_mouving = False
        self.t0 = time.time()
        self.initial_position = None
        self.x_center, self.y_center = FollowMe.polaire_vers_cartesien(0, self.mid_range)

        # Initialiser le GoHome Node
        self.go_home_node = GoHome()  # Instance du GoHome

        # Création des publisher/subscriber
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', qos)
        self.scan_sub = self.create_subscription(
            LaserScan,
            'scan',
            self.scan_callback,
            qos_profile=qos_profile_sensor_data)
        self.timer = self.create_timer(0.5, self.timer_callback)

    def scan_callback(self, msg):
        self.activated_scan = True
        self.get_logger().info(f'Scan')

        measure_angle_rad = math.radians(self.measure_angle_deg)
        nb_measure_half = int(measure_angle_rad / msg.angle_increment)

        self.scan_ranges = msg.ranges
        filtered_points = []

        for i, valeur in enumerate(self.scan_ranges):
            if self.range_min <= valeur <= self.range_max and not math.isinf(valeur) and not math.isnan(valeur):
                angle = msg.angle_min + i * msg.angle_increment
                if (angle < np.deg2rad(self.measure_angle_deg / 2)) or (angle > (2 * math.pi - np.deg2rad(self.measure_angle_deg / 2))):
                    x, y = FollowMe.polaire_vers_cartesien(angle, valeur)
                    filtered_points.append([x, y])

        if not filtered_points:
            self.linear_velocity = 0.0
            self.angular_velocity = 0.0
            self.get_logger().info("Aucun obstacle détecté dans la plage de mesure.")
        else:
            x_total = 0.0
            y_total = 0.0
            for p in filtered_points:
                x = p[0]
                y = p[1]
                x_total += x
                y_total += y

            count = len(filtered_points)
            x_mean = x_total / count
            y_mean = y_total / count

            x_delta = x_mean - self.x_center
            y_delta = y_mean - self.y_center

            self.linear_velocity = x_delta * self.k_linear
            self.angular_velocity = y_delta * self.k_angular

        self.linear_velocity = min(self.linear_velocity, self.max_linear_velocity)
        self.angular_velocity = max(min(self.angular_velocity, self.max_angular_velocity), -self.max_angular_velocity)

        if abs(self.linear_velocity) < 0.1 and abs(self.angular_velocity) < 0.1:
            self.get_logger().info(f"Pas de mouvement {time.time()} {self.t0}")
            if time.time() - self.t0 > 3.0:
                self.get_logger().info(f"Pas de mouvement depuis 3 sec!")
                if self.initial_position is not None:
                    self.get_logger().info("Retour à la position initiale...")
                    self.go_home_node.go_home(self.initial_position, (self.x_center, self.y_center))  # Appel à la fonction go_home
                else:
                    self.get_logger().info("Position initiale non définie, impossible de revenir.")
            if self.was_mouving is False:
                self.t0 = time.time()
        else:
            self.get_logger().info("Mouvement!")
            self.was_mouving = True
            self.t0 = time.time()

    def timer_callback(self):
        msg = Twist()
        msg.linear.x = self.linear_velocity
        msg.angular.z = self.angular_velocity
        self.publisher_.publish(msg)
        self.get_logger().info(f"Vitesses publiées -> LIN: {msg.linear.x}, ANG: {msg.angular.z}")

    def polaire_vers_cartesien(angle, distance):
        x = distance * math.cos(angle)
        y = distance * math.sin(angle)
        return x, y

def main(args=None):
    rclpy.init(args=args)
    followme = FollowMe()
    try:
        rclpy.spin(followme)
    except KeyboardInterrupt:
        pass
    followme.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


