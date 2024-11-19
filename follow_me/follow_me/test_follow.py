import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from rclpy.qos import qos_profile_sensor_data
from rclpy.qos import QoSProfile
import math


class FollowMe(Node):

    def __init__(self):
        super().__init__('follow_me')

        #Quality of Service
        qos = QoSProfile(depth=10)

        self.range_min = 0.1
        self.range_max = 3.5

        self.scan_ranges = []
        self.activated_scan = False

        # Initialisation des vitesses
        self.linear_velocity = 0.0
        self.angular_velocity = 0.0

        # Création du publisher pour le topic cmd_vel
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', qos)

        # Création de subscriber
        self.scan_sub = self.create_subscription(
            LaserScan,
            'scan',
            self.scan_callback,
            qos_profile=qos_profile_sensor_data)

        #Création du timer qui lance à chaque instance la fonction callback.
        timer_period = 0.5  # Période en secondes
        self.timer = self.create_timer(timer_period, self.timer_callback)

    # Fonction du scan à chaque itération du timer
    def scan_callback(self, msg):
        self.activated_scan = True # Activation du scan

        measure_angle_deg = 10
        measure_angle_rad = math.radians(measure_angle_deg)
        nb_measure = int(measure_angle_rad / msg.angle_increment)

        center_measure = len(msg.ranges) // 2
        start_measure = max(center_measure - nb_measure // 2, 0)
        end_measure = min(center_measure + nb_measure // 2, len(msg.ranges))

        self.scan_ranges = msg.ranges[start_measure:end_measure] # Implémenter les mesures dans le tableau

        filtered_scan = [valeur for valeur in self.scan_ranges if self.range_min <= valeur <= self.range_max and not math.isinf(valeur) and not math.isnan(valeur)]
        self.get_logger().info('Filtré: "%s"' %filtered_scan)

        self.dist_min = min(filtered_scan)
        self.get_logger().info('Distance minimal: "%s"' %self.dist_min)

    def timer_callback(self):
        msg = Twist()
        msg.angular.z = self.angular_velocity

        # Publication du message
        #self.publisher_.publish(msg)
        #self.get_logger().info('Publishing angular velocity z: "%s"' % msg.angular.z)


def main(args=None):
    rclpy.init(args=args)

    followme = FollowMe()

    try:
        rclpy.spin(followme)
    except KeyboardInterrupt:
        pass

    # Destruction explicite du noeud
    followme.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
