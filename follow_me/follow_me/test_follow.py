import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from rclpy.qos import qos_profile_sensor_data
from rclpy.qos import QoSProfile
import math
import numpy as np


class FollowMe(Node):

    def __init__(self):
        super().__init__('follow_me')

        #Quality of Service
        qos = QoSProfile(depth=10)

        # TODO Distances de mesures
        self.range_min = 0
        self.range_max = 3.5

        #Distance moyenne de mesure
        self.mid_range = (self.range_max + self.range_min) / 2

        # TODO Angle de mesure (en deg)
        self.measure_angle_deg = 50

        #Initialisation du Scan
        self.scan_ranges = []
        self.activated_scan = False

        # Initialisation des vitesses
        self.linear_velocity = 0.0
        self.angular_velocity = 0.0

        # TODO Défintion des gains de vitesse
        self.k_linear = 1
        self.k_angular = 1

        #TODO Limitations de vitesses
        self.max_linear_velocity = 0.5
        self.max_angular_velocity = 0.5

        # Création des publisher/subscriber
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', qos)

        self.scan_sub = self.create_subscription(
            LaserScan,
            'scan',
            self.scan_callback,
            qos_profile=qos_profile_sensor_data)

        #Création du timer qui lance à chaque instance la fonction callback.
        timer_period = 0.5  # TODO Période en secondes
        self.timer = self.create_timer(timer_period, self.timer_callback)

        #Calcul du centre de la plage de mesure
        self.x_center, self.y_center = FollowMe.polaire_vers_cartesien(0, self.mid_range)
        self.get_logger().info(f'Position centrale de mesure: Pos_X = "{self.x_center}", Pos_Y = "{self.y_center}"')
        

    # Fonction du scan à chaque itération de mesure du lidar (toutes les 0.2 secondes)
    def scan_callback(self, msg):
        self.activated_scan = True # Activation du scan

        measure_angle_rad = math.radians(self.measure_angle_deg) #Conversion de l'angle de mesure en radians
        nb_measure_half = int(measure_angle_rad / msg.angle_increment) #Nombre de mesure sur la plage de mesure

        start_measure = 0
        end_measure = nb_measure_half

      # Mesures à partir du début et de la fin
        scan_ranges_front = msg.ranges[start_measure:end_measure]
        scan_ranges_back = msg.ranges[-nb_measure_half:]

        # Combinaison des mesures des deux côtés
        #self.scan_ranges = scan_ranges_front + scan_ranges_back
        self.scan_ranges = msg.ranges

        #self.get_logger().info(f'SCAN RANGES: {len(self.scan_ranges)}')

        # Moitié du nombre de mesures total de la plage
        self.half_scan_ranges = len(self.scan_ranges) / 2

        filtered_points = []

        for i, valeur in enumerate(self.scan_ranges):
            if self.range_min <= valeur <= self.range_max and not math.isinf(valeur) and not math.isnan(valeur):
                # POI

                angle = msg.angle_min +  i * msg.angle_increment
                if (angle < np.deg2rad(self.measure_angle_deg/2)) or (angle > (2*math.pi - np.deg2rad(self.measure_angle_deg/2))):
                    x, y = FollowMe.polaire_vers_cartesien(angle, valeur)
                    filtered_points.append([x,y])
                    self.get_logger().info(f"I {i} et ANGLE {angle}")


        
        self.get_logger().info(f"Filtered point: {len(filtered_points)}")
        


        # Vérification s'il existe des distances valides
        if not filtered_points:
            self.linear_velocity = 0.0
            self.angular_velocity = 0.0
            self.get_logger().info("Aucun obstacle détecté dans la plage de mesure.")
            return

        # Calcul des coordonnées cartésiennes pour chaque mesure
        x_total = 0.0
        y_total = 0.0
        for p in filtered_points:
            x = p[0]
            y = p[1]
            x_total += x
            y_total += y

        # Calcul du point cartésien moyen
        count = len(filtered_points)
        x_mean = x_total / count
        y_mean = y_total / count
        self.get_logger().info(f"Y_TOTAl = {y_mean} COUNT = {count}")


        # Affichage des résultats
        self.get_logger().info(f"Obstacle moyen: x = {x_mean}, y = {y_mean}")

        # Calcul de l'erreur entre position centre et position moyenne des obstacles
        x_delta = x_mean - self.x_center
        y_delta = y_mean - self.y_center
        #self.get_logger().info(f"Y_CENTER:{self.y_center}")



        # Modification des vitesses en fonction de l'erreur de position
        self.linear_velocity =  x_delta * self.k_linear
        self.angular_velocity =  y_delta * self.k_angular

        #self.linear_velocity = 0.0
        #self.angular_velocity = 0.0
        #self.get_logger().info(f"Position Moyenne -> LIN: {x_mean}, ANG: {y_mean}")


        self.linear_velocity = min(self.linear_velocity, self.max_linear_velocity)
        self.angular_velocity = max(min(self.angular_velocity, self.max_angular_velocity), -self.max_angular_velocity)

        #Logs des vitesses
        #self.get_logger().info(f"V_Lin: {self.linear_velocity}, V_Ang: {self.angular_velocity}")

    def timer_callback(self):
        msg = Twist()
        msg.linear.x = self.linear_velocity
        msg.angular.z = self.angular_velocity

        # Publication du message
        self.publisher_.publish(msg)
        self.get_logger().info(f"Vitesses publiées -> LIN: {msg.linear.x}, ANG: {msg.angular.z}")

    def polaire_vers_cartesien(angle, distance):

    # Convertit des coordonnées polaires (angle, distance) en coordonnées cartésiennes (x, y).
    
    # Arguments:
    # - angle : float - l'angle en radians
    # - distance : float - la distance du point par rapport à l'origine
    
    # Retourne:
    # - tuple (x, y) des coordonnées cartésiennes

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

    # Destruction explicite du noeud
    followme.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
