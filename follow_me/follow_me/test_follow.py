import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry  # Importer le message d'odométrie
from rclpy.qos import QoSProfile
import math
from std_msgs.msg import String
import numpy as np
import time

class FollowMe(Node):

    def __init__(self):
        super().__init__('follow_me')

        #Quality of Service
        qos = QoSProfile(depth=5)

        # Distances de mesures
        self.range_min = 0.5
        self.range_max = 1.2

        # Distance moyenne de mesure
        self.mid_range = (self.range_max + self.range_min) / 2

        # Angle de mesure (en deg)
        self.measure_angle_deg = 25
    

        # Initialisation du Scan
        self.scan_ranges = []
        self.activated_scan = False

        # Initialisation des vitesses
        self.linear_velocity = 0.0
        self.angular_velocity = 0.0

        # Défintion des gains de vitesse
        self.k_linear = 1
        self.k_angular = 5
        # Limitations de vitesses
        self.max_linear_velocity = 30.0
        self.max_angular_velocity = 50.0

        # Condition d'arret du folow_me
        self.run_follow_me = True

        # Variables pour la détection du mouvement
        self.was_mouving = False
        self.t0 = time.time()

        # Variables pour stocker les coordonnées initiales
        self.x_home = 0.0
        self.y_home = 0.0
        self.theta_home = 0.0  # Angle initial

        # Flag pour savoir si le robot est revenu à la maison
        self.at_home = False

        # Création des publisher/subscriber
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', qos)

        self.move_publisher_ = self.create_publisher(String,'move',qos)

        self.scan_sub = self.create_subscription(
            LaserScan,
            'scan',
            self.scan_callback,
            qos_profile=qos_profile_sensor_data)


        #Création du timer qui lance à chaque instance la fonction callback.
        timer_period = 0.1  # Période en secondes
        self.timer = self.create_timer(timer_period, self.timer_callback)

        #Calcul du centre de la plage de mesure
        self.x_center, self.y_center = FollowMe.polaire_vers_cartesien(0, self.mid_range)
        

    # Fonction du scan à chaque itération de mesure du lidar (toutes les 0.2 secondes)
    def scan_callback(self, msg):
        # Véridfication de la condition d'arret
        if self.run_follow_me :
            self.activated_scan = True # Activation du scan

            # Combinaison des mesures des deux côtés
            self.scan_ranges = msg.ranges

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
                        
            # Vérification s'il existe des distances valides
            if not filtered_points:
                self.linear_velocity = self.linear_velocity
                self.angular_velocity = 0.0
            else:
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

                # Calcul de l'erreur entre position centre et position moyenne des obstacles
                x_delta = x_mean - self.x_center
                y_delta = - y_mean + self.y_center

                # Modification des vitesses en fonction de l'erreur de position
                self.linear_velocity = x_delta * self.k_linear
                self.angular_velocity = y_delta * self.k_angular


            self.linear_velocity = min(self.linear_velocity, self.max_linear_velocity)
            self.angular_velocity = min(self.angular_velocity, self.max_angular_velocity)

            # Vérification de la condition de mouvement
            if abs(self.linear_velocity) < 0.05 and abs(self.angular_velocity) < 0.1:
                self.angular_velocity = 0.0
                self.linear_velocity = 0.0
                if time.time() - self.t0 > 3.0:
                    msg_move = String()
                    msg_move.data = 'Lancement du goHome'
                    self.move_publisher_.publish(msg_move)
                    self.run_follow_me = False
                if self.was_mouving == False:
                    # Avant je bougeais mais plus maintenant
                    self.t0 = time.time()
            else:
                self.was_mouving = True
                self.t0 = time.time()

    def timer_callback(self):
        if self.run_follow_me:
            msg = Twist()
            msg.angular.z = - self.angular_velocity
            msg.linear.x = self.linear_velocity
            # Publication du message
            self.publisher_.publish(msg)

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

    # Création du noeud
    followme = FollowMe()
    logger = rclpy.logging.get_logger('main_logger')

    try:
        # Lancement du noeud
        rclpy.spin(followme)
    except KeyboardInterrupt:
        logger.info("Interruption manuelle détectée.")
    finally:
        # Destruction explicite du noeud
        followme.destroy_node()
        logger.info("Nœud détruit. Retour au main().")

    rclpy.shutdown()


if __name__ == '__main__':
    main()
