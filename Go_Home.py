import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point
from rclpy.node import Node
import math

class GoHome(Node):

    def __init__(self):
        super().__init__('go_home')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.linear_velocity = 0.0
        self.angular_velocity = 0.0
        self.initial_position = None
        
        self.odom_subscriber = self.create_subscription(
            Odometry, 
            '/odom',  # Nom du topic d'odométrie
            self.odom_callback, 
            10)
        
    def odom_callback(self, msg):
        print("55555555555555555555555")
   
        if self.initial_position is None:
        # La première fois, on capture la position initiale
            self.initial_position = self.get_position_from_odom(msg)
            self.get_logger().info(f"Position initiale capturée: {self.initial_position}")

    # À chaque itération, on récupère la position actuelle
        current_position = self.get_position_from_odom(msg)
        self.get_logger().info(f"Position actuelle: {current_position}")

    # Ensuite, vous pouvez utiliser ces positions dans go_home
        self.go_home(self.initial_position, current_position)

        
        
        
    def get_position_from_odom(self, msg):
        # Extraire la position depuis les données d'odométrie (en mètres)
        position = msg.pose.pose.position
        return (position.x, position.y)
    
    def go_home(self, initial_position, current_position):
        print(",,,,,,,,,,,,,,,,,,,,,,,,,")
        x_initial, y_initial = initial_position
        x_current, y_current = current_position
        print("position initiale:",x_initial,y_initial,"position actuelle : ",x_current,y_current)
        # Calcul de la distance et de l'angle à la position initiale
        x_delta = abs(x_initial - x_current)
        y_delta = abs(y_initial - y_current)
        print("delta mesuré : ",x_delta,y_delta)
        distance = math.sqrt(x_delta ** 2 + y_delta ** 2)
        print("la distance mesurée :",distance)
        angle_to_goal = math.atan2(y_delta, x_delta) * (180 / 3.14)
        print("l'angle mesuré :",angle_to_goal)
        

        # Ajuster les vitesses
        if abs(angle_to_goal) > 0.5:  # Alignement
            self.linear_velocity = 5
            self.angular_velocity = max(min(angle_to_goal * 0.1, 0.5), -0.5)
            print("???????????????????????")
        else:
            self.linear_velocity = min(distance * 0.5, 0.5)
            self.angular_velocity = 0.0
            print("???????????????????????")

        # Arrêter si proche de la position initiale
        if distance < 0.1:
            self.linear_velocity = 0.0
            self.angular_velocity = 0.0
            self.get_logger().info("Arrivé à la position initiale.")

        # Publier les vitesses pour déplacer le robot
        self.publish_velocity()

    def publish_velocity(self):
        msg = Twist()
        msg.linear.x = self.linear_velocity
        msg.angular.z = self.angular_velocity
        self.publisher_.publish(msg)
        self.get_logger().info(f"Vitesses publiées -> LIN: {msg.linear.x}, ANG: {msg.angular.z}")


