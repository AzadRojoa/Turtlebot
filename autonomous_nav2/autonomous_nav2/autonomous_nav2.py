from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import rclpy
from rclpy.duration import Duration
from rclpy.qos import QoSProfile
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.node import Node
from std_msgs.msg import String
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Twist


class GoHome(Node):
    def __init__(self):
        super().__init__('go_home')
        # Initialisation de position par défaut
        self.initial_pose_x = 0.0
        self.initial_pose_y = 0.0
        self.initial_orientation_z = 0.0
        self.initial_orientation_w = 0.0
        self.start_go_home = False
        qos = QoSProfile(depth=5)
        # Création de la stack de navigation
        self.navigator = BasicNavigator()
        self.goal_pose = PoseStamped()

        # Création des publisher move2 et cmd_vel
        self.move_publisher_ = self.create_publisher(Float32MultiArray,'move2',qos)
        self.publisher_cmd = self.create_publisher(Twist, '/cmd_vel', qos)

        # Abonnement au topic move et amcl_pose
        self.scan_sub = self.create_subscription(
            String,
            'move',
            self.message_callback,
            qos_profile=qos_profile_sensor_data)

        self.pose = self.create_subscription(
            PoseWithCovarianceStamped,
            'amcl_pose',
            self.init_pose_callback,
            qos_profile=qos_profile_sensor_data)
        self.get_logger().info(f'oui')

        
    def message_callback(self,msg):
        # Vérification de la réception du message sur le topic move
        if msg.data :
            # Initialisation de la position de retoure
            self.goal_pose.header.frame_id = 'map'
            self.goal_pose.header.stamp = self.navigator.get_clock().now().to_msg()
            self.goal_pose.pose.position.x = self.initial_pose_x
            self.goal_pose.pose.position.y = self.initial_pose_y
            self.goal_pose.pose.orientation.w = 1.0
            self.goal_pose.pose.orientation.z = self.initial_orientation_z
            # Netoyage du LocalCostmap pour avoir un retour fluide
            self.navigator.clearLocalCostmap()
            # Lancement de la navigation autonome   
            self.navigator.goToPose(self.goal_pose)
            i = 0
            # Compteur pour savoir dans combien de temps le robot arrive
            while not self.navigator.isTaskComplete():
                i = i + 1
                feedback = self.navigator.getFeedback()
                if feedback and i % 5 == 0:
                    print(
                        'Estimated time of arrival: '
                        + '{0:.0f}'.format(
                            Duration.from_msg(feedback.estimated_time_remaining).nanoseconds
                            / 1e9
                        )
                        + ' seconds.'
                    )
            # Affichage du résultat de la navigation
            result = self.navigator.getResult()
            if result == TaskResult.SUCCEEDED:
                print('Goal succeeded!')
            elif result == TaskResult.CANCELED:
                print('Goal was canceled!')
            elif result == TaskResult.FAILED:
                print('Goal failed!')
            else:
                print('Goal has an invalid return status!')
            # Création du type pour entrer les valeur a publier dans le topic move2
            msg_move2=Float32MultiArray()
            msg_move2.data = [self.initial_pose_x,self.initial_pose_y,self.initial_orientation_z]
            self.move_publisher_.publish(msg_move2)
            # Création du type pour entrer les valeur a publier dans le topic cmd_vel
            msg = Twist()
            # Variable pour que le robot arrete de bouger apres la navigation
            msg.linear.x = 0.0
            msg.angular.z = 0.0
            # Publication dans le topic cmd_vel
            self.publisher_cmd.publish(msg)

    
    def init_pose_callback(self, msg):
        # On vérifie que on intialise bien les premieres valeurs 
        if self.initial_pose_x == 0 and self.initial_pose_y == 0 and self.initial_orientation_z == 0 :
            # Initialisation des position initial
            self.initial_pose_x = msg.pose.pose.position.x
            self.initial_pose_y = msg.pose.pose.position.y
            self.initial_orientation_z = msg.pose.pose.orientation.z
            self.initial_orientation_w = msg.pose.pose.orientation.w
            initial_pose = PoseStamped()
            initial_pose.header.frame_id = 'map'
            initial_pose.header.stamp = self.navigator.get_clock().now().to_msg()
            initial_pose.pose.position.x = self.initial_pose_x
            initial_pose.pose.position.y = self.initial_pose_y
            initial_pose.pose.orientation.z = self.initial_orientation_z
            initial_pose.pose.orientation.w = self.initial_orientation_w
            self.navigator.setInitialPose(initial_pose)

            self.navigator.waitUntilNav2Active()


def main(args=None):
    rclpy.init(args=args)
    logger = rclpy.logging.get_logger('main_logger')
    # Création du noeud
    go_home = GoHome()

    try:
        # Lancement du noeud
        rclpy.spin(go_home)
    except KeyboardInterrupt:
        logger.info("Interruption manuelle détectée.")
    finally:
        # Destruction explicite du noeud
        go_home.destroy_node()
        logger.info("Nœud détruit. Retour au main().")

    rclpy.shutdown()


if __name__ == '__main__':
    main()
