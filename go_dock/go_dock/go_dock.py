from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import rclpy
from rclpy.duration import Duration
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import time

class GoDock(Node):
    def __init__(self):
        super().__init__('go_dock')
        # Initialisation de position par défaut
        self.initial_pose_x = 0.0
        self.initial_pose_y = 0.0
        self.initial_orientation_z = 0.0
        self.initial_orientation_w = 0.0
        self.start_go_dock = False
        
        # Abonnement au topic move2 et amcl_pose
        self.scan_sub = self.create_subscription(
            Float32MultiArray,
            'move2',
            self.message_callback,
            qos_profile=qos_profile_sensor_data)
        
        self.pose = self.create_subscription(
            PoseWithCovarianceStamped,
            'amcl_pose',
            self.dock_callback,
            qos_profile=qos_profile_sensor_data)
        

        

    
    def message_callback(self,msg):
        # Vérification de la réception du message sur le topic move2
        if msg.data :
            # Création de la stack de navigation
            navigator = BasicNavigator()
            # Déclaration du lancement du docking
            self.start_go_dock == True
            # Initialisation des position initial
            initial_pose = PoseStamped()
            initial_pose.header.frame_id = 'map'
            initial_pose.header.stamp = navigator.get_clock().now().to_msg()
            initial_pose.pose.position.x = msg.data[0]
            initial_pose.pose.position.y = msg.data[1]
            initial_pose.pose.orientation.z = msg.data[2]
            initial_pose.pose.orientation.w = 0.0
            navigator.setInitialPose(initial_pose)

            navigator.waitUntilNav2Active()
            # Initialisation de la position souhaiter
            goal_pose = PoseStamped()
            goal_pose.header.frame_id = 'map'
            goal_pose.header.stamp = navigator.get_clock().now().to_msg()
            goal_pose.pose.orientation.w = 0.9947749900040213
            goal_pose.pose.position.x = -0.7256793142341665
            goal_pose.pose.position.y = 5.795135239804399
            goal_pose.pose.orientation.z = 0.1020917198527843
            # Lancement de la navigation autonome   
            navigator.goToPose(goal_pose)

            i = 0
            # Compteur pour savoir dans combien de temps le robot arrive
            while not navigator.isTaskComplete():
                i = i + 1
                feedback = navigator.getFeedback()
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
            result = navigator.getResult()
            if result == TaskResult.SUCCEEDED:
                print('Goal succeeded!')
            elif result == TaskResult.CANCELED:
                print('Goal was canceled!')
            elif result == TaskResult.FAILED:
                print('Goal failed!')
            else:
                print('Goal has an invalid return status!')
            
            
    # Ca ne se fait pas appeler je ne sais pas pourquoi
    def dock_callback(self,msg):
        # Check si on peut run le docking
        if self.start_go_dock :
            # Création de la stack de navigation
            navigator = BasicNavigator()
            # Initialisation des position initial
            self.initial_pose_x = msg.pose.pose.position.x
            self.initial_pose_y = msg.pose.pose.position.y
            self.initial_orientation_z = msg.pose.pose.orientation.z
            self.initial_orientation_w = msg.pose.pose.orientation.w
            initial_pose = PoseStamped()
            initial_pose.header.frame_id = 'map'
            initial_pose.header.stamp = navigator.get_clock().now().to_msg()
            initial_pose.pose.position.x = self.initial_pose_x
            initial_pose.pose.position.y = self.initial_pose_y
            initial_pose.pose.orientation.z = self.initial_orientation_z
            initial_pose.pose.orientation.w = self.initial_orientation_w
            navigator.setInitialPose(initial_pose)

            navigator.waitUntilNav2Active()
            # Initialisation de la position souhaiter
            goal_pose = PoseStamped()
            goal_pose.header.frame_id = 'map'
            goal_pose.header.stamp = navigator.get_clock().now().to_msg()
            goal_pose.pose.orientation.w = 0.9947749900040213
            goal_pose.pose.position.x = -0.7256793142341665
            goal_pose.pose.position.y = 5.795135239804399
            goal_pose.pose.orientation.z = 0.1020917198527843
            # Lancement de la navigation autonome
            navigator.goToPose(goal_pose)
            i = 0
            # Compteur pour savoir dans combien de temps le robot arrive
            while not navigator.isTaskComplete():
                i = i + 1
                feedback = navigator.getFeedback()
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
            result = navigator.getResult()
            if result == TaskResult.SUCCEEDED:
                print('Goal succeeded!')
            elif result == TaskResult.CANCELED:
                print('Goal was canceled!')
            elif result == TaskResult.FAILED:
                print('Goal failed!')
            else:
                print('Goal has an invalid return status!')
            # Vérification de la précision du docking
            if self.initial_pose_x != goal_pose.pose.position.x and self.initial_pose_y != goal_pose.pose.position.y and self.initial_orientation_z != goal_pose.pose.orientation.x :
                navigator.goToPose(goal_pose)
            else:
                time.sleep(10)

    


def main(args=None):
    rclpy.init(args=args)
    logger = rclpy.logging.get_logger('main_logger')
    # Création du noeud
    go_dock = GoDock()

    try:
        # Lancement du noeud
        rclpy.spin(go_dock)
    except KeyboardInterrupt:
        logger.info("Interruption manuelle détectée.")
    finally:
        # Destruction explicite du noeud
        go_dock.destroy_node()
        logger.info("Nœud détruit. Retour au main().")

    rclpy.shutdown()


if __name__ == '__main__':
    main()
