import rclpy
from rclpy.duration import Duration
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from rclpy.node import Node
import time

class InitialPosePublisher(Node):
    def __init__(self):
        super().__init__('initial_pose_publisher')
        self.publisher = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def publish_initial_pose(self, x, y, orientation_z, orientation_w):
        initial_pose_msg = PoseWithCovarianceStamped()
        initial_pose_msg.header.frame_id = 'map'
        initial_pose_msg.header.stamp = self.get_clock().now().to_msg()

        # Définir la position initiale
        initial_pose_msg.pose.pose.position.x = x
        initial_pose_msg.pose.pose.position.y = y
        initial_pose_msg.pose.pose.orientation.z = orientation_z
        initial_pose_msg.pose.pose.orientation.w = orientation_w


        # Publier la pose
        self.publisher.publish(initial_pose_msg)
        self.get_logger().info(f"Pose initiale publiée : x={x}, y={y}, z={orientation_z}, w={orientation_w}")

    def timer_callback(self):
        msg = PoseWithCovarianceStamped()
        msg.data = 'Hello World: %d' % self.i
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1

def main():
    # Initialiser le système ROS 2
    rclpy.init()

    # Créer un noeud pour publier la pose initiale
    initial_pose_publisher = InitialPosePublisher()

    # Publier la pose initiale
    initial_pose_publisher.publish_initial_pose(0.0, 0.0, 0.0, 1.0)
    time.sleep(1)  # Attendre un court instant pour assurer la publication

    # Lancer le spin pour maintenir le noeud actif
    rclpy.spin(initial_pose_publisher)
    
    # Créer un navigateur pour contrôler le robot
    navigator = BasicNavigator()

    # Définir la pose initiale via le navigateur (optionnel, redondance pour AMCL)
    initial_pose = PoseStamped()
    initial_pose.header.frame_id = 'map'
    initial_pose.header.stamp = navigator.get_clock().now().to_msg()
    initial_pose.pose.position.x = 0.0  # Point de départ X
    initial_pose.pose.position.y = 0.0  # Point de départ Y
    initial_pose.pose.orientation.z = 0.0  # Orientation initiale
    initial_pose.pose.orientation.w = 1.0
    navigator.setInitialPose(initial_pose)

    # Activer la navigation
    navigator.waitUntilNav2Active()

    print("Robot initialisé et prêt pour la navigation.")

    # Vérifier que le robot est bien localisé
    print("En attente de localisation AMCL...")
    time.sleep(2)

    # Définir un premier objectif
    goal_pose1 = PoseStamped()
    goal_pose1.header.frame_id = 'map'
    goal_pose1.header.stamp = navigator.get_clock().now().to_msg()
    goal_pose1.pose.position.x = 3.5  # Objectif X
    goal_pose1.pose.position.y = 2.0  # Objectif Y
    goal_pose1.pose.orientation.z = 0.0
    goal_pose1.pose.orientation.w = 1.0

    # Aller vers le premier objectif
    print("Navigation vers le premier objectif...")
    navigator.goToPose(goal_pose1)

    while not navigator.isTaskComplete():
        feedback = navigator.getFeedback()
        if feedback:
            print(
                f"Temps estimé pour atteindre l'objectif : "
                f"{Duration.from_msg(feedback.estimated_time_remaining).nanoseconds / 1e9:.0f} secondes."
            )

    # Vérifier le résultat de la navigation
    result = navigator.getResult()
    if result == TaskResult.SUCCEEDED:
        print("Objectif atteint avec succès !")
    elif result == TaskResult.FAILED:
        print("La navigation a échoué, retour au point de départ.")
    else:
        print("Navigation annulée ou statut invalide, retour au point de départ.")

    # Retour au point de départ
    print("Retour à la position initiale...")
    navigator.goToPose(initial_pose)

    while not navigator.isTaskComplete():
        feedback = navigator.getFeedback()
        if feedback:
            print(
                f"Temps estimé pour retourner au point de départ : "
                f"{Duration.from_msg(feedback.estimated_time_remaining).nanoseconds / 1e9:.0f} secondes."
            )

    # Vérifier le retour au point de départ
    result = navigator.getResult()
    if result == TaskResult.SUCCEEDED:
        print("Retour au point de départ réussi !")
    elif result == TaskResult.FAILED:
        print("Retour au point de départ échoué !")
    else:
        print("Retour au point de départ annulé ou statut invalide.")

    # Arrêter le robot proprement
    navigator.lifecycleShutdown()
    print("Navigation terminée.")

    # Détruire le noeud de publication
    initial_pose_publisher.destroy_node()
    rclpy.shutdown()
    exit(0)

if __name__ == '__main__':
    main()