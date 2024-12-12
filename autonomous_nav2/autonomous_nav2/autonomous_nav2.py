from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import rclpy
from rclpy.duration import Duration
from rclpy.qos import QoSProfile
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.node import Node

class GoHome(Node):
    def __init__(self):
        super().__init__('go_home')
        self.initial_pose_x = 0
        self.initial_pose_y = 0
        self.initial_orientation_z = 0
        self.initial_orientation_w = 0

        self.actual_pose_x = 0
        self.actual_pose_y = 0
        self.actual_orientation_z = 0
        self.actual_orientation_w = 0
        qos = QoSProfile(depth=5)

        self.pose = self.create_subscription(
            PoseWithCovarianceStamped,
            'amcl_pose',
            self.odom_callback,
            qos_profile=qos_profile_sensor_data)
        self.get_logger().info(f'oui')
    
    def odom_callback(self, msg):
        self.get_logger().info(f'start')
        navigator = BasicNavigator()
        if self.initial_pose_x == 0 and self.initial_pose_y == 0 and self.initial_orientation_z == 0 :
            self.initial_pose_x = msg.pose.pose.position.x
            self.initial_pose_y = msg.pose.pose.position.y
            self.initial_orientation_z = msg.pose.pose.orientation.z
            self.initial_orientation_w = msg.pose.pose.orientation.w
            # Set our demo's initial pose
            initial_pose = PoseStamped()
            initial_pose.header.frame_id = 'map'
            initial_pose.header.stamp = navigator.get_clock().now().to_msg()
            initial_pose.pose.position.x = self.initial_pose_x
            initial_pose.pose.position.y = self.initial_pose_y
            initial_pose.pose.orientation.z = self.initial_orientation_z
            initial_pose.pose.orientation.w = self.initial_orientation_w
            navigator.setInitialPose(initial_pose)

            navigator.waitUntilNav2Active()

            goal_pose = PoseStamped()
            goal_pose.header.frame_id = 'map'
            goal_pose.header.stamp = navigator.get_clock().now().to_msg()
            goal_pose.pose.position.x = 2.0
            goal_pose.pose.position.y = 0.0
            goal_pose.pose.orientation.w = 1.0
            goal_pose.pose.orientation.z = 0.0

        else :
            self.actual_pose_x = msg.pose.pose.position.x
            self.actual_pose_y = msg.pose.pose.position.y
            self.actual_orientation_z = msg.pose.pose.orientation.z
            self.actual_orientation_w = msg.pose.pose.orientation.w

        
        # goal_pose.pose.position.x = self.initial_pose_x
        # goal_pose.pose.position.y = self.initial_pose_y
        # goal_pose.pose.orientation.z = self.initial_orientation_z
        goal_pose.pose.position.x = 2.0
        goal_pose.pose.position.y = 0.0
        goal_pose.pose.orientation.z = self.initial_orientation_z
        navigator.goToPose(goal_pose)
        i = 0
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
        goal_pose.pose.position.x = self.initial_pose_x
        goal_pose.pose.position.y = self.initial_pose_y
        goal_pose.pose.orientation.z = self.initial_orientation_z
        navigator.goToPose(goal_pose)
        i = 0
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
        # Do something depending on the return code
        result = navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            print('Goal succeeded!')
        elif result == TaskResult.CANCELED:
            print('Goal was canceled!')
        elif result == TaskResult.FAILED:
            print('Goal failed!')
        else:
            print('Goal has an invalid return status!')

        navigator.lifecycleShutdown()
        self.destroy_node()
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    logger = rclpy.logging.get_logger('main_logger')

    go_home = GoHome()

    try:
        rclpy.spin(go_home)
    except KeyboardInterrupt:
        logger.info("Interruption manuelle détectée.")
    finally:
        # Destruction explicite du noeud
        go_home.destroy_node()
        logger.info("Nœud détruit. Retour au main().")

    # Destruction explicite du noeud
    logger.info("Ceci est un message depuis le main()")
    rclpy.shutdown()


if __name__ == '__main__':
    main()
