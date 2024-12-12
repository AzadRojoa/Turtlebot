from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import rclpy
from rclpy.duration import Duration
from rclpy.qos import QoSProfile
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

class GoDock(Node):
    def __init__(self):
        super().__init__('go_dock')
        self.initial_pose_x = 0.0
        self.initial_pose_y = 0.0
        self.initial_orientation_z = 0.0
        self.initial_orientation_w = 0.0
        self.start_go_home = False
        qos = QoSProfile(depth=5)
        self.navigator = BasicNavigator()
        self.goal_pose = PoseStamped()

        self.scan_sub = self.create_subscription(
            Float32MultiArray,
            'move2',
            self.message_callback,
            qos_profile=qos_profile_sensor_data)

        

        
    def message_callback(self,msg):
        if msg.data :
            self.get_logger().info(f'start')
            # Set our demo's initial pose
            initial_pose = PoseStamped()
            initial_pose.header.frame_id = 'map'
            initial_pose.header.stamp = self.navigator.get_clock().now().to_msg()
            initial_pose.pose.position.x = msg.data[0]
            initial_pose.pose.position.y = msg.data[1]
            initial_pose.pose.orientation.z = msg.data[2]
            initial_pose.pose.orientation.w = 0
            self.navigator.setInitialPose(initial_pose)

            self.navigator.waitUntilNav2Active()
            
            self.goal_pose.header.frame_id = 'map'
            self.goal_pose.header.stamp = self.navigator.get_clock().now().to_msg()
            self.goal_pose.pose.orientation.w = 1.0
            self.goal_pose.pose.position.x = -3.0
            self.goal_pose.pose.position.y = 0
            self.goal_pose.pose.orientation.z = 0

            self.navigator.goToPose(self.goal_pose)

            i = 0
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
            # Do something depending on the return code
            result = self.navigator.getResult()
            if result == TaskResult.SUCCEEDED:
                print('Goal succeeded!')
            elif result == TaskResult.CANCELED:
                print('Goal was canceled!')
            elif result == TaskResult.FAILED:
                print('Goal failed!')
            else:
                print('Goal has an invalid return status!')

            self.navigator.lifecycleShutdown()
            self.destroy_node()
            rclpy.shutdown()

    


def main(args=None):
    rclpy.init(args=args)
    logger = rclpy.logging.get_logger('main_logger')

    go_home = GoDock()

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
