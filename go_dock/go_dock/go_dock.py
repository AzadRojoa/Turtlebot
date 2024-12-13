from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import rclpy
from rclpy.duration import Duration
from rclpy.qos import QoSProfile
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import time

class GoDock(Node):
    def __init__(self):
        super().__init__('go_dock')
        self.initial_pose_x = 0.0
        self.initial_pose_y = 0.0
        self.initial_orientation_z = 0.0
        self.initial_orientation_w = 0.0
        self.start_go_dock = False
        

        self.scan_sub = self.create_subscription(
            Float32MultiArray,
            'move2',
            self.message_callback,
            qos_profile=qos_profile_sensor_data)
        
        self.get_logger().info(f'oui')
        
        self.pose = self.create_subscription(
            PoseWithCovarianceStamped,
            'amcl_pose',
            self.dock_callback,
            qos_profile=qos_profile_sensor_data)
        

        

    
    def message_callback(self,msg):
        if msg.data :
            navigator = BasicNavigator()
            self.start_go_dock == True
            self.get_logger().info(f'start')
            self.get_logger().info(f"x: {msg.data[0]},y: {msg.data[1]},z: {msg.data[2]}")
            # Set our demo's initial pose
            initial_pose = PoseStamped()
            initial_pose.header.frame_id = 'map'
            initial_pose.header.stamp = navigator.get_clock().now().to_msg()
            initial_pose.pose.position.x = msg.data[0]
            initial_pose.pose.position.y = msg.data[1]
            initial_pose.pose.orientation.z = msg.data[2]
            initial_pose.pose.orientation.w = 0.0
            navigator.setInitialPose(initial_pose)

            navigator.waitUntilNav2Active()
            goal_pose = PoseStamped()
            goal_pose.header.frame_id = 'map'
            goal_pose.header.stamp = navigator.get_clock().now().to_msg()
            goal_pose.pose.orientation.w = 0.9947749900040213
            goal_pose.pose.position.x = -0.7256793142341665
            goal_pose.pose.position.y = 5.795135239804399
            goal_pose.pose.orientation.z = 0.1020917198527843

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
            
            

    def dock_callback(self,msg):
        if self.start_go_dock :
            navigator = BasicNavigator()
            self.get_logger().info(f"CALIBRE")
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
            goal_pose.pose.orientation.w = 0.9947749900040213
            goal_pose.pose.position.x = -0.7256793142341665
            goal_pose.pose.position.y = 5.795135239804399
            goal_pose.pose.orientation.z = 0.1020917198527843

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

            if self.initial_pose_x != goal_pose.pose.position.x and self.initial_pose_y != goal_pose.pose.position.y and self.initial_orientation_z != goal_pose.pose.orientation.x :
                navigator.goToPose(goal_pose)
            else:
                time.sleep(10)

    


def main(args=None):
    rclpy.init(args=args)
    logger = rclpy.logging.get_logger('main_logger')

    go_dock = GoDock()

    try:
        rclpy.spin(go_dock)
    except KeyboardInterrupt:
        logger.info("Interruption manuelle détectée.")
    finally:
        # Destruction explicite du noeud
        go_dock.destroy_node()
        logger.info("Nœud détruit. Retour au main().")

    # Destruction explicite du noeud
    rclpy.shutdown()


if __name__ == '__main__':
    main()
