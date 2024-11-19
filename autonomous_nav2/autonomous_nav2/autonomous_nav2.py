from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import rclpy
from rclpy.duration import Duration

"""
Basic navigation demo to go to pose and return to initial position.
"""

def main():
    rclpy.init()

    navigator = BasicNavigator()

    # Set the robot's initial pose
    initial_pose = PoseStamped()
    initial_pose.header.frame_id = 'map'
    initial_pose.header.stamp = navigator.get_clock().now().to_msg()
    initial_pose.pose.position.x = 0.03
    initial_pose.pose.position.y = 0.01
    initial_pose.pose.orientation.z = 0.0
    initial_pose.pose.orientation.w = 1.0
    navigator.setInitialPose(initial_pose)

    # Activate navigation
    navigator.waitUntilNav2Active()

    # First goal pose
    goal_pose1 = PoseStamped()
    goal_pose1.header.frame_id = 'map'
    goal_pose1.header.stamp = navigator.get_clock().now().to_msg()
    goal_pose1.pose.position.x = 3.81
    goal_pose1.pose.position.y = 1.1
    goal_pose1.pose.orientation.w = 1.0
    goal_pose1.pose.orientation.z = 0.0


    # Navigate to the first goal
    navigator.goToPose(goal_pose1)

    while not navigator.isTaskComplete():
        feedback = navigator.getFeedback()
        if feedback:
            print(
                f"Estimated time of arrival: "
                f"{Duration.from_msg(feedback.estimated_time_remaining).nanoseconds / 1e9:.0f} seconds."
            )

    # Check the result of the first navigation task
    result = navigator.getResult()
    if result == TaskResult.SUCCEEDED:
        print('First goal succeeded! Returning to initial position...')
    elif result == TaskResult.CANCELED:
        print('First goal was canceled! Returning to initial position anyway...')
    elif result == TaskResult.FAILED:
        print('First goal failed! Returning to initial position anyway...')
    else:
        print('First goal has an invalid return status! Returning to initial position anyway...')

    # Return to the initial position
    navigator.goToPose(initial_pose)

    while not navigator.isTaskComplete():
        feedback = navigator.getFeedback()
        if feedback:
            print(
                f"Returning to initial position. "
                f"Estimated time of arrival: "
                f"{Duration.from_msg(feedback.estimated_time_remaining).nanoseconds / 1e9:.0f} seconds."
            )

    # Final result for returning to the initial position
    result = navigator.getResult()
    if result == TaskResult.SUCCEEDED:
        print('Successfully returned to initial position!')
    elif result == TaskResult.CANCELED:
        print('Return to initial position was canceled!')
    elif result == TaskResult.FAILED:
        print('Failed to return to initial position!')
    else:
        print('Return to initial position has an invalid return status!')

    # Shutdown the navigator
    navigator.lifecycleShutdown()

    exit(0)

if __name__ == '__main__':
    main()