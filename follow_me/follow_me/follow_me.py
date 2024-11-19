import rclpy
from sensor_msgs.msg import LaserScan
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist
from rclpy.qos import QoSProfile
from rclpy.node import Node

class ScanFront(Node):
    def __init__(self):
        super().__init__('turtlebot3_detection_front')

        """************************************************************
        ** Initialise variables
        ************************************************************"""
        self.linear_velocity = 0.5  # unit: m/s
        self.angular_velocity = 0.0  # unit: m/s
        self.scan_ranges = []
        self.init_scan_state = False  # To get the initial scan data at the beginning
        self.ten = []

        """************************************************************
        ** Initialise ROS publishers and subscribers
        ************************************************************"""
        qos = QoSProfile(depth=10)

        # Initialise publishers
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', qos)

        # Initialise subscribers
        self.scan_sub = self.create_subscription(
            LaserScan,
            'scan',
            self.scan_callback,
            qos_profile=qos_profile_sensor_data)
        self.cmd_vel_raw_sub = self.create_subscription(
            Twist,
            'cmd_vel_raw',
            self.cmd_vel_raw_callback,
            qos)

        """************************************************************
        ** Initialise timers
        ************************************************************"""
        self.update_timer = self.create_timer(
            0.010,  # unit: s
            self.update_callback)

        self.get_logger().info("Turtlebot3 detection front node has been initialised.")

    def scan_callback(self, msg):
        for i in msg.ranges:
            if msg.ranges.index(i) < 13:
                print(f'index :{msg.ranges.index(i)}, et mesure:{i} ')

    def cmd_vel_raw_callback(self, msg):
        self.linear_velocity = msg.linear.x
        self.angular_velocity = msg.angular.z

    def update_callback(self):
        if self.init_scan_state is True:
            self.detect_obstacle()

    def detect_obstacle(self):
        twist = Twist()
        obstacle_distance = self.scan_ranges
        safety_distance = 0.3  # unit: m


        print(obstacle_distance)    



def main(args=None):
    rclpy.init(args=args)
    scanFront = ScanFront()
    rclpy.spin(scanFront)


if __name__ == '__main__':
    main()
