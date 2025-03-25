import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose

class Nav2GoalMultiplexer(Node):
    def __init__(self):
        super().__init__('nav2_goal_multiplexer')

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        qos_profile = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self.explore_resume_pub = self.create_publisher(Bool, '/a200_0000/explore/resume', qos_profile)
        self.nav_to_pose_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        self.timer = self.create_timer(1.0, self.check_person_frame)
        self.goal_sent = False

    def check_person_frame(self):
        """Loops until 'map' to 'person' transform is available, then stops exploration and navigates."""
        if self.goal_sent:
            return  # Avoid sending multiple goals

        try:
            trans = self.tf_buffer.lookup_transform('map', 'person', rclpy.time.Time())
            self.get_logger().info("Person detected! Stopping exploration and navigating.")
            
            # Stop exploration
            stop_msg = Bool()
            stop_msg.data = False
            
            self.explore_resume_pub.publish(stop_msg)
            
            # Send navigation goal
            goal_msg = NavigateToPose.Goal()
            goal_msg.pose.header.frame_id = 'person'
            goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
            goal_msg.pose.pose.position.x = 2.0
            goal_msg.pose.pose.position.y = 0.0
            goal_msg.pose.pose.position.z = 0.0

            self.nav_to_pose_client.wait_for_server()
            self.send_goal(goal_msg)
            
            self.goal_sent = True
        except Exception as e:
            self.get_logger().info("Waiting for person transform...")

    def send_goal(self, goal_msg):
        """Send goal to Nav2 action server and terminate upon completion."""
        self.get_logger().info("Sending navigation goal...")
        future = self.nav_to_pose_client.send_goal_async(goal_msg)
        future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Navigation goal rejected!")
            self.shutdown_node()
            return

        self.get_logger().info("Navigation goal accepted!")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.goal_result_callback)

    def goal_result_callback(self, future):
        self.get_logger().info("Navigation completed. Shutting down node.")
        self.shutdown_node()

    def shutdown_node(self):
        self.destroy_node()
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = Nav2GoalMultiplexer()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
