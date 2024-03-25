import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool

# Define NavigationClient class
class NavigationClient(Node):
    def __init__(self):
        super().__init__('navigation_client')
        self.action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # Create publisher using create_publisher method of Node class
        self.result_publisher = self.create_publisher(Bool, 'navigation_result', 10)

        while not self.action_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().info('Waiting for action server...')

    def send_goal(self, goal_pose):
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = goal_pose

        send_goal_future = self.action_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected by the action server')
            return
        self.get_logger().info('Goal accepted by the action server')

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.result_callback)

    def result_callback(self, future):
        result = future.result().result
        result_msg = Bool()
        if result.result:
            self.get_logger().info('Navigation succeeded!')
            result_msg.data = True
            self.result_publisher.publish(result_msg)
        else:
            self.get_logger().info('Navigation failed!')

def main():
    rclpy.init()
    navigation_client = NavigationClient()

    # Example: Send a navigation goal to (4.0, 4.0, 0.0)
    goal_pose = PoseStamped()
    goal_pose.header.frame_id = 'map'
    goal_pose.pose.position.x = 4.0
    goal_pose.pose.position.y = 4.0
    goal_pose.pose.position.z = 0.0

    navigation_client.send_goal(goal_pose)

    rclpy.spin(navigation_client)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
