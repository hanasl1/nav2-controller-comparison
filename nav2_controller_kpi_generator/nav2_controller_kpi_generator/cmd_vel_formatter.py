import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray

class VelocityToForceNode(Node):
    def __init__(self):
        super().__init__('velocity_to_force_node')
        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.listener_callback,
            10)
        self.publisher = self.create_publisher(
            Float32MultiArray,
            'marus_boat/pwm_out',
            10)

        # Placeholder for the inverse allocation matrix, adjust as necessary for your application
        self.inverse_allocation_matrix = np.array([[1, 0, 0], [0, 1, 0], [0, 0, 1]])

    def listener_callback(self, msg):
        force_commands = Float32MultiArray()

        # Convert velocity to force using the inverse allocation matrix
        tau = np.array([msg.linear.x, msg.linear.y, msg.angular.z])
        force_array = np.matmul(self.inverse_allocation_matrix, tau)

        force_commands.data = force_array.tolist()

        self.publisher.publish(force_commands)
        self.get_logger().info('Publishing force commands: %s' % force_commands.data)

def main(args=None):
    rclpy.init(args=args)
    velocity_to_force_node = VelocityToForceNode()
    rclpy.spin(velocity_to_force_node)
    velocity_to_force_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
