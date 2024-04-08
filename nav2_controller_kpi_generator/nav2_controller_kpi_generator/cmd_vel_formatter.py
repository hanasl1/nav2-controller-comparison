import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray

class VelocityToPWMNode(Node):
    def __init__(self):
        super().__init__('velocity_to_pwm_node')
        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

        self.publisher = self.create_publisher(
            Float32MultiArray,
            'marus_boat/pwm_out',
            10)

    def listener_callback(self, msg):
        pwm_values = Float32MultiArray()

        # Example transformation: Directly mapping linear x and angular z to PWM.
        # You should adjust the logic here based on your specific requirements and hardware setup.
        linear_pwm = msg.linear.x * 100  # Example scaling factor
        angular_pwm = msg.angular.z * 50  # Example scaling factor

        pwm_values.data = [linear_pwm + angular_pwm,linear_pwm- angular_pwm]

        self.publisher.publish(pwm_values)
        self.get_logger().info('Publishing: "%s"' % pwm_values.data)

def main(args=None):
    rclpy.init(args=args)
    velocity_to_pwm_node = VelocityToPWMNode()
    rclpy.spin(velocity_to_pwm_node)
    velocity_to_pwm_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
