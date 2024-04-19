import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path, Odometry
from scipy.spatial.distance import directed_hausdorff, cdist
import math
import numpy as np
from std_msgs.msg import Bool

class KpiGenerator(Node):
    plan_recieved = False
    travelled_path = []
    latest_recieved_plan = []
    goal_reached = False

    def __init__(self):
        super().__init__("kpi_generator")
        self.plan_subscription = self.create_subscription(Path, 'plan', self.plan_callback, 1)
        self.odometry_subscription = self.create_subscription(Odometry, "odometry/global", self.odometry_callback, 1)
        self.result_subscription = self.create_subscription(Bool, 'navigation_result', self.result_callback, 10)

    def plan_callback(self, msg):
        if not self.plan_recieved:
            self.latest_recieved_plan = msg.poses
            self.plan_recieved = True

    def odometry_callback(self, msg):
        position = msg.pose.pose.position
        if not self.goal_reached:
            self.travelled_path.append(msg.pose)
            if self.plan_recieved:
                error = self.calculate_cross_track_error(position)
                self.get_logger().info(f"Cross Track Error: {error:.3f} meters")

    def result_callback(self, msg):
        if msg.data and not self.goal_reached:
            self.get_logger().info("NAVIGATION RESULT {}".format(msg.data))
            self.goal_reached = True
            self.generate_kpi()

    def generate_kpi(self):
        self.get_logger().info("GENERATING KPI")
        self.compare_plans(self.travelled_path, self.latest_recieved_plan)
        self.get_logger().info("Area Difference: %f" % (abs(self.calculate_area(self.travelled_path) - self.calculate_area(self.latest_recieved_plan))))
        self.get_logger().info("Index Area Difference: %f" % (abs(self.calculate_area(self.travelled_path)/self.calculate_path_length(self.travelled_path) - self.calculate_area(self.latest_recieved_plan)/self.calculate_path_length(self.latest_recieved_plan))))

    def plan_to_numpy(self, plan):
        return np.array([[pose.pose.position.x, pose.pose.position.y] for pose in plan], dtype=np.float64)

    def calculate_path_length(self, plan):
        plan_np = self.plan_to_numpy(plan)
        length = np.sum(np.sqrt(np.sum(np.diff(plan_np, axis=0)**2, axis=1)))
        self.get_logger().info("Path length: %f" % length)
        return length

    def calculate_area(self, plan):
        plan_np = self.plan_to_numpy(plan)
        x0, y0 = plan_np[:, 0], plan_np[:, 1]
        area = 0.5 * np.abs(np.dot(x0, np.roll(y0, 1)) - np.dot(y0, np.roll(x0, 1)))
        return area

    def compare_plans(self, plan1, plan2):
        plan1_np = self.plan_to_numpy(plan1)
        plan2_np = self.plan_to_numpy(plan2)
        distance1_to_2 = directed_hausdorff(plan1_np, plan2_np)[0]
        distance2_to_1 = directed_hausdorff(plan2_np, plan1_np)[0]
        self.get_logger().info("Distance from plan 1 to plan 2 : %f" % distance1_to_2)
        self.get_logger().info("Distance from plan 2 to plan 1 : %f" % distance2_to_1)

    def calculate_cross_track_error(self, position):
        current_position = np.array([position.x, position.y])
        plan_positions = self.plan_to_numpy(self.latest_recieved_plan)
        distances = cdist([current_position], plan_positions)
        min_distance = np.min(distances)
        return min_distance

def main(args=None):
    rclpy.init(args=args)
    node = KpiGenerator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
