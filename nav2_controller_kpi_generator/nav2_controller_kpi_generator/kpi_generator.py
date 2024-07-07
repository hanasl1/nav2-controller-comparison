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
    time_set = False
    time = 0
    error_array = []

    def __init__(self):
        super().__init__("kpi_generator")
        self.plan_subscription = self.create_subscription(Path, 'korkyra/smoothed_path', self.plan_callback, 1)
        self.odometry_subscription = self.create_subscription(Odometry, "korkyra/odometry/filtered", self.odometry_callback, 1)

    def plan_callback(self, msg):
        self.get_logger().info("received plan")
        if not self.plan_recieved:
            self.get_logger().info("received plan 2")
            self.latest_recieved_plan = msg.poses
            self.plan_recieved = True

    def odometry_callback(self, msg):
        position = msg.pose.pose.position
        if not self.time_set:
            self.get_logger().info("set time")
            self.time = msg.header.stamp.sec
            self.time_set = True

        if position.x < 0.1 and position.y - 6.0 < 0.1 and msg.header.stamp.sec - self.time > 100:
            self.goal_reached = True
            self.generate_kpi()
            rclpy.shutdown()  # Shutdown the node after generating KPI

        self.travelled_path.append(msg.pose)  # Append travelled path continuously

        if self.plan_recieved and not self.goal_reached:
            error = self.calculate_cross_track_error(position)
            self.error_array.append(error)
            self.get_logger().info(f"Cross Track Error: {error:.3f} meters")

    def generate_kpi(self):
        self.get_logger().info("GENERATING KPI")
        self.compare_plans(self.travelled_path, self.latest_recieved_plan)
        self.get_logger().info("Area Difference: %f" % (abs(self.calculate_area(self.travelled_path) - self.calculate_area(self.latest_recieved_plan))))
        self.get_logger().info("Index Area Difference: %f" % (abs(self.calculate_area(self.travelled_path) / self.calculate_path_length(self.travelled_path) - self.calculate_area(self.latest_recieved_plan) / self.calculate_path_length(self.latest_recieved_plan))))

        # Write error_array to a text file
        with open('error_array.txt', 'w') as f:
            for error in self.error_array:
                f.write(f"{error}\n")

        # Write travelled_path to a text file
        with open('travelled_path.txt', 'w') as f:
            for pose in self.travelled_path:
                position = pose.pose.position
                f.write(f"{position.x} {position.y} {position.z}\n")

        # Write latest_received_plan to a text file
        with open('latest_received_plan.txt', 'w') as f:
            for pose in self.latest_recieved_plan:
                position = pose.pose.position
                f.write(f"{position.x} {position.y} {position.z}\n")

    def plan_to_numpy(self, plan):
        return np.array([[pose.pose.position.x, pose.pose.position.y] for pose in plan], dtype=np.float64)

    def calculate_path_length(self, plan):
        plan_np = self.plan_to_numpy(plan)
        length = np.sum(np.sqrt(np.sum(np.diff(plan_np, axis=0) ** 2, axis=1)))
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
