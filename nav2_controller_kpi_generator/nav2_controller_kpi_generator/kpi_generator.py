import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from nav_msgs.msg import Odometry
from scipy.spatial.distance import directed_hausdorff
import math
import numpy as np


class KpiGenerator(Node):
    travelled_path = []
    latest_recieved_plan = []
    def __init__(self):
        super().__init__("kpi_generator")
        self.plan_subscription = self.create_subscription(Path, 'plan', self.plan_callback, 1)
        self.odometry_subscription = self.create_subscription(Odometry, "odometry/global", self.odometry_callback, 1)

    def plan_callback(self, msg):

       if not self.compare_plans(self.latest_recieved_plan, msg.poses, 0.35):
        self.latest_recieved_plan = msg.poses


    def odometry_callback(self, msg):
        position = msg.pose.pose.position
        self.travelled_path.append(position)
        if self.latest_recieved_plan:
            latest_position = self.latest_recieved_plan[-1].pose.position
            

    def plan_to_numpy(self, plan):
        return np.array([[pose.pose.position.x, pose.pose.position.y] for pose in plan], dtype=np.float64)


    def compare_plans(self, plan1, plan2, threshold):
        if plan1 is None or plan2 is None or not plan1 or not plan2:
            return False
        
        plan1_np = self.plan_to_numpy(plan1)
        plan2_np = self.plan_to_numpy(plan2)

        # Compute the directed Hausdorff distance between the two plans
        distance1_to_2 = directed_hausdorff(plan1_np, plan2_np)[0]
        distance2_to_1 = directed_hausdorff(plan2_np, plan1_np)[0]

        # Log the distances for debugging purposes
        self.get_logger().info("Distance from plan 1 to plan 2 x: %f" % distance1_to_2)
        self.get_logger().info("Distance from plan 1 to plan 2 y: %f" % directed_hausdorff(plan1_np, plan2_np)[1])
        self.get_logger().info("Distance from plan 1 to plan 2 y: %f" % directed_hausdorff(plan1_np, plan2_np)[2])



        

        # If both distances are below the threshold, consider the plans similar
        return distance1_to_2 < threshold and distance2_to_1 < threshold




    def interpolate_plan(plan, num_intermediate_points):
        interpolated_plan = []
        for i in range(length(plan) - 1):
            start_point = plan[i]
            end_point = plan[i + 1]

            # Calculate step size along each coordinate axis
            step_x = (end_point.x - start_point.x) / (num_intermediate_points + 1)
            step_y = (end_point.y - start_point.y) / (num_intermediate_points + 1)
            step_z = (end_point.z - start_point.z) / (num_intermediate_points + 1)

            # Interpolate intermediate points
            for j in range(1, num_intermediate_points + 1):
                interpolated_point = Point()
                interpolated_point.x = start_point.x + step_x * j
                interpolated_point.y = start_point.y + step_y * j
                interpolated_point.z = start_point.z + step_z * j
                interpolated_plan.append(interpolated_point)

        return interpolated_plan




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
