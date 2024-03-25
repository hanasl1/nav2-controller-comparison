import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from nav_msgs.msg import Odometry
from scipy.spatial.distance import directed_hausdorff
import math
import numpy as np
from std_msgs.msg import Bool


class KpiGenerator(Node):
    plan_recieved = False
    travelled_path = []
    latest_recieved_plan = []
    goal_reached  = False
    def __init__(self):

        super().__init__("kpi_generator")
        self.plan_recieved = False
        self.plan_subscription = self.create_subscription(Path, 'plan', self.plan_callback, 1)
        self.odometry_subscription = self.create_subscription(Odometry, "odometry/global", self.odometry_callback, 1)
        self.result_subscription = self.create_subscription(Bool, 'navigation_result', self.result_callback, 10)


    def plan_callback(self, msg):
        if not self.plan_recieved:
            self.latest_recieved_plan = msg.poses
            self.plan_recieved = True
        else: 
            pass


    def odometry_callback(self, msg):
        position = msg.pose.pose.position
        if not self.goal_reached:
            self.travelled_path.append(msg.pose)

    def result_callback(self, msg):
        if msg.data and not self.goal_reached:
            self.get_logger().info("NAVIGATION RESULT {}".format(msg.data))
            self.goal_reached = True
            self.generate_kpi()

            

    def generate_kpi(self):
        self.get_logger().info("GENERATING KPI")

        self.compare_plans(self.travelled_path, self.latest_recieved_plan)
        self.calculate_area_difference(self.travelled_path, self.latest_recieved_plan)
        

    def plan_to_numpy(self, plan):
        return np.array([[pose.pose.position.x, pose.pose.position.y] for pose in plan], dtype=np.float64)

    def calculate_area_difference(self, plan1, plan2):
        pass


    def compare_plans(self, plan1, plan2):
        if plan1 is None or plan2 is None or not plan1 or not plan2:
            return False
        
        plan1_np = self.plan_to_numpy(plan1)
        plan2_np = self.plan_to_numpy(plan2)

        distance1_to_2 = directed_hausdorff(plan1_np, plan2_np)[0]
        distance2_to_1 = directed_hausdorff(plan2_np, plan1_np)[0]

        
        self.get_logger().info("Distance from plan 1 to plan 2 x: %f" % distance1_to_2)
        self.get_logger().info("Distance from plan 1 to plan 2 x: %f" % distance2_to_1)


        return (distance1_to_2 + distance2_to_1) /2 #??





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
