import logging
from scipy.spatial.distance import directed_hausdorff
from shapely import LineString
from shapely import hausdorff_distance
import numpy as np

def test_compare_plans():
    plan1 = np.array([(0, 0), (1, 1), (-1, 0)])  # Example plan 1
    plan2 = np.array([(0, 0), (1, 1), (1, 0)])  # Example plan 2


    plan11 = LineString([(0, 0), (1, 1), (-1, 0)])  # Example plan 1
    plan22= LineString([(0, 0), (1, 1), (1, 0)])  # Example plan 2

    # Compute the expected distance using directed Hausdorff distance
    expected_distance = max(directed_hausdorff(plan1, plan2)[0],
                            directed_hausdorff(plan2, plan1)[0])
    logging.info("Directed haussdorff Distance from plan 1 to plan 2 : %f" % expected_distance)

    new_distance = (hausdorff_distance(plan11, plan22, densify = 0.5))
    logging.info(" haussdorff Distance from plan 1 to plan 2 : %f" % new_distance)


    logging.info("Area1 %f" % calculate_area(plan1))
    logging.info("Area2 %f" % calculate_area(plan2))


def calculate_area(plan):
    if plan is None:
        return False
    
    n = len(plan)
    area = 0.0
    
    for i in range(n):
        j = (i + 1) % n
        area += plan[i, 0] * plan[j, 1]
        area -= plan[j, 0] * plan[i, 1]  # Shoelace formula
    area = abs(area) / 2.0

    return area

def main():
    logging.basicConfig(level=logging.INFO)
    test_compare_plans()

if __name__ == '__main__':
    main()
