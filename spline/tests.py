import spline

def test_spline():
    # Define 3 points
    cur_pt = [5, 7, 13]
    goal_pt = [1, 9, 0]
    next_pt = [15, 2, 11]

    # Create our spliner object
    spliner = spline.Spline(order=2)

    # Given the 3 points, generate the spline path as a bezier curve
    path = spliner.get_path(cur_pt, goal_pt, next_pt, n=30, bezier=True)

    # Print the points in the path
    for axis in range(len(path)):
        print(path[axis])


def test_optimal_n():
    center = [0.1, -0.475, 0.425, 1.2, -1.2, 1.2]

    current_point = center
    print("Current: {}".format(current_point))

    # Create a point a bit to the right of center
    goal_point = list(center)
    goal_point[0] = center[0] + 0.2
    print("Goal: {}".format(goal_point))

    # Create a point to the top of goal point
    next_point = list(goal_point)
    next_point[2] = goal_point[2] + 0.1
    print("Next: {}".format(next_point))

    spliner = spline.Spline(order=2)
    path = spliner.get_path(current_point[:3], goal_point[:3], next_point[:3], bezier=True)

    print("Optimal number of points")
    assert(len(path) == 3261)
    print(len(path))


def main():
    test_spline()
    test_optimal_n()
