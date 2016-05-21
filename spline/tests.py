import spline


def test_spline_plot():
    cur_pt = [5, 7, 13]
    goal_pt = [1, 9, 0]
    next_pt = [15, 2, 11]

    spliner = spline.Spline(order=2)
    path = spliner.get_path(cur_pt, goal_pt, next_pt, n=30, bezier=True)

    for axis in range(len(path)):
        print(path[axis])

    import matplotlib.pyplot as plt
    from mpl_toolkits.mplot3d import Axes3D

    fig = plt.figure()
    ax = Axes3D(fig, elev=-150, azim=110)

    xs = [x for x, y, z in path]
    ys = [y for x, y, z in path]
    zs = [z for x, y, z in path]
    ax.plot(xs, ys, zs)
    plt.show()


def test_spline():

    cur_pt = [5, 7, 13]
    goal_pt = [1, 9, 0]
    next_pt = [15, 2, 11]

    spliner = spline.Spline(order=2)
    path = spliner.get_path(cur_pt, goal_pt, next_pt, n=30, bezier=True)

    for axis in range(len(path)):
        print(path[axis])


def test_optimal_n():
    center = [0.1, -0.475, 0.425, 1.2, -1.2, 1.2]

    current_point = center
    print("Current: {}".format(current_point))

    goal_point = list(center)
    goal_point[0] = center[0] + 0.2
    print("Goal: {}".format(goal_point))

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
    test_spline_plot()
