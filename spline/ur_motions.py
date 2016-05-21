"""
Sample program showing how to interface spline-path library with UR5 control
"""

from __future__ import print_function

import ur_cb2.cb2_robot as cb2_robot
import time
import random
import socket
from spline import Spline

HOST = "192.168.1.100"
PORT = 30003


def center_robot(robot, pose):
    # angle_start = [90, -95, 90, 0, 90, 90]
    # angle_start = map(cb2_robot.cb2_send.deg_2_rad, angle_start)
    angle_start = [-01.5503467, -02.0665188, -01.3330899, +00.1346348, +01.6603317, -01.5165228]
    robot.add_goal(cb2_robot.Goal(angle_start, False, 'joint'))
    robot.add_goal(cb2_robot.Goal(pose, cartesian=True, move_type='linear'))

    while not robot.goals.empty():
        robot.move_on_stop()
        #time.sleep(3)


def left2right_motion(robot, left_point, center, right_point, multiplier):
    # Move left and then move back with multiplier 2
    robot.add_goal(cb2_robot.Goal(left_point, True, 'linear', radius=0.0))

    while not robot.goals.empty():
        robot.move_on_stop()
    
    print("moved left!")

    robot.add_goal(cb2_robot.Goal(center, True, 'linear', radius=0.0))
    
    spliner = Spline(order=2)
    path = spliner.get_path(left_point[:3], center[:3], right_point[:3], n=100, bezier=False)
    #print(path)

    new = center
    new[2] = new[2] + 0.1
    robot.add_goal(cb2_robot.Goal(new, True, 'linear', radius=0.0))

    while not robot.goals.empty():
        robot.move_on_error(multiplier=multiplier)

    print("moved to the right with spline")


def curve_motion(robot, current, goal, new):
    spliner = Spline(order=2)
    path = spliner.get_path(current[:3], goal[:3], new[:3], n=30, bezier=True)
    print(path)

    for p in path:
        robot.add_goal(cb2_robot.Goal([p[0], p[1], p[2], 1.2, -1.2, 1.2], True, 'process'))
    
    while not robot.goals.empty():
        robot.move_on_error(multiplier=2.5)


def semi_track(robot, current):
    """
    Start at the center and do a complete circuit motion
    Args:
        robot:
        current:

    Returns:

    """

    spliner = Spline(order=2)

    # A point little to the right of the center
    point1 = list(current)
    point1[0] = current[0] + 0.2

    # a point above the previous point.
    # Should form curve path to this using previous point as a control point
    point2 = list(point1)
    point2[2] = point1[2] + 0.1

    path = spliner.get_path(current[:3], point1[:3], point2[:3], n=30, bezier=True)
        #print(path)
    for p in path:
        robot.add_goal(cb2_robot.Goal([p[0], p[1], p[2], 1.2, -1.2, 1.2], True, 'process', radius=0.001))

    # Continue moving slightly up
    point3 = list(point2)
    point3[2] += 0.01

    robot.add_goal(cb2_robot.Goal([point3[0], point3[1], point3[2], 1.2, -1.2, 1.2], True, 'process', radius=0.001))

    # A little more up from point2
    point4 = list(point2)
    point4[2] += 0.05

    # A point to the left of point 4.
    # Should have a curve path from 3 to 5 via control point 4
    point5 = list(point4)
    point5[0] -= 0.2

    path = spliner.get_path(point3[:3], point4[:3], point5[:3], n=30, bezier=True)
    #print(path)
    for p in path:
        robot.add_goal(cb2_robot.Goal([p[0], p[1], p[2], 1.2, -1.2, 1.2], True, 'process', radius=0.001))

    while not robot.goals.empty():
        robot.move_on_error(multiplier=2.5)


def circuit(robot, start):
    spliner = Spline(order=2)

    # A point little to the right of the start
    point1 = list(start)
    point1[0] += 0.2

    # a point above the previous point.
    # Should form curve path to this using previous point as a control point
    point2 = list(point1)
    point2[2] = point1[2] + 0.1

    path = spliner.get_path(start[:3], point1[:3], point2[:3], n=50, bezier=True)
    for p in path:
        robot.add_goal(cb2_robot.Goal([p[0], p[1], p[2], 1.2, -1.2, 1.2], True, 'process', radius=0.001))

    # Continue moving slightly up
    point3 = list(point2)
    point3[2] += 0.01

    robot.add_goal(cb2_robot.Goal([point3[0], point3[1], point3[2], 1.2, -1.2, 1.2], True, 'process', radius=0.001))

    # A little more up from point3
    point4 = list(point2)
    point4[2] += 0.05

    # A point to the left of point 4.
    # Should have a curve path from 3 to 5 via control point 4
    point5 = list(point4)
    point5[0] -= 0.2

    path = spliner.get_path(point3[:3], point4[:3], point5[:3], n=50, bezier=True)
    for p in path:
        robot.add_goal(cb2_robot.Goal([p[0], p[1], p[2], 1.2, -1.2, 1.2], True, 'process', radius=0.001))

    # Point more to the left of point 5 and slightly below
    point6 = list(point5)
    point6[0] -= 0.3
    point6[2] -= 0.1

    # point down to point 6
    #point7 = list(point6)
    #point7[2] -= 0.1

    # move back to the start point in a curve
    path = spliner.get_path(point5[:3], point6[:3], start[:3], n=50, bezier=True)
    for p in path[:-5]:
        robot.add_goal(cb2_robot.Goal([p[0], p[1], p[2], 1.2, -1.2, 1.2], True, 'process', radius=0.001))

    while not robot.goals.empty():
        robot.move_on_error(multiplier=2.0)

    robot.add_goal(cb2_robot.Goal([path[-1][0], path[-1][1], path[-1][2], 1.2, -1.2, 1.2], True, 'process', radius=0.001))

    while not robot.goals.empty():
        robot.move_on_error(multiplier=1.0)

    time.sleep(2)


with cb2_robot.URRobot(HOST, PORT, error=0.01, verbose=True) as robot:
    robot.receiver.verbose = False

    center = [0.1, -0.475, 0.425, 1.2, -1.2, 1.2]
    center_robot(robot, center)

    current_point = center
    print("Current: {}".format(current_point))

    goal_point = list(center)
    goal_point[0] = center[0] + 0.2
    print("Goal: {}".format(goal_point))

    next_point = list(goal_point)
    next_point[2] = goal_point[2] + 0.1
    print("Next: {}".format(next_point))

    #curve_motion(robot, current_point, goal_point, next_point)

    """
    left = [-0.2, -0.475, 0.425, 1.2, -1.2, 1.2]
    next_point = list(center)
    next_point[0] = center[0] + 0.1
    #next_point[2] = center[2] + 0.1
    print("Next: {}".format(next_point))

    #left2right_motion(robot, left, center, next_point, 1.2)
    """

    #semi_track(robot, center)

    circuit(robot, center)
