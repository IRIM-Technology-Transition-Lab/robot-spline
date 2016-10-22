Robot Spline
============

Spline Path Generation Library (especially for Robotics)

.. image:: https://travis-ci.org/IRIM-Technology-Transition-Lab/robot-spline.svg?branch=master
    :target: https://travis-ci.org/IRIM-Technology-Transition-Lab/robot-spline
    :alt: Travis CI build status

.. image:: https://badge.fury.io/py/robot-spline.svg
    :target: https://badge.fury.io/py/robot-spline

Description
-----------

This package is used to generate spline paths (simple or bezier), which a can then be submitted to a robot to follow.

Installation
------------
::

    pip install robot-spline

Example
-------- 

You can generate a spline path quite easily::

    # Import the package
    import spline

    # Define 3 points, since we need a minimum of 3 points to generate a spline curve.
    cur_pt = [5, 7, 13]  # the robot's current position
    next_pt = [15, 2, 11]  # the robot's next position
    goal_pt = [1, 9, 0]  # The robot's final goal position

    # Create our spliner object.
    spliner = spline.Spline(order=2)

    # Given the 3 points, generate the spline path as a bezier curve
    path = spliner.get_path(cur_pt, goal_pt, next_pt, n=30, bezier=True)
