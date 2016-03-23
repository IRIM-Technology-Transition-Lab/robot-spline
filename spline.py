import numpy as np
import scipy.interpolate as sci
from scipy.misc import comb


class Spline:

    def __init__(self, smoothness=3, order=3, nest=-1):
        self.smoothness = smoothness
        self.order = order
        self.nest = nest    # Estimate of number of knots needed

    @staticmethod
    def bernstein_polynomial(i, n, t):
        """
        The Bernstein polynomial of n, i as a function of t
        Args:
            self: The Spline object
            i: Counter for the current set of points that is being interpolated.
            n: The number of points.
            t: The linear space interval.

        Returns:
            The value of the Bernstein Polynomial to be used in Bezier Path creation.
        """
        return comb(n, i) * (t**(n-i)) * (1 - t)**i

    def bezier_curve(self, points, n=1000):
        """
        Given a set of control points, return the bezier curve defined by the control points.
        See http://processingjs.nihongoresources.com/bezierinfo/

        Args:
            points: A list of lists that should have atleast 3 points, the current point,
            the current goal and the next point.
            These are the control points needed for the bezier curve.
            n:

        Returns:
            3 lists for x, y and z values of the points in the path.
        """
        n_points = len(points)
        xPoints = np.array([p[0] for p in points])
        yPoints = np.array([p[1] for p in points])
        zPoints = np.array([p[2] for p in points])

        t = np.linspace(0.0, 1.0, n)

        polynomial_array = np.array([self.bernstein_polynomial(i, n_points-1, t) for i in range(0, n_points)])

        # print(polynomial_array.shape)
        xvals = np.dot(xPoints, polynomial_array)
        yvals = np.dot(yPoints, polynomial_array)
        zvals = np.dot(zPoints, polynomial_array)

        return xvals, yvals, zvals

    def simple_curve(self, points, n):
        """
        Get a spline curve that passes through all the points specified in points.

        Args:
            points: A list of lists that should have atleast 3 points, the current point,
            the current goal and the next point.
            n: The number of points to interpolate.

        Returns:
            3 lists for x, y and z values of the points in the path.
        """
        # get the knot points
        tckp, u = sci.splprep([points[0], points[1], points[2]], s=self.smoothness, k=self.order, nest=self.nest)
        
        # evaluate the spline, including interpolated points
        xnew, ynew, znew = sci.splev(np.linspace(0, 1, n), tckp)
        return xnew, ynew, znew

    def get_path(self, cur_pt, goal_pt, next_pt, freq=65, velocity=0.002, n=None, bezier=True):
        """
        Args:
            cur_pt: The current point of the robot.
            goal_pt: The current goal to which the robot is trying to move.
            next_pt: The point the robot will move to next after the goal point.
            freq: The frequency at which the robot can handle new goal points.
            velocity: The velocity with which the robot will be moved (in m/s).
            n: The number of points in the path to be interpolated (optional).
            bezier: Flag to specify if the path should be a bezier curve or go through each point.

        Returns:
            A list of goal points (x, y, z) for each point in the path.
        """
        path = []
        for i in range(len(cur_pt)):
            path.append([cur_pt[i], goal_pt[i], next_pt[i]])

        if n is None:
            points = self.get_path(cur_pt, goal_pt, next_pt, n=500)
            n = self.get_optimal_n(points, velocity, freq)

        if bezier:
            xnew, ynew, znew = self.bezier_curve(path, n)            
        else:
            xnew, ynew, znew = self.simple_curve(path, n)

        points = [p for p in zip(xnew, ynew, znew)]

        return points

    @staticmethod
    def get_optimal_n(points, v, f):
        """
        Return the optimal number of points to interpolate on based on the speed of the robot and the
        frequency at which it can accept new goal points.

        Args:
            points: A dense set of points interpolated with an arbitrarily high n.
            v: The velocity of the robot.
            f: The frequency at which the robot can receive new goal points.

        Returns:

        """
        l = 0
        for i in range(len(points)-1):
            # Euclidean Distance between each succesive pair of points
            d = sum([(points[i][j] - points[i+1][j])**2 for j in range(len(points[i]))])
            l += np.sqrt(d)

        n = (l / v) / f

        return int(n)
