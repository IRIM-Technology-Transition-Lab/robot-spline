import numpy as np
import scipy.interpolate as sci
from scipy.misc import comb


class Spline:
    """The Spline class which abstracts the spline path generation."""

    def __init__(self, smoothness=3, order=3, nest=-1):
        """
        Initialize a Spline object to help create the spline path.

        Args:
            self: The spline object.
            smoothness: A smoothing condition. The amount of smoothness is determined by satisfying the conditions: sum((w * (y - g))**2,axis=0) <= s, where g(x) is the smoothed interpolation of (x,y). The user can use s to control the trade-off between closeness and smoothness of fit. Larger s means more smoothing while smaller values of s indicate less smoothing.
            order: The degree of the spline path. Cubic splines are recommended. Even values of k should be avoided especially with a small s-value. 1 <= k <= 5, default is 3.
            nest: An over-estimate of the total number of knots of the spline to help in determining the storage space.
        """
        self.smoothness = smoothness
        self.order = order
        self.nest = nest    # Estimate of number of knots needed

    @staticmethod
    def bernstein_polynomial(i, n, t):
        """
        The Bernstein polynomial of n, i as a function of t.

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
        Given a set of control points, return the bezier curve defined by the control points. See http://processingjs.nihongoresources.com/bezierinfo/ for more information.

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

        print(polynomial_array.shape)
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

    def get_path(self, cur_pt, goal_pt, next_pt, freq=65, velocity=0.05, n=None, bezier=True):
        """
        Return a list of n points which correspond to a spline path from `cur_pt` to `goal_pt` via `next_pt`.

        Args:
            cur_pt: The current point of the robot.
            goal_pt: The current goal to which the robot is trying to move.
            next_pt: The point the robot will move to next after the goal point.
            freq: The frequency at which the robot can handle new goal points.
            velocity: The velocity with which the robot will be moved.
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
        Retrieve the best number of points given the velocity you want the robot to move at and the frequency which it can handle.

        Args:
            points (): The list of points which are received.
            v (): The velocity at which the robot will move.
            f (): The frequency at which the robot can accept points.

        Returns:
            The optimal number of points `n` for which to create the spline curve.
        """
        l = 0

        for i in range(len(points)-1):
            # Calculate the euclidean distance
            eucl_d = np.sum(np.asmatrix(points[i]) - np.asmatrix(points[i+1]))
            l += np.sqrt(eucl_d**2)

        print("L: {0}, v: {1}, f: {2}".format(l, v, f))
        n = (l / v) * f
        return n


if __name__ == "__main__":
    # Run the test suite
    import tests
    tests.main()
