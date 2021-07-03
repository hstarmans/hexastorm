# -*- coding: utf-8 -*-
# ---
# jupyter:
#   jupytext:
#     text_representation:
#       extension: .py
#       format_name: light
#       format_version: '1.5'
#       jupytext_version: 1.5.2
#   kernelspec:
#     display_name: Python [conda env:dalry]
#     language: python
#     name: conda-env-dalry-py
# ---

# **Author:** Rik Starmans  
# **Abstract:** Splines, Bezier, B-splines, and NURBS (Non-Uniform Rational B-splines) curves are the common parametric techniques used for tool path [design](https://zero.sci-hub.se/2496/cb390d406cc077ef156deb76b34099af/desantiago-perez2013.pdf#lb0030).  For none of these an open source tool set is available,
# which can interoperate with laser scanning devices such as a prism scanner. Prism scanning would be a lot easier, if it can be combined with deposition or milling techniques.
# These techniques typically rely on G-code instruction. 
# The purpose of this notebook is to explore Bezier curves and provide an implementation on FPGA.
# A bezier curve is fitted through the tool path using least squares. This fitting approach is adopted from
# the [work](https://stackoverflow.com/questions/12643079/b%C3%A9zier-curve-fitting-with-scipy) of Tim Andrew Pastva "Bezier curve fitting".
# The idea is that these points are sent to a FPGA. The intermediate positions between points, i.e. motor positions, are calculated using [De Casteljau's algorithm](https://en.wikipedia.org/wiki/De_Casteljau%27s_algorithm).

# In the following, a line segment is defined between a start and end point. A function is defined along this segment which maps time to space.  
# This line is approximated with a Bezier curve of certain degree. To fit a third order bezier curve on a segment you need four points on this segment.

# +
from scipy.special import comb
import numpy as np
import matplotlib.pyplot as plt

fcn = np.log          # function
start, end = 0, 2.5   # start and end value
points = 81           # number of points
stepsize = 10         # step size of red dots in plot
degree = 4            # degree of bezier cuve


# -

# The following functions are needed to fit a bezier curve. This has all been adopted from [Stackoverflow](https://stackoverflow.com/questions/12643079/b%C3%A9zier-curve-fitting-with-scipy).

# +
def bpoly(n, t, k):
        """ Bernstein polynomial when a = 0 and b = 1.
        
        Parameters:
        n: degree of bernstein polynomial
        t: current value in time
        k: current order
        """
        return t ** k * (1 - t) ** (n - k) * comb(n, k)
    
def bmatrix(T, degree):
        """ Bernstein matrix for Bézier curves. 
        
        The input values T are a list of timestamps.
        The output values, aka y values, can be obtained by multiplying this matrix
        with the control points / or bezier parameters pf the same degree
        
        Parameters:
        T: a list of points
        """
        return np.matrix([[bpoly(degree, t, k) for k in range(degree + 1)] for t in T])

    
def get_bezier_parameters(X, Y, degree=3):
    """ Least square qbezier fit using penrose pseudoinverse.

    Parameters:

    X: array of x data.
    Y: array of y data. Y[0] is the y point for X[0].
    degree: degree of the Bézier curve. 2 for quadratic, 3 for cubic.

    Based on https://stackoverflow.com/questions/12643079/b%C3%A9zier-curve-fitting-with-scipy
    and probably on the 1998 thesis by Tim Andrew Pastva, "Bézier Curve Fitting".
    
    returns the control points
    """
    if degree < 1:
        raise ValueError('degree must be 1 or greater.')

    if len(X) != len(Y):
        raise ValueError('X and Y must be of the same length.')

    if len(X) < degree + 1:
        raise ValueError(f'There must be at least {degree + 1} points to '
                         f'determine the parameters of a degree {degree} curve. '
                         f'Got only {len(X)} points.')
    def least_square_fit(points, M):
        """ distance
        
        The distance taken is the norm. It is pointed out that this might not work as well for all possible curves.
        Norm measures the distance between points on the curve instead of between a point on one curve
        to the nearest point on the other curve.
        """
        M_ = np.linalg.pinv(M)
        return M_ * points
    T = np.linspace(0, 1, len(X))
    M = bmatrix(T, degree)
    points = np.array(list(zip(X, Y)))
    return least_square_fit(points, M)


# -

# The t parameter scales from 0 to 1 and is along the Bezier curve.
# The x value is the origin domain which varies from start to end

tPlot = np.linspace(0. ,1. , points)        # bezier varies is from [0,1]
xPlot = np.linspace(start+0.1, end, points) # 0.1 needed to avoid crash
tData = tPlot[start:points:stepsize]
xData = xPlot[start:points:stepsize]
control_points = get_bezier_parameters(xData, fcn(xData), degree=degree)
Bézier = bmatrix(tPlot, degree=degree).dot(control_points)
# error between calculated and actual points
error = fcn(Bézier[:,0]) - Bézier[:,1]
fig, ax = plt.subplots()
ax.plot(xPlot, fcn(xPlot),   'r-')                   # create smooth red line through inputs
ax.plot(xData, fcn(xData),    'ro', label='input')   # create red data points for inputs
ax.plot(Bézier[:,0],                                
        Bézier[:,1], 'k-', label='fit')
ax.plot(xPlot, 10.*error, 'b-', label='10*error')
ax.plot(control_points[:,0], control_points[:,1],    # control points always one more than degree
        'ko:', fillstyle='none')  
ax.legend()
fig.show()

# From the graph above it can be seen that the error is not uniform along the curve.  
# On Stackoverflow it is mentioned norm measures the distance between points on the curve instead of between  
# a point on one curve to the nearest point on the other curve. This issue is ignored for now.

# To implement this algo on FPGA the Bernstein polynomial needs to be evaluated.
# Let's write out the [de Casteljau's algorithm](https://en.wikipedia.org/wiki/De_Casteljau%27s_algorithm)

# +
import sympy
from sympy import abc
t = sympy.symbols('t')
def beta(i, j):
    """Casteljau algo of beta
    """
    if j==0:
        return sympy.symbols(f'b{i}')
    else:
        return beta(i, j-1)*(1-t)+beta(i+1, j-1)*t  

def casteljau(n):
    """Casteljau for degree is n
    """
    return beta(0,n)

sympy.collect(casteljau(2).expand(), ['b0', 'b1', 'b2'])
# -

# For second order, the calculation can be done in 4 clock cycles.
# -  t square is evaluated
# -  b0 is multiplied with polynomial which is written out so it only includes addition
# -  analogously b1 is evaluated and added to prior
# -  analogously b2 is evaluated and added to prior  
#
# This does not change for multiple motor axes; still 4 evaluations are needed.  
# The last 3 operations can be done in parallel for multiple axes.  
# For a Bezier curve of degree t, (t+1)+(t-1) calculations are needed.  
# With a clock cycle of 100 MHz, 3 motors and a degree of 3; the max update frequency is 16.7 MHz 

#

# This part is still under construction
# Let's wrap up the exploration by fitting a Bezier curve to a sine wave.

time = np.arange(0, 10, 0.1)
amplitude = np.sin(time)
plot.plot(time, amplitude)
plot.title('Sine wave')
plot.xlabel('Time')
plot.ylabel('Amplitude')
plot.grid(True, which='both')
plot.axhline(y=0, color='k')
plot.show()
x_points = np.arange(0, 10, 0.1)
y_points = np.sin(x_points)
print(f"The number of points equals {len(x_points)}")

binomial(15, 8
