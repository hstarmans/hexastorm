# -*- coding: utf-8 -*-
# ---
# jupyter:
#   jupytext:
#     text_representation:
#       extension: .py
#       format_name: light
#       format_version: '1.5'
#       jupytext_version: 1.13.0
#   kernelspec:
#     display_name: Python 3
#     language: python
#     name: python3
# ---

# **Author:** Rik Starmans  
# **Title:** Motion control
# **Abstract:**  Motion control and its implemenation on a FPGA is discussed

# # Overview
# A robot updates its position multiple times during a move. In practice it is impossible to sent over each position indivually due to data transfer limits.
# The controller solves this via interpolation.  The starting and end-conditions, i.e. boundary conditions, are given for a move.
# Possible boundary conditions are not only position but can be acceleration, jerk or number of steps in a move. It all depends on the mathematical interpretation chosen.
# Instructions for a 3D printer are created as follows. A design is made in a CAD program. The design is translated to instructions.
# Most programs like Cura and slicer store the final instructions as [G-code](https://en.wikipedia.org/wiki/G-code).
# I extensively looked at two mathematical solutions; [splines](https://en.wikipedia.org/wiki/Spline_(mathematics)) and [Bezier curves](https://en.wikipedia.org/wiki/B%C3%A9zier_curve).
# Other options are B-splines and NURBS (Non-Uniform Rational B-splines) see [article](https://zero.sci-hub.se/2496/cb390d406cc077ef156deb76b34099af/desantiago-perez2013.pdf#lb0030).
#
# # Limitations on FPGA
# Not all mathematical operations can be done on the fly on a FPGA. For instance, multiplication and floating point arithmetic do not exist on default.
# A core for floating point arithmetic is available for nmigen, https://pypi.org/project/libresoc-ieee754fpu/. 
# I opted for [fixed point arithmetic](https://vha3.github.io/FixedPoint/FixedPoint.html) as this requires less resources.
#
# # Splines
# This is desribed accurately in movement.py in the class defintion of Polynomal.
#
# # Beziers
# It is possible to multiply 32 bit wide numbers on the UP5K; see test/old/dsp/multiplation.py.
# Beziers are not implemented yet. I would again assume a maximum of 10_000 ticks per segment and a sampling speed of 1 MHz.
# The number of coefficients you need to sent over to fpga is order of Bezier multipled by the number of axes.
# The start and end point can't be too far away; the speed is limited by the Nyquist sample criterion.
# The minimum distance should at least be one step.
# I would assume you start at position zero and sent over the step displacement.
#
# The problem is currently have is in a segment time goes from 0 to 1, i actually want it to go from 0 to 10_000.
# I can't use a divider on the FPGA. Multiplcations like 0.01x0.01 have a high risk for underflow.
# This problem has not been solved and is omitted for now.

# The purpose of this notebook is to explore Bezier curves. It does not reach a conclusion, it is mainly exploritory.
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
plt.plot(time, amplitude)
plt.title('Sine wave')
plt.xlabel('Time')
plt.ylabel('Amplitude')
plt.grid(True, which='both')
plt.axhline(y=0, color='k')
plt.show()
x_points = np.arange(0, 10, 0.1)
y_points = np.sin(x_points)
print(f"The number of points equals {len(x_points)}")

# # Final Notes
#
# Main challenge is not DSP but that you don't have floating point arithmetic on the FPGA.
# You work around this using bitshifts but, this gets more complicated when the order increases.
# There are core for floating point arithmetic, https://pypi.org/project/libresoc-ieee754fpu/.
# I am not sure how to use these and mailed upstream for support.


