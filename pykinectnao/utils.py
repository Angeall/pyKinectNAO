import numpy as np
from math import *


'''
 All copyright to Connor Johnson
 Copied from http://connor-johnson.com/2014/02/01/smoothing-with-exponentially-weighted-moving-averages/
'''
def holt_winters_second_order_ewma( x, span, beta ):
    N = len(x)
    alpha = 2.0 / ( 1 + span )
    s = np.zeros(( N, ))
    b = np.zeros(( N, ))
    s[0] = x[0]
    for i in range( 1, N ):
        s[i] = alpha * x[i] + ( 1 - alpha )*( s[i-1] + b[i-1] )
        b[i] = beta * ( s[i] - s[i-1] ) + ( 1 - beta ) * b[i-1]
    return s


def quat_to_axisangle(q):
    angle = 2.0*acos(q[0])
    x = q[1]/(sqrt(1-(q[0]*q[0])))
    y = q[2]/(sqrt(1-(q[0]*q[0])))
    z = q[3]/(sqrt(1-(q[0]*q[0])))
    return [angle/pi*180.0, x, y, z]

def cart_to_spher(vector):
    r = np.linalg.norm(vector)
    unit = map(lambda x: x/r, vector)
    theta = acos(unit[2])/pi*180.
    phi = atan2(unit[1], unit[0])/pi*180.
    return [r, theta, phi]