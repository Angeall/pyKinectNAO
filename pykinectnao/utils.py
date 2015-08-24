import numpy as np
from math import *
import pykinectnao.kinecthandler as kinecthandler
from collections import deque


#
# All copyright to Connor Johnson
# Copied from http://connor-johnson.com/2014/02/01/smoothing-with-exponentially-weighted-moving-averages/
#
def holt_winters_second_order_ewma(x, span, beta):
    N = len(x)
    alpha = 2.0 / ( 1 + span )
    s = np.zeros((N, ))
    b = np.zeros((N, ))
    s[0] = x[0]
    for i in range( 1, N ):
        s[i] = alpha * x[i] + (1 - alpha)*(s[i-1] + b[i-1])
        b[i] = beta * (s[i] - s[i-1]) + (1 - beta) * b[i-1]
    return s


last_movements = {}
smoothing_dict = {}

FILTER_SPAN = 7
FILTER_BETA = 0.3
FILTER_SIZE = 15

for joint in kinecthandler.joints_map.keys():
        smoothing_dict[joint] = [deque(), deque(), deque()]
        last_movements[joint] = []


def value_filter(joint, tab):
    for i in range(len(tab)):
        if len(smoothing_dict[joint][i]) == FILTER_SIZE:
            smoothing_dict[joint][i].popleft()
        smoothing_dict[joint][i].append(tab[i])
        tab[i] = holt_winters_second_order_ewma \
            (smoothing_dict[joint][i], FILTER_SPAN, FILTER_BETA)[-1]
        last_movements[joint] = tab
    return tab


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


def valid_angle(value):
    if value > 180:
        value = -180 + (value - 180)
        return valid_angle(value)
    if value < -180:
        value = 180 + (value + 180)
        return valid_angle(value)
    return value


# coord_tab: tab of coordinates to rotate, given the angles tab
# angles : the angles to rotate (tab must have the same length than order)
# order: the rotation order, examples : xyx. or xy, etc...
# return a tab with all the coord rotated
def rotate(coord_tab, angles, order="xyz"):
    m = []
    for i in range(len(order)):
        angle = angles[i]
        if order[i] == 'x':
            m.append(np.matrix([[1, 0, 0], [0, cos(angle), -sin(angle)], [0, sin(angle), cos(angle)]]))
        elif order[i] == 'y':
            m.append(np.matrix([[cos(angle), 0, sin(angle)], [0, 1, 0], [-sin(angle), 0, cos(angle)]]))
        elif order[i] == 'z':
            m.append(np.matrix([[cos(angle), -sin(angle), 0], [sin(angle), cos(angle), 0], [0, 0, 1]]))
    matrix = reduce(lambda a, b: a * b, m)
    new_coord_tab = []
    for coord in coord_tab:
        [x, y, z] = coord
        vector = np.matrix([[x], [y], [z]])
        [x, y, z] = (matrix*vector).getA()
        new_coord_tab.append([x[0], y[0], z[0]])
    return new_coord_tab


def get_vector(final, start, transform=None):
    x = final[0]-start[0]
    y = final[1]-start[1]
    z = final[2]-start[2]
    if transform is None:
        return [x, y, z]
    else:
        temp = transform*np.matrix([[x], [y], [z]])
        return np.transpose(temp)


def normalized_cross(vect1, vect2):
    cross = np.cross(vect1, vect2)
    return normalize(cross)


def normalized_dot(vect1, vect2):
    vect1 = normalize(vect1)
    vect2 = normalize(vect2)
    dot = np.dot(vect1, vect2)
    return dot


def normalize(vect):
    return vect/np.linalg.norm(vect)


def angle_between(vector1, vector2):
    norm1 = np.linalg.norm(vector1)
    norm2 = np.linalg.norm(vector2)
    vector1 = map(lambda x: x/norm1, vector1)
    vector2 = map(lambda x: x/norm2, vector2)
    return acos(vector1[0] * vector2[0] + vector1[1] * vector2[1] + vector1[2] * vector2[2])*180./pi
