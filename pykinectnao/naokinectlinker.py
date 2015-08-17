__author__ = 'Angeall'
import joints
import kinecthandler
from math import *
import cmath
import utils


def convert_shoulder_left(pos, rot, must_filter=True):
    shoulder = kinecthandler.joints_map[joints.SHOULDER_LEFT]
    hand = kinecthandler.joints_map[joints.HAND_LEFT]
    elbow = kinecthandler.joints_map[joints.ELBOW_LEFT]
    roll_vector_1_x = pos[elbow][0] - pos[shoulder][0]
    roll_vector_1_y = pos[elbow][1] - pos[shoulder][1]
    norm1 = sqrt(roll_vector_1_x**2 + roll_vector_1_y**2)
    roll_vector_2_x = 0
    roll_vector_2_y = pos[elbow][1] - pos[shoulder][1]
    norm2 = sqrt(roll_vector_2_x**2 + roll_vector_2_y**2)
    roll_vector_1_x /= norm1
    roll_vector_1_y /= norm1
    roll_vector_2_x /= norm2
    roll_vector_2_y /= norm2
    theta= acos(roll_vector_1_x * roll_vector_2_x + roll_vector_1_y * roll_vector_2_y)*180./pi

    phi = rot[elbow][1]
    phi += 90
    phi = utils.valid_angle(phi)
    c = cmath.rect(1, phi *cmath.pi / 180.)
    c = c.conjugate()
    phi = round(cmath.phase(c)*180 / cmath.pi, 1)
    phi += 20
    phi = abs(phi)
    sign = 1
    if pos[elbow][1] > pos[shoulder][1]:
        sign = -1
    phi *= sign
    if phi<-115:
        phi = - phi - 90
    if 28 > phi > -28:
        theta *= 0.2
    tab = [theta, phi]
    if must_filter:
        tab = utils.value_filter(joints.ELBOW_LEFT, tab)
    return tab


def convert_shoulder_right(pos, rot, must_filter=True):
    shoulder = kinecthandler.joints_map[joints.SHOULDER_RIGHT]
    hand = kinecthandler.joints_map[joints.HAND_RIGHT]
    elbow = kinecthandler.joints_map[joints.ELBOW_RIGHT]
    roll_vector_1_x = pos[elbow][0] - pos[shoulder][0]
    roll_vector_1_y = pos[elbow][1] - pos[shoulder][1]
    norm1 = sqrt(roll_vector_1_x**2 + roll_vector_1_y**2)
    roll_vector_2_x = 0
    roll_vector_2_y = pos[elbow][1] - pos[shoulder][1]
    norm2 = sqrt(roll_vector_2_x**2 + roll_vector_2_y**2)
    roll_vector_1_x /= norm1
    roll_vector_1_y /= norm1
    roll_vector_2_x /= norm2
    roll_vector_2_y /= norm2
    theta= acos(roll_vector_1_x * roll_vector_2_x + roll_vector_1_y * roll_vector_2_y)*180./pi
    theta = -theta

    # print rot[elbow]
    phi = rot[elbow][1]
    phi += 90
    phi = utils.valid_angle(phi)
    c = cmath.rect(1, phi *cmath.pi / 180.)
    c = c.conjugate()
    phi = round(cmath.phase(c)*180 / cmath.pi, 1)
    phi += 10
    phi = abs(phi)
    sign = 1
    if pos[elbow][1] > pos[shoulder][1]:
        sign = -1
    phi *= sign
    if phi<-115:
        phi = - phi - 90
    if 28 > phi > -28:
        theta *= 0.2
    tab = [theta, phi]
    if must_filter:
        tab = utils.value_filter(joints.ELBOW_RIGHT, tab)
    return tab


def convert_elbow_right(pos, rot, must_filter=True):
    shoulder = kinecthandler.joints_map[joints.SHOULDER_RIGHT]
    hand = kinecthandler.joints_map[joints.HAND_RIGHT]
    elbow = kinecthandler.joints_map[joints.ELBOW_RIGHT]
    roll_vector_1_x = pos[elbow][0] - pos[hand][0]
    roll_vector_1_y = pos[elbow][1] - pos[hand][1]
    roll_vector_1_z = pos[elbow][2] - pos[hand][2]
    roll_vector_2_x = pos[shoulder][0] - pos[elbow][0]
    roll_vector_2_y = pos[shoulder][1] - pos[elbow][1]
    roll_vector_2_z = pos[shoulder][2] - pos[elbow][2]
    norm1 = sqrt(roll_vector_1_x**2 + roll_vector_1_y**2 + roll_vector_1_z**2)
    norm2 = sqrt(roll_vector_2_x**2 + roll_vector_2_y**2 + roll_vector_2_z**2)
    roll_vector_1_x /= norm1
    roll_vector_1_y /= norm1
    roll_vector_1_z /= norm1
    roll_vector_2_x /= norm2
    roll_vector_2_y /= norm2
    roll_vector_2_z /= norm2
    theta = acos(roll_vector_1_x * roll_vector_2_x + roll_vector_1_y * roll_vector_2_y +
                 roll_vector_1_z * roll_vector_2_z)*180./pi
    theta -= 10
    return theta


def convert_elbow_left(pos, rot, must_filter=True):
    shoulder = kinecthandler.joints_map[joints.SHOULDER_LEFT]
    hand = kinecthandler.joints_map[joints.HAND_LEFT]
    elbow = kinecthandler.joints_map[joints.ELBOW_LEFT]
    roll_vector_1_x = pos[elbow][0] - pos[hand][0]
    roll_vector_1_y = pos[elbow][1] - pos[hand][1]
    roll_vector_1_z = pos[elbow][2] - pos[hand][2]
    roll_vector_2_x = pos[shoulder][0] - pos[elbow][0]
    roll_vector_2_y = pos[shoulder][1] - pos[elbow][1]
    roll_vector_2_z = pos[shoulder][2] - pos[elbow][2]
    norm1 = sqrt(roll_vector_1_x**2 + roll_vector_1_y**2 + roll_vector_1_z**2)
    norm2 = sqrt(roll_vector_2_x**2 + roll_vector_2_y**2 + roll_vector_2_z**2)
    roll_vector_1_x /= norm1
    roll_vector_1_y /= norm1
    roll_vector_1_z /= norm1
    roll_vector_2_x /= norm2
    roll_vector_2_y /= norm2
    roll_vector_2_z /= norm2
    theta = acos(roll_vector_1_x * roll_vector_2_x + roll_vector_1_y * roll_vector_2_y +
                 roll_vector_1_z * roll_vector_2_z)*180./pi
    theta *= -1
    theta += 10
    return theta



# def convert_motors(results, device, sensor, avatar, must_filter=True):
#     convert_tab = modules[avatar].motors_converters[sensor]
#     if len(convert_tab) > 0:
#         motors_needed = modules[avatar].motors_needed
#         for joint in motors_needed.keys():
#             if joint in convert_tab.keys():
#                 tab = results[kinecthandler.joints_map[joints.ELBOW_RIGHT]]
#                 if joint in last_movements.keys() and len(last_movements[joint]) != 0:
#                     # if joint not tracked properly
#                     if device.positions[joint].TrackingState == PyKinectV2.TrackingState_NotTracked or \
#                                     device.positions[joint].TrackingState == PyKinectV2.TrackingState_Inferred:
#                         results[motors_needed[joint][0]] = last_movements[joint]
#                         continue
#                 for i in range(3):
#                     if motors_needed[joint][1][i]:
#                         tab[i] = convert_tab[joint][i][0] * tab[i]
#                         tab[i] = convert_tab[joint][i][1] + tab[i]
#                         tab[i] = valid_angle(tab[i])
#                         # If the X axis is inverted
#                         if convert_tab[joint][i][2] == "-":
#                             c = cmath.rect(1, tab[i] / 180. * cmath.pi)
#                             c = complex(-1 * c.real, c.imag)
#                             tab[i] = round(cmath.phase(c) / cmath.pi * 180, 1)
#                         # If the Y axis is inverted
#                         if convert_tab[joint][i][3] == "-":
#                             c = cmath.rect(1, tab[i] / 180. * cmath.pi)
#                             c = c.conjugate()
#                             tab[i] = round(cmath.phase(c) / cmath.pi * 180, 1)
#                         if must_filter:
#                             if len(smoothing_dict[joint][i]) == FILTER_SIZE:
#                                 smoothing_dict[joint][i].popleft()
#                             smoothing_dict[joint][i].append(tab[i])
#                             tab[i] = utils.holt_winters_second_order_ewma \
#                                 (smoothing_dict[joint][i], FILTER_SPAN, FILTER_BETA)[-1]
#                 last_movements[joint] = tab
#     return results
