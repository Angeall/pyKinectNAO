__author__ = 'Angeall'
from time import sleep
import cmath
from math import *
from collections import deque

from pykinect2 import PyKinectV2

import pykinectnao.kinecthandler as kinecthandler
import pykinectnao.naocommander as naocommander
import pykinectnao.utils as utils
import pykinectnao.joints as joints
import cmath

robotIP = "192.168.2.24"
PORT = 9559

FILTER_SPAN = 7
FILTER_BETA = 0.3
FILTER_SIZE = 15

last_movements = {}
smoothing_dict = {}

nb_of_body = 1

modules = {"kinecthandler": kinecthandler,
           "naocommander": naocommander}

objects = {"kinecthandler": kinecthandler.KinectHandler,
           "naocommander": naocommander.NAOCommander}


def kinect_test(kinect_h, nao_c):
    # first = True
    for joint in kinecthandler.joints_map.keys():
        smoothing_dict[joint] = [deque(), deque(), deque()]
        last_movements[joint] = []
    j = 0
    while True:
        res = kinect_h.get_movement(nb_of_body)
        if res == kinecthandler.NO_DATA:
            # first = True
            continue
        # if first == True:
        #     first = False
        #     sleep(5)
        #     nao_c.go_to_zero()
        for i in range(nb_of_body):
            # res[i] = convert_motors(res[i], kinect_h, "kinecthandler", "naocommander")
            #
            converted = convert_shoulder_right(res[i][0], res[i][1])
            # print converted
            nao_c.user_right_arm_articular(shoulder_pitch=converted[1],shoulder_roll=converted[0],pfractionmaxspeed=0.7)
            converted = convert_shoulder_left(res[i][0], res[i][1])
            nao_c.user_left_arm_articular(shoulder_pitch=converted[1],shoulder_roll=converted[0],pfractionmaxspeed=0.7)
            # print res[i][0][kinecthandler.joints_map[joints.ELBOW_RIGHT]]
            # sleep(0.5)

            # print res[i]
            # sleep(0.035)
            # if j % 5 == 0:
            #     if j == 0:
            #         print "-"*10, "BRAS LE LONG DU CORPS", "-"*10
            #     if j == 5:
            #         print "-"*10, "BRAS EN L'AIR", "-"*10
            #     if j == 10:
            #         print "-"*10, "BRAS HORIZONTAL DEVANT", "-"*10
            #     if j == 15:
            #         print "-"*10, "BRAS HORIZONTAL LATERAL", "-"*10
            #     if j == 21:
            #         kinect_h.device.close()
            #         break
            #     sleep(3)
            # else:
            #     print res[i][0]
            #     res[i] = convert_motors(res[i], "kinecthandler", "naocommander")
            #     print res[i][0]
            #     sleep(1)
            # j += 1


def valid_angle(value):
    if value > 180:
        value = -180 + (value - 180)
        return valid_angle(value)
    if value < -180:
        value = 180 + (value + 180)
        return valid_angle(value)
    return value


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


def value_filter(joint, tab):
    for i in range(len(tab)):
        if len(smoothing_dict[joint][i]) == FILTER_SIZE:
            smoothing_dict[joint][i].popleft()
        smoothing_dict[joint][i].append(tab[i])
        tab[i] = utils.holt_winters_second_order_ewma \
            (smoothing_dict[joint][i], FILTER_SPAN, FILTER_BETA)[-1]
        last_movements[joint] = tab
    return tab


def kinect_value_test(kinect_h):
    j = 0
    while True:
        res = kinect_h.get_movement(nb_of_body)
        if res == kinecthandler.NO_DATA:
            continue
        for i in range(nb_of_body):
            # print "Body no", i
            # print res[i][0][kinecthandler.joints_map[joints.ELBOW_RIGHT]]
            print convert_shoulder_right(res[i][0], res[i][1])
            # print res[i][1]
            # res[i] = convert_motors(res[i], kinect_h, "kinecthandler", "naocommander", must_filter=False)
            # print "==> ", res[i][kinecthandler.joints_map[joints.ELBOW_RIGHT]]
            # convert_shoulder_right(res[i])
            # print "==> "*2, res[i][kinecthandler.joints_map[joints.ELBOW_RIGHT]], '\n'
            # sleep(0.5)
        if j % 5 == 0:
            if j == 0:
                print "-" * 10, "BRAS LE LONG DU CORPS", "-" * 10
            if j == 5:
                print "-" * 10, "BRAS EN L'AIR", "-" * 10
            if j == 10:
                print "-" * 10, "BRAS HORIZONTAL DEVANT", "-" * 10
            if j == 15:
                print "-" * 10, "BRAS HORIZONTAL LATERAL", "-" * 10
            if j == 20:
                print "-" * 10, "BRAS OBLIQUE BAS LATERAL", "-" * 10
            if j == 25:
                print "-" * 10, "BRAS OBLIQUE HAUT LATERAL", "-" * 10
            if j == 30:
                print "-" * 10, "BRAS OBLIQUE AVANT BAS LATERAL", "-" * 10
            if j == 35:
                print "-" * 10, "BRAS OBLIQUE AVANT HAUT LATERAL", "-" * 10
            if j == 40:
                kinect_h.device.close()
                break
            sleep(3)
        else:
            sleep(1)
        j += 1


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
    phi = valid_angle(phi)
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
        tab = value_filter(joints.ELBOW_RIGHT, tab)
    return tab


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

    # print rot[elbow]
    phi = rot[elbow][1]
    phi += 90
    phi = valid_angle(phi)
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
        tab = value_filter(joints.ELBOW_LEFT, tab)
    return tab


def nao_test(nao_c):
    i = 0
    while i < 1:
        nao_c.waveYourLeftHand()
        i += 1
    # TODO : test nao's positions


def main(sensor="kinecthandler", avatar="naocommander"):
    _sensor = objects[sensor]()
    _avatar = objects[avatar](robotIP, PORT)
    kinect_test(_sensor, _avatar)
    # kinect_value_test(_sensor)


main()
