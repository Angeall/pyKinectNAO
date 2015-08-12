__author__ = 'Angeall'
from time import sleep
import cmath
from collections import deque

from pykinect2 import PyKinectV2

import pykinectnao.kinecthandler as kinecthandler
import pykinectnao.naocommander as naocommander
import pykinectnao.utils as utils
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
    for joint in naocommander.motors_needed.keys():
        smoothing_dict[joint] = [deque(), deque(), deque()]
        last_movements[joint] = []
    j = 0
    while True:
        res = kinect_h.get_movement(nb_of_body)
        if res == kinecthandler.NO_DATA:
            continue
        for i in range(nb_of_body):
            res[i] = convert_motors(res[i], kinect_h, "kinecthandler", "naocommander")

            nao_c.user_right_arm_articular(shoulder_pitch=res[i][0][0][1], shoulder_roll=res[i][0][0][2],
                                           pfractionmaxspeed=0.7)

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


def convert_motors(results, device, sensor, avatar, must_filter=True):
    convert_tab = modules[avatar].motors_converters[sensor]
    if len(convert_tab) > 0:
        motors_needed = modules[avatar].motors_needed
        for joint in motors_needed.keys():
            if joint in convert_tab.keys():
                tab = results[0][motors_needed[joint][0]]
<<<<<<< Updated upstream
                if joint in last_movements.keys() and len(last_movements[joint]) != 0:
=======
                if len(last_movements) != 0:
>>>>>>> Stashed changes
                    # if joint not tracked properly
                    if device.positions[joint].TrackingState == PyKinectV2.TrackingState_NotTracked or \
                                    device.positions[joint].TrackingState == PyKinectV2.TrackingState_Inferred:
                        results[0][motors_needed[joint][0]] = last_movements[joint]
                        continue
                for i in range(3):
                    if motors_needed[joint][1][i]:
                        tab[i] = convert_tab[joint][i][0] * tab[i]
                        tab[i] = convert_tab[joint][i][1] + tab[i]
                        tab[i] = valid_angle(tab[i])
                        # If the X axis is inverted
                        if convert_tab[joint][i][2] == "-":
                            c = cmath.rect(1, tab[i] / 180. * cmath.pi)
                            c = complex(-1 * c.real, c.imag)
                            tab[i] = round(cmath.phase(c) / cmath.pi * 180, 1)
                        # If the Y axis is inverted
                        if convert_tab[joint][i][3] == "-":
                            c = cmath.rect(1, tab[i] / 180. * cmath.pi)
                            c = c.conjugate()
                            tab[i] = round(cmath.phase(c) / cmath.pi * 180, 1)
                        if must_filter:
                            if len(smoothing_dict[joint][i]) == FILTER_SIZE:
                                smoothing_dict[joint][i].popleft()
                            smoothing_dict[joint][i].append(tab[i])
                            tab[i] = utils.holt_winters_second_order_ewma \
                                (smoothing_dict[joint][i], FILTER_SPAN, FILTER_BETA)[-1]
                last_movements[joint] = tab
    return results


def kinect_value_test(kinect_h):
    j = 0
    while True:
        res = kinect_h.get_movement(nb_of_body)
        if res == kinecthandler.NO_DATA:
            continue
        print res[0][11]

        x = res[0][11][1]
        y = res[0][11][2]
        z = res[0][11][3]
        yz = complex(y, z)
        xy = complex(x, y)
        print "Roll: ", cmath.phase(xy)*180.0/cmath.pi, " - Pitch: ", cmath.phase(yz)*180.0/cmath.pi
        # print utils.cart_to_spher([x, y, z])
        #for i in range(nb_of_body):
            # print "Body no", i
<<<<<<< Updated upstream
            #print res[i][0]
            #convert_motors(res[i], kinect_h, "kinecthandler", "naocommander", must_filter=False)
            #print "==> ", res[i][0]
            # sleep(0.5)
        if j == 0:
            print "-" * 10, "BRAS LE LONG DU CORPS", "-" * 10
            sleep(3)
        if j == 5:
            print "-" * 10, "BRAS EN L'AIR", "-" * 10
            sleep(3)
        if j == 10:
            print "-" * 10, "BRAS HORIZONTAL DEVANT", "-" * 10
            sleep(3)
        if j == 15:
            print "-" * 10, "BRAS HORIZONTAL LATERAL", "-" * 10
            sleep(3)
        if j == 20:
            print "-" * 10, "BRAS OBLIQUE BAS LATERAL", "-" * 10
            sleep(3)
        if j == 25:
            print "-" * 10, "BRAS OBLIQUE HAUT LATERAL", "-" * 10
=======
            print res[i][0]
            convert_motors(res[i], kinect_h, "kinecthandler", "naocommander", must_filter=False)
            print "==> ", res[i][0]
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
                kinect_h.device.close()
                break
>>>>>>> Stashed changes
            sleep(3)
        if j == 30:
            kinect_h.device.close()
            break
        else:
            sleep(1)
        j += 1


def nao_test(nao_c):
    i = 0
    while i < 1:
        nao_c.waveYourLeftHand()
        i += 1


def main(sensor="kinecthandler", avatar="naocommander"):
<<<<<<< Updated upstream
    _sensor = objects[sensor]()
    # _avatar = objects[avatar](robotIP, PORT)
    # kinect_test(_sensor, _avatar)
=======
    _sensor = objects[sensor](avatar)
    #_avatar = objects[avatar](robotIP, PORT)
    #kinect_test(_sensor, _avatar)
>>>>>>> Stashed changes
    kinect_value_test(_sensor)


main()
