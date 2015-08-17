__author__ = 'Angeall'
from time import sleep
import cmath

import pykinectnao.kinecthandler as kinecthandler
import pykinectnao.naocommander as naocommander
import pykinectnao.naokinectlinker as naokinectlinker
import pykinectnao.joints as joints

robotIP = "192.168.2.24"
PORT = 9559


nb_of_body = 1


def kinect_test(kinect_h, nao_c):
    while True:
        res = kinect_h.get_movement(nb_of_body)
        if res == kinecthandler.NO_DATA:
            continue
        for i in range(nb_of_body):
            converted_shoulder = naokinectlinker.convert_shoulder_right(res[i][0], res[i][1])
            converted_elbow = naokinectlinker.convert_elbow_right(res[i][0], res[i][1])
            nao_c.user_right_arm_articular(shoulder_pitch=converted_shoulder[1],shoulder_roll=converted_shoulder[0],
                                           elbow_roll=converted_elbow, pfractionmaxspeed=0.8)
            converted_shoulder = naokinectlinker.convert_shoulder_left(res[i][0], res[i][1])
            converted_elbow = naokinectlinker.convert_elbow_left(res[i][0], res[i][1])
            nao_c.user_left_arm_articular(shoulder_pitch=converted_shoulder[1],shoulder_roll=converted_shoulder[0],
                                          elbow_roll=converted_elbow, pfractionmaxspeed=0.8)


def kinect_right_shoulder_test(kinect_h):
    j = 0
    while True:
        res = kinect_h.get_movement(nb_of_body)
        if res == kinecthandler.NO_DATA:
            continue
        for i in range(nb_of_body):
            # print "Body no", i
            # print res[i][0][kinecthandler.joints_map[joints.ELBOW_RIGHT]]
            print naokinectlinker.convert_shoulder_right(res[i][0], res[i][1])
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


def kinect_right_elbow_test(kinect_h):
    j = 0
    while True:
        res = kinect_h.get_movement(nb_of_body)
        if res == kinecthandler.NO_DATA:
            continue
        for i in range(nb_of_body):
            # print "Body no", i
            # print res[i][0][kinecthandler.joints_map[joints.ELBOW_RIGHT]]
            # print naokinectlinker.convert_elbow_right(res[i][0], res[i][1])
            print "ELBOW: ", res[i][1][kinecthandler.joints_map[joints.ELBOW_RIGHT]]
            print "WRIST: ", res[i][1][kinecthandler.joints_map[joints.WRIST_RIGHT]]
        if j % 5 == 0:
            if j == 0:
                print "-" * 10, "COUDE OUVERT BRAS DEVANT VERS LE CIEL POING CIEL", "-" * 10
            if j == 5:
                print "-" * 10, "COUDE OUVERT BRAS DEVANT VERS LE CIEL POING SOL", "-" * 10
            if j == 10:
                print "-" * 10, "COUDE OUVERT BRAS DEVANT VERS LE SOL", "-" * 10
            if j == 15:
                print "-" * 10, "COUDE FERME BRAS DEVANT VERS LE CIEL", "-" * 10
            if j == 20:
                print "-" * 10, "COUDE FERME LONG DU CORPS VERS CAMERA", "-" * 10
            if j == 25:
                print "-" * 10, "COUDE FERME BRAS DEVANT POING CONTRE POING", "-" * 10
            if j == 30:
                kinect_h.device.close()
                break
            sleep(3)
        else:
            sleep(1)
        j += 1


def main():
    _sensor = kinecthandler.KinectHandler()
    # _avatar = naocommander.NAOCommander(robotIP, PORT)
    # kinect_test(_sensor, _avatar)
    # kinect_right_shoulder_test(_sensor)
    kinect_right_elbow_test(_sensor)


main()
