import pykinectnao.joints as joints

__author__ = 'Angeall'
from time import sleep
import cmath

import pykinectnao.kinecthandler as kinecthandler
import pykinectnao.naocommander as naocommander
import pykinectnao.converter as converter
from math import pi
from multiprocessing import Pool

robotIP = "192.168.2.24"
# robotIP = "127.0.0.1"
PORT = 9559
# PORT = 8352
nb_of_body = 1


def kinect_test(kinect_h, nao_c):
    p = Pool(processes=4)
    while True:
        nao_c.device.waitUntilMoveIsFinished()
        res = kinect_h.get_movement(nb_of_body)
        if res == kinecthandler.NO_DATA:
            continue
        for i in range(nb_of_body):
            [r_s_roll, r_s_pitch, r_e_roll, r_e_yaw, r_w_yaw] = converter.get_right_arm(res[i][0], res[i][1])
            [l_s_roll, l_s_pitch, l_e_roll, l_e_yaw, l_w_yaw] = converter.get_left_arm(res[i][0], res[i][1])
            h_pitch = converter.get_head(res[i][0])
            nao_c.move_robot(right_shoulder_roll=r_s_roll, right_shoulder_pitch=r_s_pitch,
                             right_elbow_roll=r_e_roll, right_elbow_yaw=r_e_yaw,
                             right_wrist_yaw=r_w_yaw,
                             left_shoulder_roll=l_s_roll, left_shoulder_pitch=l_s_pitch,
                             left_elbow_roll=l_e_roll, left_elbow_yaw=l_e_yaw,
                             left_wrist_yaw=l_w_yaw,
                             head_pitch=h_pitch,
                             pfractionmaxspeed=0.6)
            # [s_roll, s_pitch, e_roll, e_yaw, w_yaw] = converter.get_right_arm(res[i][0], res[i][1])
            # nao_c.user_right_arm_articular(shoulder_pitch=s_pitch, shoulder_roll=s_roll,
            #                                elbow_roll=e_roll, elbow_yaw=e_yaw,
            #                                wrist_yaw=w_yaw,
            #                                pfractionmaxspeed=0.6)
            # [s_roll, s_pitch, e_roll, e_yaw, w_yaw] = converter.get_left_arm(res[i][0], res[i][1])
            # nao_c.user_left_arm_articular(shoulder_pitch=s_pitch, shoulder_roll=s_roll,
            #                               elbow_roll=e_roll, elbow_yaw=e_yaw,
            #                               wrist_yaw=w_yaw,
            #                               pfractionmaxspeed=0.6)

def kinect_right_shoulder_test(kinect_h):
    j = 0
    while True:
        res = kinect_h.get_movement(nb_of_body)
        if res == kinecthandler.NO_DATA:
            continue
        for i in range(nb_of_body):
            # print "Body no", i
            # print res[i][0][kinecthandler.joints_map[joints.ELBOW_RIGHT]]
            world = converter.get_robot_world(res[i][0])
            print converter.get_right_shoulder_pitch(res[i][0], world=world)*180/pi
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
    nb_of_test = 5
    while True:
        res = kinect_h.get_movement(nb_of_body)
        if res == kinecthandler.NO_DATA:
            continue
        for i in range(nb_of_body):
            # print "Body no", i
            # print res[i][0][kinecthandler.joints_map[joints.ELBOW_RIGHT]]
            # print naokinectlinker.convert_elbow_right(res[i][0], res[i][1], must_filter=False)[1]
            # world = converter.get_robot_world(res[i][0])
            # s_pitch = converter.get_right_shoulder_pitch(res[i][0], world)
            # s_roll = converter.get_right_shoulder_roll(res[i][0], world)
            # e_roll = converter.get_right_elbow_roll(res[i][0], world)
            # e_yaw = converter.get_right_elbow_yaw(res[i][0],
            #                                       shoulder_pitch=s_pitch,
            #                                       shoulder_roll=s_roll,
            #                                       world=world)
            # print "E_YAW", e_yaw*180./pi
            # print "S_PITCH", s_pitch*180./pi
            # print "S_ROLL", s_roll*180./pi
            # print ""
            print "SHOULDER: ", res[i][1][kinecthandler.joints_map[joints.SHOULDER_RIGHT]]
            print "ELBOW: ", res[i][1][kinecthandler.joints_map[joints.ELBOW_RIGHT]]
            print "WRIST: ", res[i][1][kinecthandler.joints_map[joints.WRIST_RIGHT]]
            print "HAND: ", res[i][1][kinecthandler.joints_map[joints.HAND_RIGHT]], '\n'
            # x = kinect_h.orientations[joints.ELBOW_RIGHT].Orientation.x
            # y = kinect_h.orientations[joints.ELBOW_RIGHT].Orientation.y
            # z = kinect_h.orientations[joints.ELBOW_RIGHT].Orientation.z
            # w = kinect_h.orientations[joints.ELBOW_RIGHT].Orientation.w
            # quaternion_p = [w, x, y, z]
            # x = kinect_h.orientations[joints.WRIST_RIGHT].Orientation.x
            # y = kinect_h.orientations[joints.WRIST_RIGHT].Orientation.y
            # z = kinect_h.orientations[joints.WRIST_RIGHT].Orientation.z
            # w = kinect_h.orientations[joints.WRIST_RIGHT].Orientation.w
            # quaternion_c = [w, x, y, z]
            # quaternion_y = [0, 0, 1, 0]
            # quat = transformations.quaternion_multiply(transformations.quaternion_conjugate(quaternion_p), quaternion_c)
            # # quat = quaternion_c
            # print "ELBOW-WRIST: ", "[", kinect_h.compute_yaw(quat), ", ", kinect_h.compute_pitch(quat), ", ", \
            #                             kinect_h.compute_roll(quat), "]", '\n'
            # print "ELBOW-WRIST: ", utils.quat_to_axisangle(quat), '\n'
        if j % nb_of_test == 0:
            if j == 0:
                print "-" * 10, "BRAS DEVANT POINT EN HAUT", "-" * 10
            if j == nb_of_test:
                print "-" * 10, "BRAS DEVANT POINT EN BAS", "-" * 10
            if j == 2*nb_of_test:
                print "-" * 10, "BRAS LATERAL POINT EN HAUT", "-" * 10
            if j == 3*nb_of_test:
                print "-" * 10, "BRAS LATERAL POINT EN BAS", "-" * 10
            # if j == 4*nb_of_test:
            #     print "-" * 10, "COUDE FERME BRAS LATERAL VERS CAMERA", "-" * 10
            # if j == 5*nb_of_test:
            #     print "-" * 10, "COUDE FERME BRAS LONG DU CORPS VERS CAMERA", "-" * 10
            if j == 4*nb_of_test:
                kinect_h.device.close()
                break
            sleep(3)
        else:
            sleep(1)
        j += 1


if __name__ == '__main__':
    _sensor = kinecthandler.KinectHandler()
    _avatar = naocommander.NAOCommander(robotIP, PORT)
    kinect_test(_sensor, _avatar)
    # kinect_right_shoulder_test(_sensor)
    # kinect_right_elbow_test(_sensor)
