__author__ = 'Angeall'
import pykinectnao.kinecthandler as kinecthandler
import pykinectnao.naocommander as naocommander
import pykinectnao.converter as converter

robotIP = "192.168.2.24"
# robotIP = "127.0.0.1"
PORT = 9559
# PORT = 8352
nb_of_body = 1


def kinect_test(kinect_h, nao_c):
    while True:
        nao_c.device.waitUntilMoveIsFinished()
        res = kinect_h.get_movement(nb_of_body)
        if res == kinecthandler.NO_DATA:
            continue
        for i in range(nb_of_body):
            [r_s_roll, r_s_pitch, r_e_roll, r_e_yaw, r_w_yaw] = converter.get_right_arm(res[i][0], res[i][1])
            [l_s_roll, l_s_pitch, l_e_roll, l_e_yaw, l_w_yaw] = converter.get_left_arm(res[i][0], res[i][1])
            h_pitch = converter.get_head(res[i][0])
            [r_hand, l_hand] = converter.get_hands(res[i][2])
            nao_c.move_robot(right_shoulder_roll=r_s_roll, right_shoulder_pitch=r_s_pitch,
                             right_elbow_roll=r_e_roll, right_elbow_yaw=r_e_yaw,
                             right_wrist_yaw=r_w_yaw,
                             left_shoulder_roll=l_s_roll, left_shoulder_pitch=l_s_pitch,
                             left_elbow_roll=l_e_roll, left_elbow_yaw=l_e_yaw,
                             left_wrist_yaw=l_w_yaw,
                             head_pitch=h_pitch, right_hand=r_hand, left_hand=l_hand,
                             pfractionmaxspeed=0.4)


if __name__ == '__main__':
    _sensor = kinecthandler.KinectHandler()
    _avatar = naocommander.NAOCommander(robotIP, PORT)
    kinect_test(_sensor, _avatar)
