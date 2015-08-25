import cmath

__author__ = 'Angeall'
import kinecthandler
import joints
import numpy as np
import utils


def get_robot_world(kinect_pos):
    hip_right = kinect_pos[kinecthandler.joints_map[joints.HIP_RIGHT]]
    hip_left = kinect_pos[kinecthandler.joints_map[joints.HIP_LEFT]]
    spine_shoulder = kinect_pos[kinecthandler.joints_map[joints.SPINE_SHOULDER]]
    hip_vector = utils.get_vector(hip_right, hip_left)
    spine_shoulder_to_hip = utils.get_vector(hip_right, spine_shoulder)
    z = utils.normalized_cross(hip_vector, spine_shoulder_to_hip)
    x = utils.normalize(hip_vector)
    y = np.cross(z, x)
    z_0 = np.array([0, 0, 1])
    y_0 = np.array([0, 1, 0])
    x_0 = np.array([1, 0, 0])

    x1 = utils.normalized_dot(x, x_0)
    x2 = utils.normalized_dot(x, y_0)
    x3 = utils.normalized_dot(x, z_0)
    y1 = utils.normalized_dot(y, x_0)
    y2 = utils.normalized_dot(y, y_0)
    y3 = utils.normalized_dot(y, z_0)
    z1 = utils.normalized_dot(z, x_0)
    z2 = utils.normalized_dot(z, y_0)
    z3 = utils.normalized_dot(z, z_0)
    A = np.matrix([[x1, x2, x3], [y1, y2, y3], [z1, z2, z3]])
    return [A, np.array([x, y, z])]


def get_right_elbow_roll(kinect_pos, world=None):
    if world is None:
        world = get_robot_world(kinect_pos)
    shoulder = kinect_pos[kinecthandler.joints_map[joints.SHOULDER_RIGHT]]
    elbow = kinect_pos[kinecthandler.joints_map[joints.ELBOW_RIGHT]]
    wrist = kinect_pos[kinecthandler.joints_map[joints.WRIST_RIGHT]]
    shoulder_elbow = utils.get_vector(elbow, shoulder, transform=world[0])
    elbow_wrist = utils.get_vector(wrist, elbow, transform=world[0])
    return np.arccos(utils.normalized_dot(shoulder_elbow, elbow_wrist))


def get_left_elbow_roll(kinect_pos, world=None):
    if world is None:
        world = get_robot_world(kinect_pos)
    shoulder = kinect_pos[kinecthandler.joints_map[joints.SHOULDER_LEFT]]
    elbow = kinect_pos[kinecthandler.joints_map[joints.ELBOW_LEFT]]
    wrist = kinect_pos[kinecthandler.joints_map[joints.WRIST_LEFT]]
    shoulder_elbow = utils.get_vector(elbow, shoulder, transform=world[0])
    elbow_wrist = utils.get_vector(wrist, elbow, transform=world[0])
    return -np.arccos(utils.normalized_dot(shoulder_elbow, elbow_wrist))


def get_right_shoulder_pitch(kinect_pos, world=None):
    if world is None:
        world = get_robot_world(kinect_pos)
    shoulder = kinect_pos[kinecthandler.joints_map[joints.SHOULDER_RIGHT]]
    spine_shoulder = kinect_pos[kinecthandler.joints_map[joints.SPINE_SHOULDER]]
    spine_mid = kinect_pos[kinecthandler.joints_map[joints.SPINE_MID]]
    elbow = kinect_pos[kinecthandler.joints_map[joints.ELBOW_RIGHT]]
    finger_tip = kinect_pos[kinecthandler.joints_map[joints.HAND_TIP_RIGHT]]
    head = kinect_pos[kinecthandler.joints_map[joints.HEAD]]

    shoulder_elbow = utils.get_vector(elbow, shoulder, transform=world[0])
    shoulder_elbow = utils.normalize(shoulder_elbow)
    shoulder_spin_shoulder = utils.get_vector(spine_shoulder, shoulder, transform=world[0])
    shoulder_spin_shoulder = utils.normalize(shoulder_spin_shoulder)
    modified_spine_mid = [shoulder[0], spine_mid[1], spine_mid[2]]
    spine_shoulder_spine_mid = utils.get_vector(modified_spine_mid, spine_shoulder, transform=world[0])
    spine_shoulder_spine_mid = utils.normalize(spine_shoulder_spine_mid)
    cross = np.cross(spine_shoulder_spine_mid, shoulder_spin_shoulder)
    cross = np.reshape(utils.normalize(cross), (3, 1))
    # Only z and y needed
    cross = [cross[2], cross[1]]
    cross = utils.normalize(cross)
    # Only z and y needed
    shoulder_elbow = [shoulder_elbow[2], shoulder_elbow[1]]
    shoulder_elbow = utils.normalize(shoulder_elbow)
    sign = -1
    # If the elbow is higher than the shoulder
    if shoulder_elbow[1] < 0 or finger_tip[1] > head[1]:
        sign = 1
    res = sign * np.arccos(np.dot(shoulder_elbow, cross)[0])
    if res > 1.79:
        res = 1.79
    elif res < -1.57:
        res = -1.57
    return res


def get_left_shoulder_pitch(kinect_pos, world=None):
    if world is None:
        world = get_robot_world(kinect_pos)
    shoulder = kinect_pos[kinecthandler.joints_map[joints.SHOULDER_LEFT]]
    spine_shoulder = kinect_pos[kinecthandler.joints_map[joints.SPINE_SHOULDER]]
    spine_mid = kinect_pos[kinecthandler.joints_map[joints.SPINE_MID]]
    elbow = kinect_pos[kinecthandler.joints_map[joints.ELBOW_LEFT]]
    finger_tip = kinect_pos[kinecthandler.joints_map[joints.HAND_TIP_LEFT]]
    head = kinect_pos[kinecthandler.joints_map[joints.HEAD]]

    shoulder_elbow = utils.get_vector(elbow, shoulder, transform=world[0])
    shoulder_elbow = utils.normalize(shoulder_elbow)
    shoulder_spin_shoulder = utils.get_vector(spine_shoulder, shoulder, transform=world[0])
    shoulder_spin_shoulder = utils.normalize(shoulder_spin_shoulder)
    modified_spine_mid = [shoulder[0], spine_mid[1], spine_mid[2]]
    spine_shoulder_spine_mid = utils.get_vector(modified_spine_mid, spine_shoulder, transform=world[0])
    spine_shoulder_spine_mid = utils.normalize(spine_shoulder_spine_mid)
    cross = np.cross(shoulder_spin_shoulder, spine_shoulder_spine_mid)
    cross = np.reshape(utils.normalize(cross), (3, 1))
    # Only z and y needed
    cross = [cross[2], cross[1]]
    cross = utils.normalize(cross)
    # Only z and y needed
    shoulder_elbow = [shoulder_elbow[2], shoulder_elbow[1]]
    shoulder_elbow = utils.normalize(shoulder_elbow)
    sign = 1
    # If the elbow is higher than the shoulder
    if shoulder_elbow[1] < 0 or finger_tip[1] > head[1]:
        sign = -1
    res = sign * np.arccos(np.dot(shoulder_elbow, cross)[0])
    if res > 1.79:
        res = 1.79
    elif res < -1.57:
        res = -1.57
    return res


def get_right_shoulder_roll(kinect_pos, world=None):
    if world is None:
        world = get_robot_world(kinect_pos)
    cross = np.cross(world[1][1], world[1][2])
    # norm1 = np.linalg.norm(cross)
    shoulder = kinect_pos[kinecthandler.joints_map[joints.SHOULDER_RIGHT]]
    elbow = kinect_pos[kinecthandler.joints_map[joints.ELBOW_RIGHT]]
    shoulder_elbow = utils.get_vector(elbow, shoulder, transform=world[0])
    # norm2 = np.linalg.norm(shoulder_elbow)
    res = -1 * ((np.pi / 2.) - np.arccos(utils.normalized_dot(shoulder_elbow, cross)))
    if res > 0.085:
        res = 0.085
    elif res < -1.13:
        res = -1.13
    return res


def get_left_shoulder_roll(kinect_pos, world=None):
    if world is None:
        world = get_robot_world(kinect_pos)
    cross = np.cross(world[1][1], world[1][2])
    # norm1 = np.linalg.norm(cross)
    shoulder = kinect_pos[kinecthandler.joints_map[joints.SHOULDER_LEFT]]
    elbow = kinect_pos[kinecthandler.joints_map[joints.ELBOW_LEFT]]
    shoulder_elbow = utils.get_vector(elbow, shoulder, transform=world[0])
    # norm2 = np.linalg.norm(shoulder_elbow)
    res = -1*((np.pi / 2.) - np.arccos(utils.normalized_dot(shoulder_elbow, cross)))
    if res < -0.085:
        res = -0.085
    elif res > 1.13:
        res = 1.13
    return res


def get_right_elbow_yaw(kinect_pos, shoulder_roll=None, shoulder_pitch=None, world=None):
    if world is None:
        world = get_robot_world(kinect_pos)
    if shoulder_roll is None:
        shoulder_roll = get_right_shoulder_roll(kinect_pos, world)
    if shoulder_pitch is None:
        shoulder_pitch = get_right_shoulder_pitch(kinect_pos, world)
    shoulder = kinect_pos[kinecthandler.joints_map[joints.SHOULDER_RIGHT]]
    elbow = kinect_pos[kinecthandler.joints_map[joints.ELBOW_RIGHT]]
    wrist = kinect_pos[kinecthandler.joints_map[joints.WRIST_RIGHT]]
    pitch_matrix = np.matrix([[1, 0, 0],
                              [0, np.cos(shoulder_pitch), -np.sin(shoulder_pitch)],
                              [0, np.sin(shoulder_pitch), np.cos(shoulder_pitch)]])
    roll_matrix = np.matrix([[np.cos(shoulder_roll), 0, np.sin(shoulder_roll)],
                             [0, 1, 0],
                             [-np.sin(shoulder_roll), 0, np.cos(shoulder_roll)]])
    transform = world[0] * pitch_matrix * roll_matrix
    elbow_shoulder = utils.get_vector(shoulder, elbow, transform=transform)
    elbow_shoulder = utils.normalize(elbow_shoulder)
    modified_elbow = [elbow[0], elbow[1] + 2, elbow[2]]
    elbow_vertical = utils.get_vector(modified_elbow, elbow, transform=transform)
    elbow_wrist = utils.get_vector(wrist, elbow, transform=transform)
    elbow_wrist = utils.normalize([elbow_wrist[0], elbow_wrist[1]])
    cross_arm = np.cross(elbow_shoulder, elbow_vertical)
    cross_arm = utils.normalize([cross_arm[0], cross_arm[1]])
    # cross_arm = np.array([cross_arm[0], cross_arm[1]])
    # elbow_wrist = np.array([elbow_wrist[0], elbow_wrist[1]])
    sign = 1
    if elbow_wrist[1] > 0:
        sign = -1
    dot = utils.normalized_dot(elbow_wrist, cross_arm)
    return sign * (np.arccos(dot))


def get_left_elbow_yaw(kinect_pos, shoulder_roll=None, shoulder_pitch=None, world=None):
    if world is None:
        world = get_robot_world(kinect_pos)
    if shoulder_roll is None:
        shoulder_roll = get_right_shoulder_roll(kinect_pos, world)
    if shoulder_pitch is None:
        shoulder_pitch = get_right_shoulder_pitch(kinect_pos, world)
    shoulder = kinect_pos[kinecthandler.joints_map[joints.SHOULDER_LEFT]]
    elbow = kinect_pos[kinecthandler.joints_map[joints.ELBOW_LEFT]]
    wrist = kinect_pos[kinecthandler.joints_map[joints.WRIST_LEFT]]
    pitch_matrix = np.matrix([[1, 0, 0],
                              [0, np.cos(shoulder_pitch), -np.sin(shoulder_pitch)],
                              [0, np.sin(shoulder_pitch), np.cos(shoulder_pitch)]])
    roll_matrix = np.matrix([[np.cos(shoulder_roll), 0, np.sin(shoulder_roll)],
                             [0, 1, 0],
                             [-np.sin(shoulder_roll), 0, np.cos(shoulder_roll)]])
    transform = world[0] * pitch_matrix * roll_matrix
    elbow_shoulder = utils.get_vector(shoulder, elbow, transform=transform)
    elbow_shoulder = utils.normalize(elbow_shoulder)
    modified_elbow = [elbow[0], elbow[1] + 2, elbow[2]]
    elbow_vertical = utils.get_vector(modified_elbow, elbow, transform=transform)
    elbow_wrist = utils.get_vector(wrist, elbow, transform=transform)
    elbow_wrist = utils.normalize([elbow_wrist[0], elbow_wrist[1]])
    cross_arm = np.cross(elbow_vertical, elbow_shoulder)
    cross_arm = utils.normalize([cross_arm[0], cross_arm[1]])
    # cross_arm = np.array([cross_arm[0], cross_arm[1]])
    # elbow_wrist = np.array([elbow_wrist[0], elbow_wrist[1]])
    sign = -1
    if elbow_wrist[1] > 0:
        sign = 1
    dot = utils.normalized_dot(elbow_wrist, cross_arm)
    return sign * (np.arccos(dot))


def get_right_arm(kinect_pos, kinect_rot):
    world = get_robot_world(kinect_pos)
    shoulder_roll = get_right_shoulder_roll(kinect_pos, world=world) * 180 / np.pi
    shoulder_pitch = get_right_shoulder_pitch(kinect_pos, world=world) * 180 / np.pi
    elbow_roll = get_right_elbow_roll(kinect_pos, world=world) * 180 / np.pi
    elbow_yaw = get_right_elbow_yaw(kinect_pos,
                                    shoulder_pitch=shoulder_pitch * np.pi / 180,
                                    shoulder_roll=shoulder_roll * np.pi / 180,
                                    world=world) * 180 / np.pi
    wrist = kinect_pos[kinecthandler.joints_map[joints.WRIST_RIGHT]]
    elbow = kinect_pos[kinecthandler.joints_map[joints.ELBOW_RIGHT]]
    vect = utils.get_vector(wrist, elbow, transform=world[0])
    if elbow_roll > 60:
        # if vect[1] > 0 and shoulder_roll < -40 and shoulder_pitch > 21:
        #     shoulder_pitch = 21
        elbow_yaw += abs(shoulder_pitch) - 15
    # Si l'angle du coude n'est pas assez important, bras plat
    else:
        elbow_yaw = 80
        elbow_roll = 7.5
    wrist_yaw = kinect_rot[kinecthandler.joints_map[joints.WRIST_RIGHT]][2] + \
                kinect_rot[kinecthandler.joints_map[joints.ELBOW_RIGHT]][2]
    c = cmath.rect(1, wrist_yaw * cmath.pi / 180.)
    c = c.conjugate()
    wrist_yaw = round(cmath.phase(c)*180 / cmath.pi, 1) + 30 - elbow_yaw +80
    return [shoulder_roll, shoulder_pitch, elbow_roll, elbow_yaw, wrist_yaw]


def get_left_arm(kinect_pos, kinect_rot):
    world = get_robot_world(kinect_pos)
    shoulder_roll = get_left_shoulder_roll(kinect_pos, world=world) * 180 / np.pi
    shoulder_pitch = get_left_shoulder_pitch(kinect_pos, world=world) * 180 / np.pi
    elbow_roll = get_left_elbow_roll(kinect_pos, world=world) * 180 / np.pi
    elbow_yaw = get_left_elbow_yaw(kinect_pos,
                                    shoulder_pitch=shoulder_pitch * np.pi / 180,
                                    shoulder_roll=shoulder_roll * np.pi / 180,
                                    world=world) * 180 / np.pi
    wrist = kinect_pos[kinecthandler.joints_map[joints.WRIST_LEFT]]
    elbow = kinect_pos[kinecthandler.joints_map[joints.ELBOW_LEFT]]
    vect = utils.get_vector(wrist, elbow, transform=world[0])
    if elbow_roll < -60:
        # if vect[1] > 0 and shoulder_roll < -40 and shoulder_pitch > 21:
        #     shoulder_pitch = 21
        elbow_yaw -= abs(shoulder_pitch) - 15
    # Si l'angle du coude n'est pas assez important, bras plat
    else:
        elbow_yaw = -80
        elbow_roll = -7.5
    wrist_yaw = kinect_rot[kinecthandler.joints_map[joints.WRIST_LEFT]][2] + \
                kinect_rot[kinecthandler.joints_map[joints.ELBOW_LEFT]][2]
    c = cmath.rect(1, wrist_yaw * cmath.pi / 180.)
    c = c.conjugate()
    wrist_yaw = round(cmath.phase(c)*180 / cmath.pi, 1) - 30 + elbow_yaw - 80
    return [shoulder_roll, shoulder_pitch, elbow_roll, elbow_yaw, wrist_yaw]
