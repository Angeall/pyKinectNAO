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
    return [A, np.array([x, y , z])]


def get_right_elbow_roll(kinect_pos, world=None):
    if world is None:
        world = get_robot_world(kinect_pos)
    shoulder = kinect_pos[kinecthandler.joints_map[joints.SHOULDER_RIGHT]]
    elbow = kinect_pos[kinecthandler.joints_map[joints.ELBOW_RIGHT]]
    wrist = kinect_pos[kinecthandler.joints_map[joints.WRIST_RIGHT]]
    shoulder_elbow = utils.get_vector(elbow, shoulder, transform=world[0])
    elbow_wrist = utils.get_vector(wrist, elbow, transform=world[0])
    dot = np.dot(shoulder_elbow, np.reshape(elbow_wrist, (3, 1)))
    norm1 = np.linalg.norm(shoulder_elbow)
    norm2 = np.linalg.norm(elbow_wrist)
    dot2 = np.dot(norm1, norm2)
    return np.arccos(dot/dot2)


# def get_right_shoulder_pitch(kinect_pos, world=None):
#     if world is None:
#         world = get_robot_world(kinect_pos)
#     shoulder = kinect_pos[kinecthandler.joints_map[joints.SHOULDER_RIGHT]]
#     elbow = kinect_pos[kinecthandler.joints_map[joints.ELBOW_RIGHT]]
#     shoulder_elbow = utils.get_vector(elbow, shoulder, transform=world[0]).getA()[0]
#     return -1*(np.arctan(shoulder_elbow[0]/shoulder_elbow[2]))*(4/3.)

def get_right_shoulder_pitch(kinect_pos, world=None):
    if world is None:
        world = get_robot_world(kinect_pos)
    shoulder = kinect_pos[kinecthandler.joints_map[joints.SHOULDER_RIGHT]]
    spine_shoulder = kinect_pos[kinecthandler.joints_map[joints.SPINE_SHOULDER]]
    spine_mid = kinect_pos[kinecthandler.joints_map[joints.SPINE_MID]]
    elbow = kinect_pos[kinecthandler.joints_map[joints.ELBOW_RIGHT]]
    shoulder_elbow = utils.get_vector(elbow, shoulder, transform=world[0])
    shoulder_elbow = utils.normalize(shoulder_elbow).getA()[0]
    shoulder_spin_shoulder = utils.get_vector(spine_shoulder, shoulder, transform=world[0])
    shoulder_spin_shoulder = utils.normalize(shoulder_spin_shoulder)
    modified_spine_mid = [shoulder[0], spine_mid[1], spine_mid[2]]
    spine_shoulder_spine_mid = utils.get_vector(modified_spine_mid, spine_shoulder, transform=world[0])
    spine_shoulder_spine_mid = utils.normalize(spine_shoulder_spine_mid).getA()[0]
    cross = np.cross(spine_shoulder_spine_mid, shoulder_spin_shoulder)
    cross = np.reshape(utils.normalize(cross), (3, 1))
    cross = [cross[2], cross[1]]
    cross = utils.normalize(cross)
    shoulder_elbow = [shoulder_elbow[2], shoulder_elbow[1]]
    shoulder_elbow = utils.normalize(shoulder_elbow)
    sign = 1
    if shoulder_elbow[1]<0:
        sign = -1
    return sign*np.arccos(np.dot(shoulder_elbow, cross)[0])


def get_right_shoulder_roll(kinect_pos, world=None):
    if world is None:
        world = get_robot_world(kinect_pos)
    cross = np.cross(world[1][1], world[1][2])
    norm1 = np.linalg.norm(cross)
    shoulder = kinect_pos[kinecthandler.joints_map[joints.SHOULDER_RIGHT]]
    elbow = kinect_pos[kinecthandler.joints_map[joints.ELBOW_RIGHT]]
    shoulder_elbow = utils.get_vector(elbow, shoulder, transform=world[0])
    norm2 = np.linalg.norm(shoulder_elbow)
    return -1*((np.pi/2.)-np.arccos(np.dot(shoulder_elbow, cross)/np.dot(norm1, norm2)))


def get_right_elbow_yaw(kinect_pos, shoulder_roll=None, shoulder_pitch=None, world=None):
    # TODO : refaire tout le elbow yaw.. (y meler le shoulder pitch ?)
    if world is None:
        world = get_robot_world(kinect_pos)
    if shoulder_roll is None:
        shoulder_roll = get_right_shoulder_roll(kinect_pos, world)
    if shoulder_pitch is None:
        shoulder_pitch = get_right_shoulder_pitch(kinect_pos, world)
    shoulder = kinect_pos[kinecthandler.joints_map[joints.SHOULDER_RIGHT]]
    elbow = kinect_pos[kinecthandler.joints_map[joints.ELBOW_RIGHT]]
    wrist = kinect_pos[kinecthandler.joints_map[joints.WRIST_RIGHT]]
    shoulder_elbow = utils.get_vector(elbow, shoulder, transform=world[0])
    elbow_wrist = utils.get_vector(wrist, elbow, transform=world[0])
    roll_matrix = np.matrix([[np.cos(shoulder_roll), -np.sin(shoulder_roll), 0],
                             [np.sin(shoulder_roll), np.cos(shoulder_roll), 0], [0, 0, 1]])
    pitch_matrix = np.matrix([[np.cos(shoulder_pitch), 0, np.sin(shoulder_pitch)], [0, 1, 0],
                              [-np.sin(shoulder_pitch), 0, np.cos(shoulder_pitch)]])
    z = np.reshape(world[1][2], (3, 1))
    b = pitch_matrix*roll_matrix*z
    cross_arm = np.cross(shoulder_elbow, elbow_wrist)
    norm = np.linalg.norm(cross_arm)
    a_left = map(lambda x: x/norm, cross_arm)
    a_left = [a_left[0][0], a_left[0][1], a_left[0][2]]
    b = b.getA()
    b = [b[0][0], b[1][0], b[2][0]]
    return (np.pi/2.)-np.arccos(utils.normalized_dot(a_left, b))
