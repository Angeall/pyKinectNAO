__author__ = 'Angeall'
from pykinect2 import PyKinectV2
from pykinect2 import PyKinectRuntime
from math import *
import joints
import utils
import transformations

EPSILON = 0.0001
NO_DATA = -1

# joints_tree = Tree(joints.SPINE_BASE,
#                    [Node(joints.SPINE_MID,
#                          [Node(joints.SPINE_SHOULDER,
#                                [Node(joints.NECK,
#                                      [Node(joints.HEAD)]),
#                                 Node(joints.SHOULDER_LEFT,
#                                      [Node(joints.ELBOW_LEFT,
#                                            [Node(joints.WRIST_LEFT,
#                                                  [Node(joints.HAND_LEFT,
#                                                        [Node(joints.HAND_TIP_LEFT)]),
#                                                   Node(joints.THUMB_LEFT)])])]),
#                                 Node(joints.SHOULDER_RIGHT,
#                                      [Node(joints.ELBOW_RIGHT,
#                                            [Node(joints.WRIST_RIGHT,
#                                                  [Node(joints.HAND_RIGHT,
#                                                        [Node(joints.HAND_TIP_RIGHT)]),
#                                                   Node(joints.THUMB_RIGHT)])])])])]),
#                     Node(joints.HIP_LEFT,
#                          [Node(joints.KNEE_LEFT,
#                                [Node(joints.ANKLE_LEFT,
#                                      [Node(joints.FOOT_LEFT)])])]),
#                     Node(joints.HIP_RIGHT,
#                          [Node(joints.KNEE_RIGHT,
#                                [Node(joints.ANKLE_RIGHT,
#                                      [Node(joints.FOOT_RIGHT)])])])])

joints_map = {joints.SPINE_BASE: 0,
              joints.SPINE_MID: 1,
              joints.NECK: 2,
              joints.HEAD: 3,
              joints.SHOULDER_LEFT: 4,
              joints.ELBOW_LEFT: 5,
              joints.WRIST_LEFT: 6,
              joints.HAND_LEFT: 7,
              joints.SHOULDER_RIGHT: 8,
              joints.ELBOW_RIGHT: 9,
              joints.WRIST_RIGHT: 10,
              joints.HAND_RIGHT: 11,
              joints.HIP_LEFT: 12,
              joints.KNEE_LEFT: 13,
              joints.ANKLE_LEFT: 14,
              joints.FOOT_LEFT: 15,
              joints.HIP_RIGHT: 16,
              joints.KNEE_RIGHT: 17,
              joints.ANKLE_RIGHT: 18,
              joints.FOOT_RIGHT: 19,
              joints.SPINE_SHOULDER: 20,
              joints.HAND_TIP_LEFT: 21,
              joints.THUMB_LEFT: 22,
              joints.HAND_TIP_RIGHT: 23,
              joints.THUMB_RIGHT: 24, }

bones_map = {(joints.SPINE_BASE, joints.SPINE_MID): 0,
             (joints.SPINE_MID, joints.SPINE_SHOULDER): 1,
             (joints.SPINE_SHOULDER, joints.NECK): 2,
             (joints.NECK, joints.HEAD): 3,
             (joints.SPINE_SHOULDER, joints.SHOULDER_LEFT): 4,
             (joints.SHOULDER_LEFT, joints.ELBOW_LEFT): 5,
             (joints.ELBOW_LEFT, joints.WRIST_LEFT): 6,
             (joints.WRIST_LEFT, joints.HAND_LEFT): 7,
             (joints.HAND_LEFT, joints.HAND_TIP_LEFT): 8,
             (joints.WRIST_LEFT, joints.THUMB_LEFT): 9,
             (joints.SPINE_SHOULDER, joints.SHOULDER_RIGHT): 10,
             (joints.SHOULDER_RIGHT, joints.ELBOW_RIGHT): 11,
             (joints.ELBOW_RIGHT, joints.WRIST_RIGHT): 12,
             (joints.WRIST_RIGHT, joints.HAND_RIGHT): 13,
             (joints.HAND_RIGHT, joints.HAND_TIP_RIGHT): 14,
             (joints.WRIST_RIGHT, joints.THUMB_RIGHT): 15,
             (joints.SPINE_BASE, joints.HIP_LEFT): 16,
             (joints.HIP_LEFT, joints.KNEE_LEFT): 17,
             (joints.KNEE_LEFT, joints.ANKLE_LEFT): 18,
             (joints.ANKLE_LEFT, joints.FOOT_LEFT): 19,
             (joints.SPINE_BASE, joints.HIP_RIGHT): 20,
             (joints.HIP_RIGHT, joints.KNEE_RIGHT): 21,
             (joints.KNEE_RIGHT, joints.ANKLE_RIGHT): 22,
             (joints.ANKLE_RIGHT, joints.FOOT_RIGHT): 23}


class KinectHandler():
    def __init__(self):
        self.device = PyKinectRuntime.PyKinectRuntime(PyKinectV2.FrameSourceTypes_Color |
                                                      PyKinectV2.FrameSourceTypes_Body)
        self.bodies = None
        self.active_bodies_indices = []
        self.orientations_pattern_list = []
        for i in range(len(bones_map)):
            self.orientations_pattern_list.append(None)

    def close(self):
        self.device.close()

    def get_movement(self, nb_of_body=1):
        if self.device.has_new_body_frame():
            self.bodies = self.device.get_last_body_frame()
        if self.bodies is not None:
            for index in self.active_bodies_indices:
                if not self.bodies.bodies[index].is_tracked:
                    self.active_bodies_indices.remove(index)
            # Search after bodies Kinect might have detected
            for i in range(0, self.device.max_body_count):
                body = self.bodies.bodies[i]
                if not body.is_tracked:
                    continue
                else:
                    if i not in self.active_bodies_indices:
                        self.active_bodies_indices.append(i)
            if len(self.active_bodies_indices) == 0:
                return NO_DATA
            res = []
            for j in range(nb_of_body):
                body = self.bodies.bodies[self.active_bodies_indices[j]]
                self.positions = body.joints
                self.orientations = body.joint_orientations
                res.append(self.convert_orientations())
            return res
        else:
            return NO_DATA

    def convert_orientations(self):
        orientations = self.orientations_pattern_list[:]
        for bone in bones_map.keys():
            or1 = self.orientations[joints_map[bone[0]]].Orientation
            q1 = [or1.w, or1.x, or1.y, or1.z]
            or2 = self.orientations[joints_map[bone[1]]].Orientation
            q2 = [or2.w, or2.x, or2.y, or2.z]
            # q = [q2[0] - q1[0], q2[1] - q1[1], q2[2] - q1[2], q2[3] - q1[3]]
            # q3 = [0, 0, 1, 0]
            # q4 = transformations.quaternion_multiply(q, q3)
            # q = transformations.quaternion_multiply(q4, transformations.quaternion_conjugate(q))
            orientations[bones_map[bone]] = utils.quat_to_axisangle(q2)
        return orientations
    #
    # def convet_orientations(self):
    #     orientations = self.orientations_pattern_list[:]