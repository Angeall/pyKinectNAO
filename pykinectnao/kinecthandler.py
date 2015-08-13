__author__ = 'Angeall'
from pykinect2 import PyKinectV2
from pykinect2 import PyKinectRuntime
import pykinectnao.joints as joints
import transformations
from math import *

NO_DATA = -1

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


class KinectHandler():
    def __init__(self):
        self.device = PyKinectRuntime.PyKinectRuntime(PyKinectV2.FrameSourceTypes_Color |
                                                      PyKinectV2.FrameSourceTypes_Body)
        self.bodies = None
        self.active_bodies_indices = []

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
                res.append(self.convert_orientation())
            return res
        else:
            return NO_DATA

    # Be sure to have the proper joint_map linked with self.joint_map
    # Be sure to have a non-empty list of positions
    # This method match the sensor joint_map with the avatar position_needed map
    # Override if needed
    # def convert_positions(self):
    #     if self.positions is None:
    #         return []
    #     positions = self.position_pattern_list[:]
    #     for index in self.positions_conversion_map.keys():
    #         positions[self.positions_conversion_map[index]] = [self.positions[index].Position.x,
    #                                                            self.positions[index].Position.y,
    #                                                            self.positions[index].Position.z]
    #         return positions

    # Be sure to have the proper joint_map linked with self.joint_map
    # Be sure to have a non-empty list of orientations
    # This method match the sensor joint_map with the avatar motors_needed map
    # Override if needed
    def convert_orientation(self):
        orientations = []
        for i in range(25):
            orientations.append(None)
        for joint in joints_map.keys():
            tab = [None, None, None]
            tab[0] = self.compute_yaw(joints_map[joint])
            tab[1] = self.compute_pitch(joints_map[joint])
            tab[2] = self.compute_roll(joints_map[joint])
        #     index = joints_map[joint]
        #     x = self.orientations[index].Orientation.x
        #     y = self.orientations[index].Orientation.y
        #     z = self.orientations[index].Orientation.z
        #     w = self.orientations[index].Orientation.w
        #     tab = transformations.euler_from_quaternion([w,x,y,z], "szyx")
        #     tab = map(lambda x: x/pi*180., tab)
            orientations[joints_map[joint]] = tab
        return orientations

    def compute_yaw(self, index):
        x = self.orientations[index].Orientation.x
        y = self.orientations[index].Orientation.y
        z = self.orientations[index].Orientation.z
        w = self.orientations[index].Orientation.w
        yaw = asin(2 * ((w * y) - (x * z))) / pi * 180.0
        return yaw

    def compute_pitch(self, index):
        x = self.orientations[index].Orientation.x
        y = self.orientations[index].Orientation.y
        z = self.orientations[index].Orientation.z
        w = self.orientations[index].Orientation.w
        pitch = atan2(2 * ((y * z) + (w * x)), 1-2*((x*x) + (y*y))) / pi * 180.0
        return pitch

    def compute_roll(self, index):
        x = self.orientations[index].Orientation.x
        y = self.orientations[index].Orientation.y
        z = self.orientations[index].Orientation.z
        w = self.orientations[index].Orientation.w
        roll = atan2(2 * ((x * y) + (w * z)), 1-2*((y*y)+(z*z))) / pi * 180.0
        return roll

    # def compute_yaw(self, index):
    #     x = self.orientations[index].Orientation.x
    #     y = self.orientations[index].Orientation.y
    #     z = self.orientations[index].Orientation.z
    #     w = self.orientations[index].Orientation.w
    #     yaw = asin(2 * ((w * x) + (y * z))) / pi * 180.0
    #     return yaw
    #
    # def compute_pitch(self, index):
    #     x = self.orientations[index].Orientation.x
    #     y = self.orientations[index].Orientation.y
    #     z = self.orientations[index].Orientation.z
    #     w = self.orientations[index].Orientation.w
    #     pitch = atan2(2 * ((w * z) - (x * y)), 1-2*((w*w) + (y*y))) / pi * 180.0
    #     return pitch
    #
    # def compute_roll(self, index):
    #     x = self.orientations[index].Orientation.x
    #     y = self.orientations[index].Orientation.y
    #     z = self.orientations[index].Orientation.z
    #     w = self.orientations[index].Orientation.w
    #     roll = atan2(2 * ((x * z) - (w * y)), 1-2*((x*x)+(y*y))) / pi * 180.0
    #     return roll