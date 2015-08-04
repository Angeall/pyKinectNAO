__author__ = 'Angeall'
from pykinect2 import PyKinectV2
from pykinect2 import PyKinectRuntime
from pykinectnao.sensors.sensor import *
import pykinectnao.avatars.joints as joints
from math import *

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


class KinectHandler(Sensor):
    def __init__(self, avatar_module_name):
        super(KinectHandler, self).__init__(avatar_module_name, joints_map)
        self.device = PyKinectRuntime.PyKinectRuntime(PyKinectV2.FrameSourceTypes_Color |
                                                      PyKinectV2.FrameSourceTypes_Body)
        self.bodies = None
        self.body = None

    def close(self):
        self.kinect.close()

    def get_movement(self):
        if self.device.has_new_body_frame():
            self.bodies = self.device.get_last_body_frame()
        if self.bodies is not None:
            for i in range(0, self.device.max_body_count):
                body = self.bodies.bodies[i]
                if not body.is_tracked:
                    continue
                joints = body.joints
                orientation = body.joint_orientations[PyKinectV2.JointType_ElbowRight].Orientation
                print joints[PyKinectV2.JointType_Head].Position.x, joints[PyKinectV2.JointType_Head].Position.y, joints[PyKinectV2.JointType_Head].Position.z
                print orientation.x, orientation.y, orientation.z, orientation.w
                self.body = body
                self.positions = self.body.joints
                self.orientations = self.body.joint_orientations
            return self.convert_orientation(), self.convert_positions()
        else:
            return -1, -1

    # Be sure to have the proper joint_map linked with self.joint_map
    # Be sure to have a non-empty list of positions
    # This method match the sensor joint_map with the avatar position_needed map
    # Override if needed
    def convert_positions(self):
        if self.positions is None:
            return []
        positions = self.position_pattern_list[:]
        for index in self.positions_conversion_map.keys():
            positions[self.positions_conversion_map[index]] = [self.positions[index].Position.x,
                                                               self.positions[index].Position.y,
                                                               self.positions[index].Position.z]
            return positions

    # Be sure to have the proper joint_map linked with self.joint_map
    # Be sure to have a non-empty list of orientations
    # This method match the sensor joint_map with the avatar motors_needed map
    # Override if needed
    def convert_orientation(self):
        if self.orientations is None:
            return []
        orientations = self.orientation_pattern_list[:]
        for index in self.orientations_conversion_map.keys():
            # If it can yaw
            if self.orientations_conversion_map[index][1][0]:
                self.orientations[index][0] = self.compute_yaw(index)
            # If it can pitch
            if self.orientations_conversion_map[index][1][1]:
                self.orientations[index][1] = self.compute_pitch(index)
            # If it can roll
            if self.orientations_conversion_map[index][1][2]:
                self.orientations[index][2] = self.compute_roll(index)
            orientations[self.orientations_conversion_map[index][0]] = self.orientations[index]
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
        pitch = atan2(2 * ((y * z) + (w * x)), (w * w) - (x * x) - (y * y) + (z * z)) / pi * 180.0
        return pitch

    def compute_roll(self, index):
        x = self.orientations[index].Orientation.x
        y = self.orientations[index].Orientation.y
        z = self.orientations[index].Orientation.z
        w = self.orientations[index].Orientation.w
        roll = atan2(2 * ((x * y) + (w * z)), (w * w) + (x * x) - (y * y) - (z * z)) / pi * 180.0
        return roll

