__author__ = 'Angeall'

import pykinectnao.avatars.joints as joints
from pykinectnao.avatars.avatar import Avatar

# Follows the avatar needs : joints.(...): (index, [Yaw, Pitch, Roll])
motors_needed = {joints.HEAD: (0, [True, True, False]),
                 joints.SHOULDER_RIGHT: (1, [False, True, True]),
                 joints.ELBOW_RIGHT: (2, [True, False, True]),
                 joints.HAND_RIGHT: (3, [True, False, False]),
                 joints.SHOULDER_LEFT: (4, [False, True, True]),
                 joints.ELBOW_LEFT: (5, [True, False, True]),
                 joints.HAND_LEFT: (6, [True, False, False]),
                 joints.HIP_RIGHT: (7, [True, True, True]),
                 joints.KNEE_RIGHT: (8, [False, True, False]),
                 joints.FOOT_RIGHT: (9, [False, True, True]),
                 joints.HIP_LEFT: (10, [True, True, True]),
                 joints.KNEE_LEFT: (11, [False, True, False]),
                 joints.FOOT_LEFT: (12, [False, True, True])}

# For the movement, we need two positions: one around the hip center, one around the neck
#   these two positions will allow to make sure the person in front of the camera as moved his whole body
#   so we can make NAO walk to the new position (other joints could be used in order to do that)
positions_needed = {joints.NECK: 0,
                    joints.SPINE_BASE: 1}


class NAOCommander(Avatar):
    def __init__(self):
        # TODO : Proxy connection to NAO
        self.device = None

    # TODO move with motors and positions
    # def move(self):

