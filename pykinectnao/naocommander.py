__author__ = 'Angeall'

import joints
import motion
from naoqi import ALProxy

# Follows the avatar needs : joints.(...): (index, [Yaw, Pitch, Roll])
motors_needed = {  # joints.SHOULDER_RIGHT: (0, [False, True, True]),}
                   # Iinitially ELBOW_RIGHT : (1, [False, True, True]),
                   joints.ELBOW_RIGHT: (0, [True, True, True]), }
                   # joints.HAND_RIGHT: (2, [True, False, False]),
                   # joints.SHOULDER_LEFT: (3, [False, True, True]),
                   # joints.ELBOW_LEFT: (4, [True, False, True]),
                   # joints.HAND_LEFT: (5, [True, False, False]),
                   # joints.HIP_RIGHT: (6, [True, True, True]),
                   # joints.KNEE_RIGHT: (7, [False, True, False]),
                   # joints.FOOT_RIGHT: (8, [False, True, True]),
                   # joints.HIP_LEFT: (9, [True, True, True]),
                   # joints.KNEE_LEFT: (10, [False, True, False]),
                   # joints.FOOT_LEFT: (11, [False, True, True])}

# For the movement, we need two positions: one around the hip center, one around the neck
#   these two positions will allow to make sure the person in front of the camera as moved his whole body
#   so we can make NAO walk to the new position (other joints could be used in order to do that)
positions_needed = {joints.NECK: 0,
                    joints.SPINE_BASE: 1}

# The syntax for this dict is the followinf :
#   {name_of_the_sensor_module: {joints_constant: [(YawRatio, YawXPhi, YawYPhi, YawAxis), same for Pitch and Roll]}}
#   where Yaw,Roll,PitchRatio is a ratio to multiply with the real value. (to adjust the motors values)
#   where Yaw,Roll, PitchXPhi is the phase shift in degree of the x axis to convert the sensor value to the avatar value
#   where Yaw,Roll, PitchYPhi is the phase shift in degree of the y axis to convert the sensor value to the avatar value
#   where Yaw,Roll,PitchAxis is "+" if the sensor axis is oriented the same way than the avatar, "-" otherwise
motors_converters = {"kinecthandler":
                        {joints.ELBOW_RIGHT: [(1, 0, "+", "+"), (1., +90, "+", "-"), (1., +0, "+", "+")]}}


class NAOCommander():
    def __init__(self, robotIP, PORT):
        motionproxy = ALProxy("ALMotion", robotIP, PORT)
        postureproxy = ALProxy("ALRobotPosture", robotIP, PORT)
        # Wake up robot
        motionproxy.wakeUp()

        self.device = motionproxy
        self.postureProxy = postureproxy

    # TODO move with motors and positions
    # def move(self):

    def user_left_arm_articular(self, shoulder_pitch=87, shoulder_roll=0, elbow_yaw=-70,
                                elbow_roll=-34, wrist_yaw=0., hand=0.28, pfractionmaxspeed=0.6):
        # Arms motion from user have always the priority than walk arms motion
        jointnames = ["LShoulderPitch", "LShoulderRoll", "LElbowYaw", "LElbowRoll", "LWristYaw", "LHand"]
        arm1 = [shoulder_pitch, shoulder_roll, elbow_yaw, elbow_roll, wrist_yaw]
        arm1 = [x * motion.TO_RAD for x in arm1]
        # The hand is not in degree, we need to add it after the conversion
        arm1.append(hand)

        self.device.angleInterpolationWithSpeed(jointnames, arm1, pfractionmaxspeed)

    def wave_your_left_hand(self):
        # Arms motion from user have always the priority than walk arms motion
        jointnames = ["LShoulderPitch", "LShoulderRoll", "LElbowYaw", "LElbowRoll", "LHand"]
        arm1 = [-90, 55, -6, -3.]
        arm1 = [x * motion.TO_RAD for x in arm1]
        # The hand is not in degree, we need to add it after the conversion
        arm1.append(0.98)

        arm2 = [-90, 55, -6, -85.]
        arm2 = [x * motion.TO_RAD for x in arm2]
        # The hand is not in degree, we need to add it after the conversion
        arm2.append(0.98)

        arm0 = [87, 0, -70, -34.]
        arm0 = [x * motion.TO_RAD for x in arm0]
        # The hand is not in degree, we need to add it after the conversion
        arm0.append(0.28)

        pfractionmaxspeed = 0.6

        self.device.angleInterpolationWithSpeed(jointnames, arm1, pfractionmaxspeed)
        self.device.angleInterpolationWithSpeed(jointnames, arm2, pfractionmaxspeed)
        self.device.angleInterpolationWithSpeed(jointnames, arm1, pfractionmaxspeed)
        self.device.angleInterpolationWithSpeed(jointnames, arm2, pfractionmaxspeed)
        self.device.angleInterpolationWithSpeed(jointnames, arm0, pfractionmaxspeed)

    def user_right_arm_articular(self, shoulder_pitch=87, shoulder_roll=0, elbow_yaw=70,
                                 elbow_roll=34, wrist_yaw=0., hand=0.28, pfractionmaxspeed=0.6):
        if shoulder_pitch > 115:
            shoulder_pitch = 115
        if shoulder_pitch < -117:
            shoulder_pitch = -117
        if shoulder_roll > 5:
            shoulder_roll = 5
        if shoulder_roll < -65:
            shoulder_roll = -65

        # Arms motion from user have always the priority than walk arms motion
        jointnames = ["RShoulderPitch", "RShoulderRoll", "RElbowYaw", "RElbowRoll", "RWristYaw", "RHand"]
        arm1 = [shoulder_pitch, shoulder_roll, elbow_yaw, elbow_roll, wrist_yaw]
        arm1 = [x * motion.TO_RAD for x in arm1]
        # The hand is not in degree, we need to add it after the conversion
        arm1.append(hand)

        self.device.angleInterpolationWithSpeed(jointnames, arm1, pfractionmaxspeed)
