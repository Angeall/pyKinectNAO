__author__ = 'Angeall'

# An avatar should have the following dicts to define the motors he needs and the positions he needs

# This dict links the motors (constants of joints.py) needed
#   with the tab index (position of the motor in the motors list)
#   and the motors specs (yaw, pitch, roll)
# This dict has to follow this format :
#   {jointsModuleConstant: (listIndexOfYourChoice, [Yaw, Pitch, Roll]), ...}
#   e.g. {..., joints.ELBOW_LEFT: (0, [True, False, True]), ...} as the left elbow can yaw and roll, but not pitch
motors_needed = {}

# This dict should link the positions needed with list index of your choice.
#   be careful if the camera you use can not return this position
#   => you should check if the index in your list is not None
positions_needed = {}


class Avatar(object):
    def __init__(self):
        # device should be the connection to your robot
        self.device = None
        # motors should be the motors commands you asked from the camera
        #   it should contain lists of absolute degrees [Yaw, Pitch, Roll]
        self.motors = None
        # positions should be the positions list you asked from the camera
        self.positions = None