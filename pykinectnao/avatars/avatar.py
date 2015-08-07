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

# The syntax for this dict is the followinf :
#   {name_of_the_sensor_module: {joints_constant: [(YawRatio, Phi, YawAxis), same for Pitch, same for Roll]}}
#   where Yaw,Roll,PitchRatio is a ratio to multiply with the real value. (to adjust the motors values)
#   where Yaw,Rol, PitchPhi is the phase shift in degree of the axis to convert the sensor value to the avatar value
#   where Yaw,Roll,PitchAxis is "+" if the sensor axis is oriented the same way than the avatar, "-" otherwise
# See an example in "naocommander.py"
motors_converter = {}


class Avatar(object):
    def __init__(self, motors_converters=motors_converter):
        # device should be the connection to your robot
        self.device = None
        # motors should be the motors commands you asked from the camera
        #   it should contain lists of absolute degrees [Yaw, Pitch, Roll]
        self.motors = None
        # positions should be the positions list you asked from the camera
        self.positions = None
        # if we need converters camera->avatar for the [Yaw, Pitch, Roll], then you put it here
        self.motors_converters = motors_converters
