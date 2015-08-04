__author__ = 'Angeall'

from pykinect2 import PyKinectV2
from pykinect2.PyKinectV2 import *
from pykinect2 import PyKinectRuntime
from time import sleep

kinect = PyKinectRuntime.PyKinectRuntime(PyKinectV2.FrameSourceTypes_Color | PyKinectV2.FrameSourceTypes_Body)
bodies = None


def run():
    global bodies
    if kinect.has_new_body_frame():
        bodies = kinect.get_last_body_frame()
    if bodies is not None:
        for i in range(0, kinect.max_body_count):
            body = bodies.bodies[i]
            if not body.is_tracked:
                continue
            joints = body.joints
            # convert joint coordinates to color space
            orientation = body.joint_orientations[PyKinectV2.JointType_ElbowRight].Orientation
            print i
            print joints[PyKinectV2.JointType_Head].Position.x, joints[PyKinectV2.JointType_Head].Position.y, joints[PyKinectV2.JointType_Head].Position.z
            print orientation.x, orientation.y, orientation.z, orientation.w

run()
for i in range(250):
    sleep(0.5)
    run()
kinect.close()