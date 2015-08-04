__author__ = 'Angeall'
from pykinect2 import PyKinectV2
from pykinect2 import PyKinectRuntime


class KinectHandler(object):
    def __init__(self):
        self.kinect = PyKinectRuntime.PyKinectRuntime(PyKinectV2.FrameSourceTypes_Color |
                                                      PyKinectV2.FrameSourceTypes_Body)
        self.bodies = None

    def close(self):
        self.kinect.close()