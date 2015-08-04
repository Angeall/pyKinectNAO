__author__ = 'Angeall'
import pykinectnao.sensors.implemented.kinecthandler as kinecthandler
import pykinectnao.avatars.implemented.naocommander as naocommander
from time import sleep

robotIP = "192.168.2.24"


def main():
    kinect_h = kinecthandler.KinectHandler("naocommander")
    nao_c = naocommander.NAOCommander(robotIP)
    #kinect_h.wait_for_data()
    while True:
        res = kinect_h.get_movement()[0]
        if not res == -1:
            print res
        sleep(0.1)

main()
