__author__ = 'Angeall'
import pykinectnao.sensors.implemented.kinecthandler as kinecthandler
import pykinectnao.avatars.implemented.naocommander as naocommander
from time import sleep
import pykinectnao.sensors.sensor as sensor

robotIP = "192.168.2.24"
nb_of_body = 1

def main():
    kinect_h = kinecthandler.KinectHandler("naocommander")
    nao_c = naocommander.NAOCommander(robotIP)
    while True:
        res = kinect_h.get_movement(nb_of_body)
        if res == sensor.NO_DATA:
          continue
        for i in range(nb_of_body):
            print "Body no", i
            print res[i][0]
            print res[i][1]
        sleep(0.1)

main()
