__author__ = 'Angeall'
import pykinectnao.sensors.implemented.kinecthandler as kinecthandler
import pykinectnao.avatars.implemented.naocommander as naocommander
from time import sleep
import pykinectnao.sensors.sensor as sensor

robotIP = "192.168.2.24"
PORT = 9559

nb_of_body = 1


def kinect_test(kinect_h):
    j = 0
    while True:
        res = kinect_h.get_movement(nb_of_body)
        if res == sensor.NO_DATA:
          continue
        for i in range(nb_of_body):
            # print "Body no", i
            print res[i][0]
            # print res[i][1]
        if j%5 == 0:
            if j == 0:
                print "-"*10, "BRAS LE LONG DU CORPS", "-"*10
            if j == 5:
                print "-"*10, "BRAS EN L'AIR", "-"*10
            if j == 10:
                print "-"*10, "BRAS HORIZONTAL DEVANT", "-"*10
            if j == 15:
                print "-"*10, "BRAS HORIZONTAL LATERAL", "-"*10
                sleep(3)
                kinect_h.close()
                break
            sleep(3)
        else:
            sleep(1)
        j += 1


def nao_test(nao_c):
    i=0
    while i < 1:
        nao_c.waveYourLeftHand()
        i += 1


def main():
    kinect_h = kinecthandler.KinectHandler("naocommander")
    #nao_c = naocommander.NAOCommander(robotIP, PORT)
    #nao_test(nao_c)
    kinect_test(kinect_h)
main()
