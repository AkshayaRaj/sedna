#!/usr/bin/env python

import roslib; roslib.load_manifest('vision')

import cv2

import math

import numpy as np

import rospy

from std_msgs.msg import String

from sensor_msgs.msg import Image

from cv_bridge import CvBridge, CvBridgeError

from dynamic_reconfigure.server import Server as DynServer

from srmauv_msgs.msg import  *

from srmauv_msgs.srv import *

import vision.cfg.flareConfig as Config

from unittest import signals

from OpenGL.raw.GL.SGIX import framezoom

import signal

class image_converter:



    def __init__(self):
       self.image_1_pub = rospy.Publisher("image_topic_2",Image)
       self.rate=rospy.Rate(30)
       cv2.namedWindow("Image window", 1)
       self.bridge = CvBridge()
       self.cir()
       # self.image_sub = rospy.Subscriber("image_topic",Image,self.callback)


    def cir(self):
        cap=cv2.VideoCapture('/tmp/lc.mp4')
        while(cap.isOpened()):
            ret,frame=cap.read()
            #cv2.waitKey(0)
            if cv2.waitKey(1) & 0xFF==ord('q'):
                break
            self.rate.sleep()
            try:
                self.image_1_pub.publish(self.bridge.cv2_to_imgmsg(frame,encoding="bgr8"))
            except CvBridgeError, e:
                print e

        cap.release()
        cv2.destroyAllWindows()


    #def main(args):


if __name__ == '__main__':

    #main(sys.argv)

    rospy.init_node('image_converter234', anonymous=True)
    ic = image_converter()
    try:
      rospy.spin()
    except KeyboardInterrupt:

        print "Shutting down"
    cv2.destroyAllWindows()

'''

import roslib; roslib.load_manifest('vision')

import cv2

import math

import numpy as np

import rospy

from std_msgs.msg import String

from sensor_msgs.msg import Image

from cv_bridge import CvBridge, CvBridgeError

from dynamic_reconfigure.server import Server as DynServer

from srmauv_msgs.msg import  *

from srmauv_msgs.srv import *

import vision.cfg.flareConfig as Config

from unittest import signals

from OpenGL.raw.GL.SGIX import framezoom

import signal


rospy.init_node("buoy_detector")
rospy.spin()
cap=cv2.VideoCapture('/tmp/lc.mp4')
bridge=CvBridge()
image_1_pub=rospy.Publisher('/sedna/camera/front/image_ra1',Image)
while(cap.isOpened()):
    ret,frame=cap.read()
    #cv2.waitKey(0)
    image_1_pub.publish(bridge.cv2_to_imgmsg(frame,encoding="bgr8"))

    gray=cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)
    #cv2.imshow('frame',gray)
    if cv2.waitKey(1) & 0xFF==ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
'''
