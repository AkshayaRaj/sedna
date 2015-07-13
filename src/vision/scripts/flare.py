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

lowThresh=np.array([0,0,0])

highThresh=np.array([0,0,0])

cv2.namedWindow('image')

class Buoys:

    bridge=None

    lowThresh=np.array([0,0,0])

    highThresh=np.array([0,0,0])

    screen={'width':320,'height':240}

    image=None

    circleParams = {'minRadius':13, 'maxRadius': 0 }

    houghParams = (74, 11)

    allCentroidList = []

    allAreaList = []

    allRadiusList = []

    blur=True

    minContourArea = 250

    previousArea=None

    previousCentroid=None

    # Keep track of the previous centroids for matching

    previousCentroid = None

    previousArea = None

    found=None

    def rosimg2cv(self,ros_img):

        try:

            frame=self.bridge.imgmsg_to_cv2(ros_image,ros_image.encoding)

        except CvBridgeError as e:

            rospy.logerr(e)

            rospy.loginfo("CvBridge error")

        return frame

    def __init__(self):

        self.dyn_reconfigure_server=DynServer(Config,self.reconfigure)

        #image= np.zeros((320,512,3), np.uint8)

        self.imgData={'detected':False}

        self.bridge=CvBridge()

        self.camera_topic=rospy.get_param('~image',        '/sedna/camera/front/image_raw')
        self.image_filter_pub=rospy.Publisher("/vision/buoy/image_filter",Image)

        self.buoy_pub=rospy.Publisher("/buoy",buoy)
        self.image_1_pub=rospy.Publisher("/vision/buoy/thresh",Image)
        #self.image_2_pub=rospy.Publisher("/vision/image_2",Image)
        #    self.image_3_pub=rospy.Publisher("/vision/image_3",Image)

        self.register()

        self.previousCentroid=(-1,-1)

        self.previousArea=0

    	#self.found=False

      	self .buoy_msg=buoy()
        self.missions_msg=missions()

    def reconfigure(self,config,level):

        rospy.loginfo('Reconfigure request !')

       # self.lowThresh[0]=config['loL']

        self.lowThresh[1]=config['hiGreen']

        self.lowThresh[2]=config['hiRed']

        #self.highThresh[0]=config['hiL']

        self.highThresh[1]=config['loGreen']

        self.highThresh[2]=config['loRed']

        self.minContourArea=config['minContourArea']

        #self.blur=config['blur']

        print "configured"

        return config

    def circles(self,cv_image):


        cv_image=cv2.resize(cv_image,dsize=(self.screen['width'],self.screen['height']))
        print "im in"
        #if self.blur:

        #    cv_image=cv2.GaussianBlur(cv_image,ksize=[5,5],sigmaX=0)

        #Added Code

        ''' inB, inG, inR = cv2.split(cv_image)

        avgR = np.mean(inR)

        avgG = np.mean(inG)

        avgB = np.mean(inB)

        avgGray = np.mean((avgB, avgG, avgR))

        if avgB == 0:

            outB = inB

        else:

            outB = (avgGray/avgB)*inB

        if avgG == 0:

            outG = inG

        else:

            outG = (avgGray/avgG)*inG

        if avgR == 0:

            outR = inR

        else:

            outR = (avgGray/avgR)*inR

        maxRGB = (np.max(outR), np.max(outG), np.max(outB))

        factor = np.max(maxRGB)

        if factor > 1:

            outR = 255*outR/factor

            outG = 255*outG/factor

            outB = 255*outB/factor

        outImg = cv2.merge((np.uint8(outB), np.uint8(outG), np.uint8(outR))) '''


        self.image=cv_image.copy()
        channels=cv2.split(cv_image)

        channels[0] = cv2.equalizeHist(channels[0])

        channels[1] = cv2.equalizeHist(channels[1])

        #channels[2] = cv2.equalizeHist(channels[2])

        img = cv2.merge(channels, cv_image)

        img=cv2.bilateralFilter(img, -1, 5, 0.1)

        kern = cv2.getStructuringElement(cv2.MORPH_RECT, (5,5))

        img=cv2.morphologyEx(img, cv2.MORPH_CLOSE, kern)

        hsvImg=cv2.cvtColor(img,cv2.COLOR_BGR2HSV)

        luvImg=cv2.cvtColor(img,cv2.COLOR_BGR2LUV)

        gauss = cv2.GaussianBlur(luvImg, ksize=(5,5), sigmaX=10)

        sum = cv2.addWeighted(luvImg, 1.5, gauss, -0.6, 0)

        enhancedImg = cv2.medianBlur(sum, 3)

        ch=cv2.split(enhancedImg) #print "value",self.highThresh[2]

        mask = cv2.inRange(ch[2],self.highThresh[2],self.lowThresh[2])

   #     mask1=cv2.inRange(ch[1],self.highThresh[0],self.lowThresh[0])

        mask2=cv2.inRange(ch[2],self.highThresh[1],self.lowThresh[1])
        mas=mask.copy()
        mas2=mask2.copy()

        #	mas1=mask1.copy() mas2=mask2.copy()

        #ADDED mask_out=cv2.cvtColor(mask,cv2.COLOR_GRAY2BGR)

        if(self.missions_msg.buoy_red==True):

            self.cir(mas,cv_image,1)
            print "publishwa1franci"

        if(self.missions_msg.buoy_green==True):

            print "publishwa"
            self.cir(mas2,cv_image,2)

#        self.cir(mas1)

 #       self.cir(mas2)

       # cv2.imshow(mask)

        #cv2.imshow(mask1)

        #cv2.imshow(mask2)

        mask_out=cv2.cvtColor(mask,cv2.COLOR_GRAY2BGR)

        mask_out1=cv2.cvtColor(mask2,cv2.COLOR_GRAY2BGR) #
       # mask_out2=cv2.cvtColor(mas2,cv2.COLOR_GRAY2BGR) #
       # mask_out3=cv2.cvtColor(mask,cv2.COLOR_GRAY2BGR)

        try:



            self.image_filter_pub.publish(self.bridge.cv2_to_imgmsg(self.image,encoding="bgr8"))
            if(self.missions_msg.buoy_red==True):
                self.image_1_pub.publish(self.bridge.cv2_to_imgmsg(mask_out,encoding="bgr8"))
            if(self.missions_msg.buoy_green==True):
                self.image_1_pub.publish(self.bridge.cv2_to_imgmsg(mask_out1,encoding="bgr8"))

            self.buoy_pub.publish(self.buoy_msg);

            #       self.image_2_pub.publish(self.bridge.cv2_to_imgmsg(mask_out2,encoding="bgr8"))
             # self.image_3_pub.publish(self.bridge.cv2_to_imgmsg(mask_out3,encoding="bgr8"))
        except CvBridgeError as e:

            rospy.logerr(e)

    # ADDED FUNCTION

    def cir(self,im,cv_image,dis):

        contours, hierachy = cv2.findContours(im, cv2.RETR_EXTERNAL,

                                              cv2.CHAIN_APPROX_NONE)

        #cv2.drawContours(mask, contours,-1, (0,255,0), 3)

        contours = filter(lambda c: cv2.contourArea(c) >self.minContourArea,
        contours)

        contours = sorted(contours, key=cv2.contourArea, reverse=True)

        # Sort by largest contour

    	if len(contours) > 0 :

             if(dis==1):

                 self.buoy_msg.possible_red=True
             if(dis==2):

    	         self.buoy_msg.possible_green=True

        else :
            if(dis==1):
                self.buoy_msg.possible_red=False
            if(dis==2):
                self.buoy_msg.possible_green=False

        if len(contours) > 0:

            largestContour = contours[0]

            mu = cv2.moments(largestContour)

            muArea = mu['m00']

            centroidToBump = (int(mu['m10']/muArea), int(mu['m01']/muArea))

            d=(int(mu['m10']/muArea))


            cv2.circle(self.image,centroidToBump,2,(0,255,0),3)
            #for yaw value

            b=math.radians(30)

            y=math.tan(b)

            w=self.screen['width']/2

            x=d-w

            #print("offset",x)

            fin=math.atan(y*x/w)
            fin=math.degrees(fin)
            cv2.putText(self.image,str(fin), (30, 30),cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255))

            if(dis==2):
                self.buoy_msg.y_offset_green =(float)((-0.65)*(self.screen['width']/2 -centroidToBump[0]))
                self.buoy_msg.x_offset_green=float(fin)



            if(dis==1):
                self.buoy_msg.x_offset_red=float(fin)

                self.buoy_msg.y_offset_red=(float)((-0.65)*(self.screen['width']/2 -centroidToBump[0]))



            #CENTER POINT VALUES

	''' #rectArea = muArea

            previousCentroid = centroidToBump

            #previousArea = rectArea

        else:

        # Find hough circles

            circles = cv2.HoughCircles(binImg, cv2.cv.CV_HOUGH_GRADIENT, 1,

                                   minDist=30, param1=houghParams[0],

                                   param2=houghParams[1],

                                   minRadius = circleParams['minRadius'],

                                   maxRadius = circleParams['maxRadius'])

                # Check if center of circles inside contours

            if contours is not None:

                for contour in contours:

                    mu = cv2.moments(contour)

                    muArea = mu['m00']

                    centroid = (mu['m10']/muArea, mu['m01']/muArea)

                    if circles is not None:

                        for circle in circles[0,:,:]:

                             circleCentroid = (circle[0], circle[1])

                             if
                             abs(math.hypot(centroid[0]-circlecentroid[0],centroid[1]-circlecentroid[1])) <
                             circle[2]:

                                 fb = True

                                    # Find new centroid by averaging the
                                    # centroid and circle centroid

                             newCentroid =(int(centroid[0]+circleCentroid[0])/2,


                                              int(centroid[1]+circleCentroid[1])/2)

                             allCentroidList.append(newCentroid)

                             allAreaList.append(cv2.contourArea(contour))

                             allRadiusList.append(circle[2])

                             # Draw circles

                             cv2.circle(img, newCentroid, circle[2], (255, 255,
                             0), 2)

                             cv2.circle(img, newCentroid, 2, (255, 0, 255), 3)

                 # Find the circle with the largest radius

                    if not len(allCentroidList) == 0:

                        maxIndex = allRadiusList.index(max(allRadiusList))

                        centroidToBump = allCentroidList[maxIndex]

                        rectArea = allAreaList[maxIndex]

                        previousCentroid = centroidToBump

                        previousArea = rectArea

                 '''

    def register(self):

#
#        self.image_sub=rospy.Subscriber(self.camera_topic,Image,self.cameraCallback)


        self.image_sub=rospy.Subscriber(self.camera_topic,Image,self.cameraCallBack)
        self.missions_sub=rospy.Subscriber("/missions",missions,self.missionsCallBack)
        rospy.loginfo("Subscribed to front camera")

        rospy.loginfo(self.camera_topic)

    def unregister(self):

        self.image_sub.unregister()

        rospy.loginfo("Unregistered front camera")

    def missionsCallBack(self,msg):

        self.missions_msg=msg

    def cameraCallBack(self,ros_image):

        try:

            frame=self.bridge.imgmsg_to_cv2(ros_image,ros_image.encoding)

        except CvBridgeError as e:

            rospy.logerr(e)

            rospy.loginfo("CvBridge error")

        if(self.missions_msg.buoy_red==True):

            self.circles(frame)
        if(self.missions_msg.buoy_green==True):
            self.circles(frame)

if __name__=="__main__":

    rospy.init_node("buoy_detector")

    buoys=Buoys()

    rospy.spin()
