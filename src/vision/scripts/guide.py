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

import vision.cfg.lineConfig as Config

from unittest import signals

from OpenGL.raw.GL.SGIX import framezoom

import signal

lowThresh=np.array([0,0,0])

highThresh=np.array([0,0,0])

#cv2.namedWindow('image')

class Line:

    bridge=None

    lowThresh=np.array([0,0,0])

    highThresh=np.array([0,0,0])

    screen={'width':320,'height':240}

    image=None

    '''

    circleParams = {'minRadius':13, 'maxRadius': 0 }

    houghParams = (74, 11)

    allCentroidList = []

    allAreaList = []

    allRadiusList = []

    minContourArea = 250

    previousArea=None

    previousCentroid=None

    '''

    blur=True

    testing = False

    val=1

    thval = 30

    upperThresh = 70

    areaThresh = 3000

    upperAreaThresh = 100000

    angle=0

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

#        signal.signal(signal.SIGINT,self.userQuit)

        self.imgData={'detected':False}

        self.bridge=CvBridge()

        self.camera_topic=rospy.get_param('~image',
        '/sedna/camera/bottom/image_raw')

        self.image_filter_pub=rospy.Publisher("/Vision/image_filter",Image)

	self.image_thresh_pub=rospy.Publisher("/Vision/image_thresh",Image)

	self.line_pub=rospy.Publisher("/line_follower",line)

        self.register()

        self.previousCentroid=(-1,-1)

        self.previousArea=0

    	#self.found=False

	self.lineMsg=line()

	self.lineMsg.possible=False

	self.lineMsg.heading=0

	self.lineMsg.distance=0

    def reconfigure(self,config,level):

        rospy.loginfo('Reconfigure request !')

        self.val = config['val']

        self.upperThresh =config['upperThresh']

        self.areaThresh = config['areaThresh']

        self.upperAreaThresh = config['upperAreaThresh']

        self.lowThresh=config['loOrange']

        self.highThresh=config['hiOrange']

        self.blur=config['blur']

        return config

    def userQuit(self,signal,frame):

        rospy.loginfo("Buoy server is shutting down")

    def circles(self,cv_image):


        cv_image=cv2.resize(cv_image,dsize=(self.screen['width'],self.screen['height']))

        #if self.blur:

        #    cv_image=cv2.GaussianBlur(cv_image,ksize=[5,5],sigmaX=0)

        #Added Code

        '''

        inB, inG, inR = cv2.split(cv_image)

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

        outImg = cv2.merge((np.uint8(outB), np.uint8(outG), np.uint8(outR)))

	'''

	self.image=cv_image.copy()

	out=cv_image.copy();

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

        ch=cv2.split(enhancedImg)

	#print "value",self.highThresh[2]

        mask = cv2.inRange(ch[1],self.lowThresh,self.highThresh)

   #     mask1=cv2.inRange(ch[1],self.highThresh[0],self.lowThresh[0])

  #      mask2=cv2.inRange(ch[2],self.highThresh[1],self.lowThresh[1])

	mas=mask.copy()

        #ADDED

	mask_out=cv2.cvtColor(mask,cv2.COLOR_GRAY2BGR)

        out = cv2.cvtColor(grayImg, cv2.cv.CV_GRAY2BGR)

        # Find centroid and bounding box

        pImg = mask.copy()

        contours, hierachy = cv2.findContours(pImg, cv2.RETR_LIST,
        cv2.CHAIN_APPROX_SIMPLE)

        maxArea = 0

        for contour in contours:

            area = cv2.contourArea(contour)

            if area > self.areaThresh and area < self.upperAreaThresh and area > maxArea:

                #Find the center using moments

                mu = cv2.moments(contour, False)

                centroidx = mu['m10'] / mu['m00']

                centroidy = mu['m01'] / mu['m00']

                maxArea = area

                centroid1 = centroidx

                centroid2 = centroidy

                #rectData['rect'] = cv2.minAreaRect(contour)

                rect = cv2.minAreaRect(contour)

        if maxArea > 0:

        	self.lineMsg.possible=True

        else :

        	self.lineMsg.possible=False

        if maxArea > 0:

            #rectData['detected'] = True

            points = np.array(cv2.cv.BoxPoints(rect))

            #Find the blackline heading

            edge1 = points[1] - points[0]

            edge2 = points[2] - points[1]

            print "points1",points[1]

            print "points0",points[0]

            print "edge1",edge1

            #Choose the vertical edge

            if cv2.norm(edge1) > cv2.norm(edge2):

                edge1[1] = edge1[1] if edge1[1] != 0.0 else math.copysign(0.01,
                edge1[1])

                #rectData['angle'] = math.degrees(math.atan(edge1[0]/edge1[1]))

                angle = math.degrees(math.atan(edge1[0]/edge1[1]))

            else:

                edge2[1] = edge2[1] if edge2[1] != 0.0 else math.copysign(0.01,
                edge2[1])

                #rectData['angle'] = math.degrees(math.atan(edge2[0]/edge2[1]))

                angle = math.degrees(math.atan(edge2[0]/edge2[1]))

            #Chose angle to turn if horizontal

            if angle == 90:

                if centroid1 > screen['width'] / 2:

                    angle = -90

            elif angle == -90:

                if centroid1 < screen['width'] / 2:

                    angle = 90

	    self.lineMsg.heading=float(angle)

            #Testing

            centerx = int(centroid1)

            centery = int(centroid2)

            cv2.circle(out, (centerx, centery), 5, (0, 255, 0))

            for i in range(4):

                pt1 = (int(points[i][0]), int(points[i][1]))

                pt2 = (int(points[(i+1)%4][0]), int(points[(i+1)%4][1]))

                cv2.line(out, pt1, pt2, (255, 0, 0))

            cv2.putText(cv_image, str(angle), (30, 30),

                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255))

	    #grayImg=cv2.cvtColor(grayImg,cv2.COLOR_GRAY2BGR)

        try:

            self.image_filter_pub.publish(self.bridge.cv2_to_imgmsg(out,
            encoding="bgr8"))


            self.image_thresh_pub.publish(self.bridge.cv2_to_imgmsg(cv_image,encoding="bgr8"))

            self.line_pub.publish(self.lineMsg);

	except CvBridgeError as e:

            rospy.logerr(e)

    def register(self):

        #
        # self.image_sub=rospy.Subscriber(self.camera_topic,Image,self.cameraCallback)


        self.image_sub=rospy.Subscriber(self.camera_topic,Image,self.cameraCallback)

        rospy.loginfo("Subscribed to front camera")

        rospy.loginfo(self.camera_topic)

    def unregister(self):

        self.image_sub.unregister()

        rospy.loginfo("Unregistered front camera")

    def cameraCallback(self,ros_image):

        #rospy.loginfo("in cam")

        #cv_image=self.rosimg2cv(ros_image)

       # self.circles(cv_image)

        try:

            frame=self.bridge.imgmsg_to_cv2(ros_image,ros_image.encoding)

        except CvBridgeError as e:

            rospy.logerr(e)

            rospy.loginfo("CvBridge error")

        self.circles(frame)

if __name__=="__main__":

    rospy.init_node("line_detector")

    buoys=Line()

    rospy.spin()
