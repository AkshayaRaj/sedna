mport roslib; roslib.load_manifest('vision')

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

	self.flare_pub=rospy.Publisher("/flare",flare)

        self.camera_topic=rospy.get_param('~image',
        '/sedna/camera/front/image_raw')


        self.image_filter_pub=rospy.Publisher("/vision/flare/image_filter",Image)

	self.image_1_pub=rospy.Publisher("/vision/flare/thresh",Image)

#	self.image_2_pub=rospy.Publisher("/vision/image_2",Image)

#	self.image_3_pub=rospy.Publisher("/vision/image_3",Image)

        self.register()

        self.previousCentroid=(-1,-1)

        self.previousArea=0

    	#self.found=False

      	self.flare_msg=flare()

    def reconfigure(self,config,level):

        rospy.loginfo('Reconfigure request !')

        self.lowThresh[0]=config['hiRed']

        #self.lowThresh[1]=config['loU']

        self.lowThresh[2]=config['hiYellow']

        self.highThresh[0]=config['loRed']

        #self.highThresh[1]=config['hiU']

        self.highThresh[2]=config['loYellow']

        self.minContourArea=config['minContourArea']

        #self.blur=config['blur']

        print "configured"

        return config

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

            mask = cv2.inRange(ch[2],self.highThresh[2],self.lowThresh[2])

            mask1=cv2.inRange(ch[1],self.highThresh[0],self.lowThresh[0])

            mask2=cv2.inRange(ch[2],self.highThresh[1],self.lowThresh[1])

            mask3=cv2.inRange(ch[2],self.highThresh[1],self.lowThresh[1])

            mas=mask.copy()

            mas1=mask1.copy()

            mas2=mask2.copy()

            mas3=mask3.copy()

            #for red

            mask_out=cv2.cvtColor(mask,cv2.COLOR_GRAY2BGR)

            self.cir(mas,cv_image)

            con, hierachy = cv2.findContours(mask1, cv2.RETR_EXTERNAL,

                                              cv2.CHAIN_APPROX_NONE)

            # for yellow

            w=self.screen['width']/2

            ht=self.screen['height']/2

            if (len(con)>0):

                self.torpedo_msg.redfound=True

            else :

		self.torpedo_msg.redfound=False

            if (len(con)>0):

                con = filter(lambda c: cv2.contourArea(c) >minContourArea, con)

                con = sorted(con, key=cv2.contourArea, reverse=True) # Sort by
                largest contour

                lar=con[0]

                if(cv2.contourArea(lar)>2000):

                    mu = cv2.moments(con)

                    muA = mu['m00']

                    cex = (mu['m10']/muArea)

                    cey= (mu['m01']/muArea)

                    cv2.circle(img,cex,cey, 2, (255, 0, 255), 3)

                    self.torpedo_msg.yel_x_off=cex-w

                    self.torpedo_msg.yel_y_off=cey-ht

            # for lid

            con1, hierachy = cv2.findContours(mask3, cv2.RETR_EXTERNAL,

                                              cv2.CHAIN_APPROX_NONE)

            if (len(con1)>0):

                self.torpedo_msg.lidfound=True

            else :

		self.torpedo_msg.lidfound=False

            if (len(con1)>0):

                con2, hierachy = cv2.findContours(mask1, cv2.RETR_EXTERNAL,

                                              cv2.CHAIN_APPROX_NONE)

                # for purple handlle

                if (len(con2)>0):

                    self.torpedo_msg.redfound=True

                else :

                    self.torpedo_msg.redfound=False

                if (len(con2)>0):

                    con2 = filter(lambda c: cv2.contourArea(c) >minContourArea,
                    con2)

                    con2 = sorted(con2, key=cv2.contourArea, reverse=True) #
                    Sort by largest contour

                    lar=con2[0]

                    mu = cv2.moments(con)

                    muA = mu['m00']

                    cex = (mu['m10']/mu)

                    cey= (mu['m01']/muA)

                    cv2.circle(img,cex,cey, 2, (255, 0, 255), 3)

                    self.torpedo_msg.pur_x_off=cex-w

                    self.torpedo_msg.pur_y_off=cey-ht

                else :

                    con1 = filter(lambda c: cv2.contourArea(c) >minContourArea,
                    con1)

                    con1 = sorted(con1, key=cv2.contourArea, reverse=True) #
                    Sort by largest contour

                    lar=con1[0]

                    if(cv2.contourArea(lar)>2000):

                        mu = cv2.moments(con1)

                        muA = mu['m00']

                        cex = (mu['m10']/muA)

                        cey= (mu['m01']/muA)

                        cv2.circle(img,cex,cey, 2, (255, 0, 255), 3)

                        self.torpedo_msg.ora_x_off=cex-w

                        self.torpedo_msg.ora_y_off=cey-ht

#        self.cir(mas1)

 #       self.cir(mas2)

       # cv2.imshow(mask)

        #cv2.imshow(mask1)

        #cv2.imshow(mask2)

       # mask_out=cv2.cvtColor(mask,cv2.COLOR_GRAY2BGR)

#	mask_out1=cv2.cvtColor(mas1,cv2.COLOR_GRAY2BGR)

#	mask_out2=cv2.cvtColor(mas2,cv2.COLOR_GRAY2BGR)

#	mask_out3=cv2.cvtColor(mask,cv2.COLOR_GRAY2BGR)

        try:

            self.image_filter_pub.publish(self.bridge.cv2_to_imgmsg(self.image,
            encoding="bgr8"))

	    self.image_1_pub.publish(self.bridge.cv2_to_imgmsg(mask_out,
	    encoding="bgr8"))

	    self.flare_pub.publish(self.flare_msg);

	  #  self.image_2_pub.publish(self.bridge.cv2_to_imgmsg(mask_out2,
	  #  encoding="bgr8"))

         #   self.image_3_pub.publish(self.bridge.cv2_to_imgmsg(mask_out3,
         #   encoding="bgr8"))

        except CvBridgeError as e:

            rospy.logerr(e)

    # ADDED FUNCTION

    def cir(self,im,cv_image):

        contours, hierachy = cv2.findContours(im, cv2.RETR_EXTERNAL,

                                              cv2.CHAIN_APPROX_NONE)

        #cv2.drawContours(mask, contours,-1, (0,255,0), 3)

        contours = filter(lambda c: cv2.contourArea(c) >self.minContourArea,
        contours)

        contours = sorted(contours, key=cv2.contourArea, reverse=True) # Sort by
        largest contour

	if len(contours) > 0 :

		self.torpedo_msg.redfound=True

	else :

		self.torpedo_msg.redfound=False

        if len(contours) > 0:

        i=0

        for cont in contours:

            peri=cv2.arclength(cont,True)

            approx=cv2.approxPolyDP(cont,0.02*peri,True)

            if(len(approx)==4):

                mu = cv2.moments(cont)

                muArea = mu['m00']

                centroid = (int(mu['m10']/muArea), int(mu['m01']/muArea))

                d=(int(mu['m10']/muArea))

                cv2.circle(self.image,centroid,2,(0,255,0),3)

                if(i==0):

                    self.torpedo_msg.red0_x_off=centroid[0]-w

                    self.torpedo_msg.red0_y_off=centroid[1]-ht

                if(i==1):

                    self.torpedo_msg.red1_x_off=centroid[0]-w

                    self.torpedo_msg.red1_y_off=centroid[1]-ht

                if(i==2):

                    self.torpedo_msg.red2_x_off=centroid[0]-w

                    self.torpedo_msg.red2_y_off=centroid[1]-ht

                if(i==3):

                    self.torpedo_msg.red3_x_off=centroid[0]-w

                    self.torpedo_msg.red3_y_off=centroid[1]-ht

                i=i+1

            '''

            #for yaw value

            b=math.radians(30)

            y=math.tan(b)

            w=self.screen['width']/2

            ht=self.screen['height']/2

            x=d-w

            #print("offset",x)

            fin=math.atan(y*x/w)

	    fin=math.degrees(fin)

	    cv2.putText(self.image, str(fin), (30, 30),cv2.FONT_HERSHEY_SIMPLEX, 1, (0,
	    0, 255))

            #print ("final",fin)

            #print("degrees",math.degrees(fin))

            self.flare_msg.heading_offset=float(fin)

	    # CHANGED D SIGN

	    self.flare_msg.y_offset=(float)((-0.65)*(self.screen['width']/2 -
	    centroidToBump[0]))

            #CENTER POINT VALUES

            '''

    def register(self):

#
#        self.image_sub=rospy.Subscriber(self.camera_topic,Image,self.cameraCallback)


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

    rospy.init_node("flare_detector")

    buoys=Buoys()

    rospy.spin() 
