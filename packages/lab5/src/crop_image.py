#!/usr/bin/env python3

import sys
import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class Homework8:
    def __init__(self):
        # Instatiate the converter class once by using a class member
        self.bridge = CvBridge()
        rospy.Subscriber("image", Image, self.callback)
        # publishes to image_cropped topic
        self.pub_crop = rospy.Publisher("image_cropped", Image, queue_size=10)
        # publishes to image_white topic
        self.pub_white = rospy.Publisher("image_white", Image, queue_size=10)
        # publishes to image_yellow topic
        self.pub_yellow = rospy.Publisher("image_yellow", Image, queue_size=10)

    
    def callback(self, msg):
        # convert to a ROS image using the bridge
        cv_img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        
        # crop the image to 50%(vertically). Also note that decimal shape[0] is converted to integer
        cv_cropped = cv_img[int(cv_img.shape[0]/2):cv_img.shape[0], 0:cv_img.shape[1]]

        # convert colorspace from BGR to HSV
        hsv_img = cv2.cvtColor(cv_cropped, cv2.COLOR_BGR2HSV)

        #filter the color from the HSV image
        
        # convert new image to ROS to send
        ros_cropped = self.bridge.cv2_to_imgmsg(cv_cropped, "bgr8")

        
        # publish the cropped image
        self.pub_crop.publish(ros_cropped)

        # filter white lane
        white_filter = cv2.inRange(hsv_img, (0,0,225),(180,40,255))

        ros_white = self.bridge.cv2_to_imgmsg(white_filter, "mono8")

        # publish white filtered image
        self.pub_white.publish(ros_white) 

        # filter yellow lane
        yellow_filter = cv2.inRange(hsv_img, (0,150,180),(70,255,255))

        ros_yellow = self.bridge.cv2_to_imgmsg(yellow_filter, "mono8") 
 
        # publish yellow filtered image
        self.pub_yellow.publish(ros_yellow)     

if __name__=="__main__":
    # initialize our node and create a publisher as normal
    rospy.init_node("homework8_node", anonymous=True)
    hw8 = Homework8()
    rospy.spin()
