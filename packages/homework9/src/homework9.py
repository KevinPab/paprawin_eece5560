#!/usr/bin/env python3

import sys
import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import message_filters
import numpy as np

#Note: this node utilizes message_filers to synchronize multiple topics to be used in the same callback function

class Homework9:
    def __init__(self):
        # Instantiate the converter class once by using a class member
        self.bridge = CvBridge()

        # subscribes to the cropped img from hw8
        self.sub_cropped = message_filters.Subscriber("image_cropped", Image)

        # subscribes to image_yellow topic
        self.sub_yellow = message_filters.Subscriber("image_yellow", Image)

        # subscribes to image_white topic
        self.sub_white = message_filters.Subscriber("image_white", Image)

        # use time synchronizer to analyze 3 messages in one callback function
        self.ts = message_filters.TimeSynchronizer([self.sub_cropped, self.sub_yellow, self.sub_white], 10)

        # register the time synchronizer to the callback function
        self.ts.registerCallback(self.callback)

        # publishes final white filtered image
        self.pub_white = rospy.Publisher("image_lines_white",Image, queue_size=10)

        # publishes final yellow filtered image
        self.pub_yellow = rospy.Publisher("image_lines_yellow",Image, queue_size=10)


    # this callback function receives 3 msgs with the help of time synchronizer
    def callback(self, cropped_msg, yellow_msg, white_msg):

        # cropped_msg comes in bgr8 format, the rest will be in mono8
        # receive the cropped image
        cropped_cv = self.bridge.imgmsg_to_cv2(cropped_msg, "bgr8")

        # receive the yellow-line image
        # Note: yellow and white lane filtered images come in mono8 format
        yellow_cv = self.bridge.imgmsg_to_cv2(yellow_msg, "mono8")

        # receive the white-line image
        white_cv = self.bridge.imgmsg_to_cv2(white_msg, "mono8")

        # convert the cropped_image to gray 
        grey_img = cv2.cvtColor(cropped_cv,cv2.COLOR_BGR2GRAY)

        # perform canny edge detection
        edges = cv2.Canny(grey_img,0,255)

        # need to dilate incoming white and yellow lane images before doing bitwise and
        # implement the dilate function privided in lecture slide
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(7,7))

        # also create a unique kernel to dilate filtered edges
        kernel_for_edges = cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(1,1))
        
        # dilate white and yellow filtered images
        dilated_white = cv2.dilate(white_cv, kernel)
        dilated_yellow = cv2.dilate(yellow_cv, kernel)

        # dilate the edges to account for lines that are too thin
        dilated_edges = cv2.dilate(edges, kernel_for_edges)

        # perform bitwise and for the white and yellow lines
        and_white = cv2.bitwise_and(dilated_edges, dilated_white, mask=None)
        and_yellow = cv2.bitwise_and(dilated_edges, dilated_yellow, mask=None)

        # apply Probabilistic Hough Transform on both white and yellow images
        # using     -->   HoughLinesP(input  ,rho, np.pi/180, thres, None, minLen, maxGap)

        # white lines work well with param (..,..,.., 5 , None, 10, 5)
        lines_white = cv2.HoughLinesP(and_white, 1, np.pi/180, 5, None, 10, 5)

        # yellow lines work well with param (..,..,.., 2 , None, 5, 5)
        lines_yellow = cv2.HoughLinesP(and_yellow, 1, np.pi/180, 5, None, 2, 2)

        # draw blue line on the two images using function provided in homework9 pdf
        image_lines_white = self.output_lines(cropped_cv, lines_white)
        image_lines_yellow = self.output_lines(cropped_cv, lines_yellow)

        # convert images back to imgmsg to be published
        ros_white = self.bridge.cv2_to_imgmsg(image_lines_white, "bgr8")
        ros_yellow = self.bridge.cv2_to_imgmsg(image_lines_yellow, "bgr8")

        # publish to 2 topics, white and yellow
        self.pub_white.publish(ros_white)
        self.pub_yellow.publish(ros_yellow)

    # function to draw blue lines (provided in homework9 instruction)
    def output_lines(self, original_image, lines):
        output = np.copy(original_image)
        if lines is not None:
            for i in range(len(lines)):
                l = lines[i][0]
                cv2.line(output, (l[0],l[1]), (l[2],l[3]), (255,0,0), 2, cv2.LINE_AA)
                cv2.circle(output, (l[0],l[1]), 2, (0,255,0))
                cv2.circle(output, (l[2],l[3]), 2, (0,0,255))
        return output 

if __name__=="__main__":
    # initialize our node and create a publisher as normal
    rospy.init_node("homework9_node", anonymous=True)
    hw9 = Homework9()
    rospy.spin()
