#!/usr/bin/env python3

import sys
import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import message_filters
import numpy as np
from duckietown_msgs.msg import SegmentList
from sensor_msgs.msg import CompressedImage

#Note: this node utilizes message_filers to synchronize multiple topics to be used in the same callback function

#combines hw8 and hw9 class into a class name Lab5

class Lab5:
    def __init__(self):
        # Instatiate the converter class once by using a class member
        self.bridge = CvBridge()
        #subscribe to the image from duckiebot's camera
        rospy.Subscriber("camera_node/image/compressed", CompressedImage, self.callback_lab5, queue_size=1, buff_size=2**24)

        # publishes to line_detector_node/segment_list
        # the type of message sent in SegmentList
        self.pub_seglist = rospy.Publisher("line_detector_node/segment_list", SegmentList, queue_size=10)

        self.pub_white = rospy.Publisher("image_lines_white",Image, queue_size=10)

    
    def callback_lab5(self, msg):
        # convert to a ROS image using the bridge
        cv_img = self.bridge.compressed_imgmsg_to_cv2(msg, "bgr8")

        image_size = (160, 120)
        offset = 40
        new_image = cv2.resize(cv_img, image_size, interpolation=cv2.INTER_NEAREST)
        cv_cropped = new_image[offset:, :]
        
        # crop the image to 50%(vertically). Also note that decimal shape[0] is converted to integer
        # cv_cropped = cv_img[int(cv_img.shape[0]/2):cv_img.shape[0], 0:cv_img.shape[1]]

        # convert colorspace from BGR to HSV
        hsv_img = cv2.cvtColor(cv_cropped, cv2.COLOR_BGR2HSV)

        #filter the color from the HSV image
        
        # convert new image to ROS to send
        # ros_cropped = self.bridge.cv2_to_imgmsg(cv_cropped, "bgr8")

        
        # publish the cropped image
        # self.pub_crop.publish(ros_cropped)

        # filter white lane   (0,0,225),(180,40,255)
        white_filter = cv2.inRange(hsv_img, (0,0,225), (180,40,255))

        # ros_white = self.bridge.cv2_to_imgmsg(white_filter, "mono8")

        # publish white filtered image
        # self.pub_white.publish(ros_white) 

        # filter yellow lane
        yellow_filter = cv2.inRange(hsv_img, (0,150,180),(70,255,255))

        # ros_yellow = self.bridge.cv2_to_imgmsg(yellow_filter, "mono8") 
 
        # publish yellow filtered image
        # self.pub_yellow.publish(ros_yellow)     

        # cropped_msg comes in bgr8 format, the rest will be in mono8
        # receive the cropped image
        # cropped_cv = self.bridge.imgmsg_to_cv2(cropped_msg, "bgr8")

        # receive the yellow-line image
        # Note: yellow and white lane filtered images come in mono8 format
        # yellow_cv = self.bridge.imgmsg_to_cv2(yellow_msg, "mono8")

        # receive the white-line image
        # white_cv = self.bridge.imgmsg_to_cv2(white_msg, "mono8")

        # convert the cropped image to gray 
        grey_img = cv2.cvtColor(cv_cropped,cv2.COLOR_BGR2GRAY)

        # perform canny edge detection
        edges = cv2.Canny(grey_img,0,255)

        # need to dilate incoming white and yellow lane images before doing bitwise and
        # implement the dilate function privided in lecture slide
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(7,7))

        # also create a unique kernel to dilate filtered edges
        kernel_for_edges = cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(1,1))
        
        # dilate white and yellow filtered images
        dilated_white = cv2.dilate(white_filter, kernel)
        dilated_yellow = cv2.dilate(yellow_filter, kernel)

        # dilate the edges to account for lines that are too thin
        dilated_edges = cv2.dilate(edges, kernel_for_edges)

        # perform bitwise and for the white and yellow lines
        and_white = cv2.bitwise_and(dilated_edges, dilated_white, mask=None)
        and_yellow = cv2.bitwise_and(dilated_edges, dilated_yellow, mask=None)

        # apply Probabilistic Hough Transform on both white and yellow images' lines
        # white lines
        lines_white = cv2.HoughLinesP(and_white, 1, np.pi/180, 5, None, 10, 5)

        # yellow lines
        lines_yellow = cv2.HoughLinesP(and_yellow, 1, np.pi/180, 5, None, 2, 2)

        # draw blue line on the two images using function provided in homework9 pdf
        #image_lines_white = self.output_lines(cv_cropped, lines_white)
        #image_lines_yellow = self.output_lines(cv_cropped, lines_yellow)

        # convert images back to imgmsg to be published

        # ros_yellow = self.bridge.cv2_to_imgmsg(image_lines_yellow, "bgr8")

        arr_cutoff = np.array([0, offset, 0, offset])
        arr_ratio = np.array([1. / image_size[1], 1. / image_size[0], 1. / image_size[1], 1. / image_size[0]])

        rospy.logwarn(lines_white.shape)
        rospy.logwarn(arr_cutoff.shape)
        rospy.logwarn(arr_ratio.shape)

        #for i in range(len(lines_white)):
        line_normalized_white = (lines_white + arr_cutoff) * arr_ratio
        line_normalized_yellow = (lines_yellow + arr_cutoff) * arr_ratio

        white_out = self.output_lines(cv_cropped, line_normalized_white)
        yellow_out = self.output_lines(white_out, line_normalized_yellow)

        ros_output = self.bridge.cv2_to_imgmsg(yellow_out, "bgr8")

        self.pub_seglist.publish(ros_output)

        rospy.logwarn("calculation succeeded \n")

        #create a SegmentList message to be published
        #SegmentList my_seg_msg;


        # publish the message to the topic that ground_projection_node receives
        #self.pub_seglist(my_seg_msg)

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
    rospy.init_node("lab5_node", anonymous=True)
    lab5 = Lab5()
    rospy.spin()
