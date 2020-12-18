#!/usr/bin/env python3

import sys
import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import message_filters
import numpy as np
from duckietown_msgs.msg import SegmentList
from duckietown_msgs.msg import Segment
from sensor_msgs.msg import CompressedImage
import itertools

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
        self.pub_yellow = rospy.Publisher("image_lines_yellow",Image, queue_size=10)
        self.seg_msg = SegmentList()

    
    def callback_lab5(self, msg):
        # convert to a ROS image using the bridge
        cv_img = self.bridge.compressed_imgmsg_to_cv2(msg, "bgr8")

        image_size = (160, 120)
        offset = 40
        new_image = cv2.resize(cv_img, image_size, interpolation=cv2.INTER_NEAREST)
        cv_cropped = new_image[offset:, :]

        # convert colorspace from BGR to HSV
        hsv_img = cv2.cvtColor(cv_cropped, cv2.COLOR_BGR2HSV)

        # filter white lane   (0,0,225),(180,40,255)
        white_filter = cv2.inRange(hsv_img, (0,0,225), (180,40,255))

        # filter yellow lane (0,150,180) (70,255,255)
        yellow_filter = cv2.inRange(hsv_img, (25,80,120), (75,255,255))

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
        image_lines_white = self.output_lines(cv_cropped, lines_white)
        image_lines_yellow = self.output_lines(cv_cropped, lines_yellow)

        # convert images back to imgmsg to be published
        ros_white = self.bridge.cv2_to_imgmsg(image_lines_white, "bgr8")
        ros_yellow = self.bridge.cv2_to_imgmsg(image_lines_yellow, "bgr8")

        # publish to 2 topics representing white and yellow filtered images
        self.pub_white.publish(ros_white)
        self.pub_yellow.publish(ros_yellow)




        # values for normalization provided in lab5 pdf
        arr_cutoff = np.array([0, offset, 0, offset])
        arr_ratio = np.array([1. / image_size[1], 1. / image_size[0], 1. / image_size[1], 1. / image_size[0]])

        resultList = SegmentList()

        #check to make sure white line is detected
        if (lines_white is not None):
          line_normalized_white = (lines_white + arr_cutoff) * arr_ratio
          w_list = []

          #iterate through each white line
          for i in range(len(line_normalized_white)):

            #create a single segment to store values
            w_seg = Segment()
            w_seg.color = Segment.WHITE

            #store values x0, y0, x1, y1 to this segment
            w_seg.pixels_normalized[0].x = line_normalized_white[i][0][0]
            w_seg.pixels_normalized[0].y = line_normalized_white[i][0][1]
            w_seg.pixels_normalized[1].x = line_normalized_white[i][0][2]
            w_seg.pixels_normalized[1].y = line_normalized_white[i][0][3]

            w_list.append(w_seg)

          # add the resulting list to the resulting List
          resultList.segments.extend(w_list)

        #check to make sure yellow line is detected
        if(lines_yellow is not None):
          line_normalized_yellow = (lines_yellow + arr_cutoff) * arr_ratio
          y_list = []
          #iterate through each yellow line
          for i in range(len(line_normalized_yellow)):

            #create a single segment to store values
            y_seg = Segment()
            y_seg.color = Segment.YELLOW

            #store values x0, y0, x1, y1 to this segment
            y_seg.pixels_normalized[0].x = line_normalized_yellow[i][0][0]
            y_seg.pixels_normalized[0].y = line_normalized_yellow[i][0][1]
            y_seg.pixels_normalized[1].x = line_normalized_yellow[i][0][2]
            y_seg.pixels_normalized[1].y = line_normalized_yellow[i][0][3]

            # add this segment to the list of yellow segments
            y_list.append(w_seg)

          # finally, add the list to the resulting List
          resultList.segments.extend(y_list)

          #self.seg_msg.pixels_normalized[0].x = line_normalized_white

          #ros_output = self.bridge.cv2_to_imgmsg(yellow_out, "bgr8")

          # publish the resulting list of segments to a topic received by ground_detection_node
          self.pub_seglist.publish(resultList)

          #rospy.logwarn("calculation succeeded \n")

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
