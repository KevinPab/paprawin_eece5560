#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32, String
from odometry_hw.msg import Pose2D, DistWheel
import math

class Homework7:
    def __init__(self):

        # publishes to /pose topic with a Pose2D message type
        self.pub = rospy.Publisher("/pose", Pose2D, queue_size=10)

        # pose initially set to 0 
        self.my2DPose = Pose2D(0,0,0)

        # subscribes to /dist_wheel topic
        rospy.Subscriber("/dist_wheel", DistWheel, self.callback)



    # receive values from rosbag given in this assignment and calculate odometry
    def callback(self, disWheel):

        # Implement equation provided in lecture slides
        d_s = (disWheel.dist_wheel_left + disWheel.dist_wheel_right)/2
        d_theta = (disWheel.dist_wheel_right - disWheel.dist_wheel_left)/0.1
        d_x = d_s*math.cos(self.my2DPose.theta + d_theta/2)
        d_y = d_s*math.sin(self.my2DPose.theta + d_theta/2)

        # accumulate the values x,y,theta
        self.my2DPose.x += d_x
        self.my2DPose.y += d_y
        self.my2DPose.theta += d_theta

        # send new pose to the graph
        self.pub.publish(self.my2DPose)


if __name__ == '__main__':

    rospy.init_node('homework7_node')
    
    Homework7()

    # spin() keeps python from exiting
    rospy.spin()

