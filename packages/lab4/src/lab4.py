#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32, String
from odometry_hw.msg import Pose2D, DistWheel
from duckietown_msgs.msg import WheelsCmdStamped
import math

class Lab4:
    def __init__(self):


        # pose initially set to 0 
        self.x = 0
        self.y = 0
        self.theta = 0

        # in this duckiebot no.33, calibration has made right velocity greater than left velocity
        # constant = right_vel / left_vel during straight-forward driving (calculate to 17 decimal places using WolframAlpha.com)
        self.vel_fix_constant = 1.0304568569730501

        # subscribes to wheels_driver_node/wheels_cmd
        rospy.Subscriber("wheels_driver_node/wheels_cmd", WheelsCmdStamped, self.callback)



    # receive values from rosbag given in this assignment and calculate odometry
    def callback(self, wheelsCmd):

        # account for the different left/right wheel velocity due to kinematic calibration by applying pre-calculated constant
        if (wheelsCmd.vel_left != 0):
          wheelsCmd.vel_left *= self.vel_fix_constant

        # Implement equation provided in lecture slides
        d_s = (wheelsCmd.vel_left + wheelsCmd.vel_right)/2
        d_theta = (wheelsCmd.vel_right - wheelsCmd.vel_left)/0.1
        d_x = d_s*math.cos(self.theta + d_theta/2)
        d_y = d_s*math.sin(self.theta + d_theta/2)

        # accumulate the values x,y,theta
        self.x += d_x
        self.y += d_y
        self.theta += d_theta

        rospy.logwarn("%f, %f, %f", self.x, self.y, self.theta)
        # send new pose to the graph
        #self.pub.publish(self.my2DPose)


if __name__ == '__main__':

    rospy.init_node('lab4_node')
    
    Lab4()

    # spin() keeps python from exiting
    rospy.spin()

