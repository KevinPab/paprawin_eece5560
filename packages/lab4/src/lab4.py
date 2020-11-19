#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32, String
from duckietown_msgs.msg import WheelsCmdStamped
import math


# Define constants to help fix the errors in pose reported from robot
# account for the left/right wheel difference due to calibrated trim value 

TRIM_FIX = 1.0304568569730501

# convert actual meter run into correct reported meters run: 1m / 1m(measured valued)
DISTANCE_FIX = 1.0/41.1982

# fix angle
ANGLE_FIX = 90.0/96.0


class Lab4:
    def __init__(self):


        # pose initially set to 0 
        self.x = 0
        self.y = 0
        self.theta = 0

        # in this duckiebot no.33, calibration has made right velocity greater than left velocity
        # constant = right_vel / left_vel during straight-forward driving (calculate to 17 decimal places using WolframAlpha.com)

        # subscribes to wheels_driver_node/wheels_cmd
        rospy.Subscriber("wheels_driver_node/wheels_cmd", WheelsCmdStamped, self.callback)



    # receive values from rosbag given in this assignment and calculate odometry
    def callback(self, wheelsCmd):

        # apply constant to fix velocity differences caused by trim
        if (wheelsCmd.vel_left != 0):
          wheelsCmd.vel_left *= TRIM_FIX

        # Implement equation provided in lecture slides
        d_s = (wheelsCmd.vel_left + wheelsCmd.vel_right)/2
        d_theta = (wheelsCmd.vel_right - wheelsCmd.vel_left)/0.1
        # apply constant to fix angle from actual measured to reported value
        d_x = d_s*math.cos(self.theta + ANGLE_FIX * d_theta/2)
        d_y = d_s*math.sin(self.theta + ANGLE_FIX * d_theta/2)

        # accumulate the values x,y,theta and apply constant to correct report the actual meters ran on the mat
        self.x += d_x * DISTANCE_FIX
        self.y += d_y * DISTANCE_FIX
        
        self.theta += d_theta

        rospy.logwarn("x: %f, y: %f, angle: %f", self.x, self.y, self.theta)
        # send new pose to the graph
        #self.pub.publish(self.my2DPose)


if __name__ == '__main__':

    rospy.init_node('lab4_node')
    
    Lab4()

    # spin() keeps python from exiting
    rospy.spin()

