#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32, String
from duckietown_msgs.msg import WheelsCmdStamped, Pose2DStamped
import math


# Define constants to help fix the errors in pose reported from robot
# this scales the left & right velocity from the trimmed value (DB-18)
TRIM_SCALING = 0.48468437790870667 / 0.4703587293624878

# convert measured distance into desired distance: (1.0 meter) / (actual value reported after running 1 meter)
DISTANCE_FIX = 1.0/39.239

# fix angle: (desired angle) / (measured angle)
ANGLE_FIX = 90.0/52.0


class Lab4:
    def __init__(self):


        # pose initially set to 0 

        self.pose = Pose2DStamped()

        # in this duckiebot no.33, calibration has made right velocity greater than left velocity
        # constant = right_vel / left_vel during straight-forward driving (calculate to 17 decimal places using WolframAlpha.com)

        # subscribes to wheels_driver_node/wheels_cmd
        rospy.Subscriber("wheels_driver_node/wheels_cmd", WheelsCmdStamped, self.callback)
        self.pub = rospy.Publisher("lab4_results", Pose2DStamped, queue_size=10)
        



    # receive values from rosbag given in this assignment and calculate odometry
    def callback(self, wheelsCmd):

        # apply trim scaling to make left/right velocity equal
        if (wheelsCmd.vel_left != 0):
          wheelsCmd.vel_left *= TRIM_SCALING

        # Implement equation provided in lecture slides
        d_s = (wheelsCmd.vel_left + wheelsCmd.vel_right)/2
        # apply constant to fix distance in order to obtained desired value
        d_s *= DISTANCE_FIX
        d_theta = (wheelsCmd.vel_right - wheelsCmd.vel_left)/0.2
        # apply constant to fix angle from actual measured to reported value
        d_x = d_s*math.cos(math.radians(self.pose.theta + (d_theta/2)))
        d_y = d_s*math.sin(math.radians(self.pose.theta + (d_theta/2)))
        
        # logwarn for debugging
        #rospy.logwarn("d_s=%f my_angle=%f inside_bracket=%f theta=%f sin = %f", d_s,self.theta,(self.theta + (d_theta/2)),d_theta,math.sin(math.radians(self.theta+(d_theta/2))))

        # accumulate the values x,y,theta and apply constant to correct report the actual meters ran on the mat
        self.pose.x += d_x
        self.pose.y += d_y
        
        self.pose.theta += d_theta * ANGLE_FIX

        #rospy.logwarn("x: %f, y: %f, angle: %f", self.x, self.y, self.theta)

        self.pub.publish(self.pose)


if __name__ == '__main__':

    rospy.init_node('lab4_node')
    
    Lab4()

    # spin() keeps python from exiting
    rospy.spin()

