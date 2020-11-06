#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32, String
from duckietown_msgs.msg import LanePose, Twist2DStamped, FSMState
from pid import PID

class Lab3:
    def __init__(self):

        # publish to cmd
        self.pub = rospy.Publisher("car_cmd_switch_node/cmd", Twist2DStamped, queue_size=10)

        # tune pid values below
        self.pid_1 = PID(p= -6.5, i= 0, d=0)
        self.pid_2 = PID(p= -5, i= 0 , d=0)

        self.my_msg = Twist2DStamped()
        self.allow_follow = False
        self.velocity = 0.2  # adjust velocity here

        # receive the phi and d to update PID controller and movement
        rospy.Subscriber("lane_filter_node/lane_pose", LanePose, self.follow_lane)

        # receive command from joystick
        rospy.Subscriber("fsm_node/mode", FSMState, self.callbackFSM)


    # use the pose (d and phi) to calculate omega 
    def follow_lane(self, pose):

        # if joystick key "a" is pressed, run lane follow
        if (self.allow_follow):
            rospy.logwarn("PAPRAWIN LANE FOLLOWING Code")
            rospy.logwarn("d received = %f", pose.d)

            self.my_msg.v = self.velocity

            # use first PID controller on d
            omega_1 = self.pid_1.calc_control(pose.d, 0.0015)

            # use second PID controller on phi
            omega_2 = self.pid_2.calc_control(pose.phi, 0.0015)

            # add up omega values
            self.my_msg.omega = omega_1 + omega_2

            # sends new controller message to system 
            self.pub.publish(self.my_msg)

        # else it will stop running
        else:
            self.my_msg.v = 0
            self.my_msg.omega = 0
            self.pub.publish(self.my_msg)

    def callbackFSM(self, keypressed):
        if (keypressed.state == "LANE_FOLLOWING"):
            self.allow_follow = True # program will run once "a" is pressed

        elif (keypressed.state == "NORMAL_JOYSTICK_CONTROL"):
            self.allow_follow = False # disable lane following program when "s" is pressed



if __name__ == '__main__':

    rospy.init_node('lab3_node')
    Lab3()

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

