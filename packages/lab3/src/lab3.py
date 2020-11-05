#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32, String
from duckietown_msgs.msg import LanePose, Twist2dStamped, FSMState
from pid import PID

class Homework5:
    def __init__(self):

        # homework 5 publishes to the control input for vehicle_dynamics to read
        self.pub = rospy.Publisher("lane_controller_node/cmd", Twist2DStamped, queue_size=10)
        self.pid_1 = PID(p=0.03, i=0.01, d=0)
        self.pid_2 = PID(p=0.03, i=0.01, d=0)
        self.my_msg = Twist2DStamped()
        self.my_msg.v = 20 # velocity

        # receive the phi and d from lane_pose
        rospy.Subscriber("lane_filter_node/lane_pose", LanePose, self.callback)


    # call PID class' control calculation when receiving error message
    def callback(self, error):

        # use first PID controller on d
        omega_1 = self.pid_1.calc_control(error.data.d, 0.001)

        # use second PID controller on phi
        omega_2 = self.pid_2.calc_control(error.data.phi, 0.001)

        # add up omega values
        self.my_msg.omega = omega1 + omega2

        # sends new controller message to system 
        self.pub.publish(self.my_msg)



if __name__ == '__main__':

    rospy.init_node('homework5_node')
    # sleep allows user to run rqt_plot
    rospy.sleep(5)
    Homework5()

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

