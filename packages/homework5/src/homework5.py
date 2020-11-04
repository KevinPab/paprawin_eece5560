#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32, String
from pid import PID

class Homework5:
    def __init__(self):

        # homework 5 publishes to the control input for vehicle_dynamics to read
        self.pub = rospy.Publisher("/control_input", Float32, queue_size=10)
        self.pid_obj = PID(p=0.03, i=0.01, d=0.317)

        # set param controller_ready to true
        rospy.set_param("controller_ready", "true") 

        # receives/responds-to messages from error topic published by vehicle_dynamics.py
        rospy.Subscriber("/error", Float32, self.callback)



    # call PID class' control calculation when receiving error message
    def callback(self, error):

        # calculates and updates the pid values using the received error
        result = self.pid_obj.calc_control(error.data, 0.001)

        # sends new controller message to system 
        self.pub.publish(result)



if __name__ == '__main__':

    rospy.init_node('homework5_node')
    # sleep allows user to run rqt_plot
    rospy.sleep(5)
    Homework5()

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

