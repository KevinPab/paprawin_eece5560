#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32, String
from pid import PID

class Homework5:
    def __init__(self):
        # receives messages from error topic published by vehicle_dynamics.py
        rospy.Subscriber("error", Float32, self.callback)

        # homework 4 publishes with a "hw4_msg" type message
        self.pub = rospy.Publisher("/homework4/converted_total", hw4_msg, queue_size=10)
        self.pid_control = PID()
        
    
    def callback(self, data):

        pid_cont = PID(p=0, i=0, d=0) # initialize PID object
        pid_cont.calc_control(data)

        # check if units exist
        if rospy.has_param("units"):
            # get unit to be converted into: meter, smoots, or feet
            # save into the message type
            my_msg.my_string = rospy.get_param("units")

            # smoots conversion
            if my_msg.my_string == "smoots":
                my_msg.my_float = data.data * 0.1791
                rospy.loginfo("smoots: %f", my_msg.my_float)
                self.pub.publish(my_msg) # this publishes both the float and string in my_msg

            # feet conversion
            elif my_msg.my_string == "feet":
                my_msg.my_float = data.data # no change in data
                rospy.loginfo("feet: %f", my_msg.my_float)
                self.pub.publish(my_msg)

            # default converts to meters
            else :
                my_msg.my_float = data.data * 0.3048
                rospy.loginfo("meters: %f", my_msg.my_float)
                self.pub.publish(my_msg)
        else:
            rospy.logwarn("Param not found")

if __name__ == '__main__':
    rospy.init_node('homework5_node')
    rospy.set_param("controller_ready", "true") # set controller to ready
    Homework5()

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

