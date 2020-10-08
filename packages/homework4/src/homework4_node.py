#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32, String
from homework4.msg import hw4_msg

class Homework4:
    def __init__(self):
        # receives messages from homework1/total topic
        rospy.Subscriber("/homework1/total", Float32, self.callback)

        # homework 4 publishes with a "hw4_msg" type message
        self.pub = rospy.Publisher("/homework4/converted_total", hw4_msg, queue_size=10)
        self.smoot = 0
        self.meter = 0
        self.feet = 0
        self.paramStr = ""
    
    # gets called after receiving message from hw1's total
    def callback(self, data):
        my_msg = hw4_msg() # initialized the message
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
    rospy.init_node('homework4_node')
    Homework4()

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

