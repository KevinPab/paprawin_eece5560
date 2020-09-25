#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32, String

class Homework3:
    def __init__(self):
        rospy.Subscriber("/homework1/total", Float32, self.callback)
        self.pub = rospy.Publisher("/homework3/converted_total", Float32, queue_size=10)
        self.smoot = 0
        self.meter = 0
        self.feet = 0
        self.paramStr = ""

    def callback(self, data):
        # check if units exist
        if rospy.has_param("units"):
            # get unit to be converted into: meter, smoots, or feet
            self.paramStr = rospy.get_param("units")
            # smoots conversion
            if self.paramStr == "smoots":
                self.smoot = data.data * 0.1791
                rospy.loginfo("heyyyy: %f",self.smoot )
                self.pub.publish(self.smoot)

            # feet conversion
            elif self.paramStr == "feet":
                self.feet = data.data # no change in data
                rospy.loginfo("feet: %f", self.feet)
                self.pub.publish(self.feet)
            # default converts to meters
            else :
                self.meter = data.data * 0.3048
                rospy.loginfo("meters: %f", self.meter)
                self.pub.publish(self.meter)
        else:
            rospy.logwarn("Param not found")

if __name__ == '__main__':
    rospy.init_node('homework3')
    Homework3()

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

