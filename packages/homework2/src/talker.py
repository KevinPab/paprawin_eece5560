#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float32

class Talker:
    num = 0
    def __init__(self):
        self.pub = rospy.Publisher("/homework1/delta", Float32, queue_size=10)

    def talk(self):
        self.num += 1
        rospy.loginfo(self.num)
        self.pub.publish(self.num)

if __name__ == '__main__':
    try:
        t = Talker()
        rospy.init_node('talker', anonymous=True)
        rate = rospy.Rate(1)
        while not rospy.is_shutdown():
            t.talk()
            rate.sleep()
    except rospy.ROSInterruptException:
            pass

