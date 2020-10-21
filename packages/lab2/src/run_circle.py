#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32, String
from duckietown_msgs.msg import Twist2DStamped 

class Run_Circle:
    def __init__(self):

        # publishes to the output of carm_cmd_switch_node 
        self.pub = rospy.Publisher("car_cmd_switch_node/cmd", Twist2DStamped, queue_size=10)
        my_msg = Twist2DStamped() # initialized the message
    
    # sends the velocity and angular message to the 
    def send_motor_msg(self, vel, angVel):
        self.my_msg.v = vel
        self.my_msg.omega = angVel
        self.pub.publish(my_msg)

if __name__ == '__main__':
    try:
        run_c = Run_Circle()
        rospy.init_node('run_circle_node', anonymous=True)
        rate = rospy.Rate(1)
        while not rospy.is_shutdown():
            run_c.send_motor_msg(0.403, 4.0)
            rate.sleep()
    except rospy.ROSInterruptException:
            pass

