#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32, String
from duckietown_msgs.msg import Twist2DStamped 

class Run_Circle:
    def __init__(self):

        # publishes to the output of car_cmd_switch_node 
        self.pub = rospy.Publisher("car_cmd_switch_node/cmd", Twist2DStamped, queue_size=10)
        self.my_msg = Twist2DStamped() # initialized the message
    
    # sends the velocity and angular message to the 
    def send_motor_msg(self, vel, angVel):
        self.my_msg.v = vel
        self.my_msg.omega = angVel
        self.pub.publish(self.my_msg)

if __name__ == '__main__':
    try:
        run_c = Run_Circle()
        rospy.init_node('run_circle_node', anonymous=True)
        time_allowed = 10 # edit total running time here
        for i in range(time_allowed):
            run_c.send_motor_msg(0.34, 2.409) # edit velocity and omega values here
            rospy.sleep(1.09)
        run_c.send_motor_msg(0, 0) # stops all motors
        rospy.sleep(1)
    except rospy.ROSInterruptException:
            pass

