#!/usr/bin/env python3

import rospy
from example_service.srv import *


if __name__ == "__main__":
    
    # start the client node
    rospy.init_node('hw4_client_node')
    
    # request service 
    rospy.wait_for_service('calc_fibonacci')
    try:
        fibonacci_serv = rospy.ServiceProxy('calc_fibonacci', Fibonacci)
        rospy.loginfo("Before service requested")
        respons = fibonacci_serv(3) # asks for 3 fibonacci sequences
        print(respons) # print the fibonacci sequence
        rospy.loginfo("After service requested")
    except rospy.ServiceException as e:
        print("Service call failed: %s" %e)

