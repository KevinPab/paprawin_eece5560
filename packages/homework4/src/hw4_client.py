#!/usr/bin/env python3

import rospy
import actionlib
from example_service.srv import *
import example_action_server.msg



if __name__ == "__main__":
    
    # start the client node
    rospy.init_node('hw4_client')
    
    # request service
    rospy.wait_for_service('/example_services/calc_fibonacci')
    try:
        # Run the Server client with 3 fib request
        serv_client = rospy.ServiceProxy('/example_services/calc_fibonacci', Fibonacci)
        rospy.loginfo("Before service called") # log before service executes
        responses = serv_client(2) # asks for 3 fibonacci sequences
        rospy.loginfo("After service called") # log after service executes
        rospy.loginfo(responses) # log result

        # Run the action-server client with 3 fib request
        action_client = actionlib.SimpleActionClient('/example_action_server/fibonacci', example_action_server.msg.FibonacciAction)
        action_client.wait_for_server()
        goal = example_action_server.msg.FibonacciGoal(order=2) 
        rospy.loginfo("Before action request")
        action_client.send_goal(goal)  # ask for 3 fib sequence
        rospy.loginfo("After action request")
        action_client.wait_for_result()
        rospy.loginfo(action_client.get_result()) # log result
        rospy.loginfo("After result received")
        
        # Second Run for 15 numbers:
        # Run the Server client again but with 15 fib request
        serv_client = rospy.ServiceProxy('/example_services/calc_fibonacci', Fibonacci)
        rospy.loginfo("2nd run - Before service called") # log before service executes
        responses = serv_client(14) # asks for 15 fibonacci sequences
        rospy.loginfo("2nd run - After service called") # log after service executes
        rospy.loginfo(responses) # log result

        # Run the action-server client again with 15 fib request
        action_client = actionlib.SimpleActionClient('/example_action_server/fibonacci', example_action_server.msg.FibonacciAction)
        action_client.wait_for_server()
        goal = example_action_server.msg.FibonacciGoal(order=14) 
        rospy.loginfo("2nd run - Before action request")
        action_client.send_goal(goal)  # ask for 15 fib sequence
        rospy.loginfo("2nd run - After action request")
        action_client.wait_for_result()
        rospy.loginfo(action_client.get_result()) # log result
        rospy.loginfo("2nd run - After result received...program completed!")


    except rospy.ServiceException as e:
        print("Service call failed: %s" %e)
    
