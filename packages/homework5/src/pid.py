#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32, String

class PID:
    def __init__(self, p, i, d):
        self.kp = p
        self.ki = i
        self.kd = d
    
    # change gains
    def change_gains(self, p, i, d):
        self.kp = p
        self.ki = i
        self.kd = d

    # calculate control signal. Returns control signal message
    def calc_control(self, error, time):
        # Todo: update p,i,d using error val
        # add p + i + d and return the result
        self.kp = self.kp * error
        self.ki = self.ki + (error * time) 
        self.kd = 
        result = self.kp + self.ki + self.kd
        return result
        

