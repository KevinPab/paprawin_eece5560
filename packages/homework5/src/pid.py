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

    # Receives (error value, delta time). Returns updated control signal value
    def calc_control(self, error, dt):

        # Todo: update p,i,d using error val and return the result
        p = error
        i = self.ki + (error * dt)
        d = (error - self.kd) / dt
        result = (self.kp*p) + (self.ki * i) + (self.kd * d)
        return result
        

