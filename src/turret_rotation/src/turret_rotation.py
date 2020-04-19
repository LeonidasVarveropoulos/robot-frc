#!/usr/bin/env python

import math
from math import sin, cos, pi
import rospy
import tf
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32, Bool
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3

class TurretRotation:
    def __init__(self):
        rospy.init_node('turret_rotation')
        self.cmd_vel_pub = rospy.Publisher("turret/cmd_vel", Float32, queue_size=1)
        self.rot_ready_pub = rospy.Publisher("turret/rot_ready", Bool, queue_size=1)

        self.update_rate = 15

        self.pid = PID(self.update_rate, 15.0, -15.0)
        # This is the feedback and setpoint from the vision processing node (pixels)
        self.feedback = 0.0
        self.setpoint = 0.0

        # Thsi is the control output of the pid that will be sent to the roboRio (velocity)
        self.control = 0.0

        self.kp = 0.05
        self.ki = 0.0
        self.kd = 0.0
        self.kf = 0.0

    def feed_callback(self, msg):
        # Feedback from vision targeting
        self.feedback = msg.data
        print("Got Data F")

    def set_callback(self, msg):
        # Setpoint from vision targeting
        self.setpoint = msg.data
        print("Got Data S")

    def main(self):

        feed_sub = rospy.Subscriber('turret/feedback', Float32, self.feed_callback)
        set_sub = rospy.Subscriber('turret/setpoint', Float32, self.set_callback)

        r = rospy.Rate(self.update_rate)

        while not rospy.is_shutdown():

            self.control = self.pid.update_pid(self.setpoint, self.feedback, self.kp, self.ki, self.kd, self.kf)

            self.rot_ready = self.pid.is_rotation_ready(self.setpoint, self.feedback)
            # Publishes the rpm to ros for the proxy to send to the roboRio
            data = Float32()
            data.data = self.control
            self.cmd_vel_pub.publish(data)

            rot_data = Bool()
            rot_data.data = self.rot_ready
            self.rot_ready_pub.publish(rot_data)
            
            r.sleep()

class PID:
    def __init__(self, update_rate, control_max, control_min):
        self.control = 0.0
        self.error_sum = 0.0
        self.update_rate = update_rate
        self.control_max = control_max 
        self.control_min = control_min
        self.prev_error = 0.0
    
    def reset(self):
        self.control = 0.0
        self.error_sum = 0.0
        self.prev_error = 0.0

    def update_pid(self, setpoint, feedback, kp, ki, kd, kf):
        # Calculate error
        error = (setpoint) - feedback
        
        # Calculate sum of error over time
        self.error_sum += (error/self.update_rate) # Not sure if this should be here

        if ki != 0:
            error_sum_max = self.control_max / ki
            error_sum_min = self.control_min / ki
            if self.error_sum > error_sum_max:
                self.error_sum = error_sum_max
            elif self.error_sum < error_sum_min:
                self.error_sum = error_sum_min
        
        self.control = (setpoint * kf) + (kp * error) + (ki *self.error_sum) + (kd * ((error - self.prev_error) * self.update_rate))

        if self.control > self.control_max:
            self.control = self.control_max
        elif self.control < self.control_min:
            self.control = self.control_min

        self.prev_error = error

        return self.control
    def is_rotation_ready(self, setpoint, feedback):
        dist = abs(setpoint-feedback)
        if (dist > 0 and dist < 10):
            return True
        return False

tr = TurretRotation()
tr.main()
