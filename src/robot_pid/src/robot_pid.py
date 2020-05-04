#!/usr/bin/env python

import math
from math import sin, cos, pi
import rospy
import tf
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32, Bool
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3

class RobotPID:
    def __init__(self):
        rospy.init_node('robot_pid')

        self.pid_list = rospy.get_param("~pid_list", [])

        self.update_rate = rospy.get_param("~rate", 15)

        # This is the feedback and setpoint from the subscribed topics
        self.feedback_data = {}
        self.setpoint_data = {}

        self._publishers = {}
        self._subscribers = []

    def feed_callback(self, msg):
        # Feedback from topic
        self.feedback_data[str(msg._connection_header["topic"])] = msg

    def set_callback(self, msg):
        # Setpoint from topic
        self.setpoint_data[str(msg._connection_header["topic"])] = msg

    def subscribe(self, topic_name, callback_funct):
        """ This sets up the ros subscribers for incoming data """
        # Checks if subscriber exists, if not create one
        if topic_name in self._subscribers:
            return
        rospy.Subscriber(topic_name, Float32, callback_funct)
        self._subscribers.append(topic_name)

    def publish(self, topic_name, data_type, data, queue = 10, latching = False):
        """ This publishes ros data """
        # Check if publisher exists, if not create and publish data
        if not topic_name in self._publishers:
            self._publishers[topic_name] = rospy.Publisher(topic_name, data_type, queue_size=queue, latch=latching)
        self._publishers[topic_name].publish(data)

    def get_setpoint(self, topic_name):
        """ This gets the subscribed ros data """
        # Get the data from the dict if it exists
        if topic_name in self.setpoint_data:
            return self.setpoint_data[topic_name].data
        return None
    
    def get_feedback(self, topic_name):
        """ This gets the subscribed ros data """
        # Get the data from the dict if it exists
        if topic_name in self.feedback_data:
            return self.feedback_data[topic_name].data
        return None

    def main(self):

        for pid in self.pid_list:
            pid["PID"] = PID(self.update_rate, pid["control_max"], pid["control_min"])
            self.subscribe(pid["feedback_topic"], self.get_feedback)
            self.subscribe(pid["setpoint_topic"], self.get_setpoint)

        r = rospy.Rate(self.update_rate)

        while not rospy.is_shutdown():
            for pid in self.pid_list:
                if self.get_setpoint(pid["setpoint_topic"]) != None and self.get_feedback(pid["feedback_topic"]) != None:
                    setpoint = self.get_setpoint(pid["setpoint_topic"])
                    feedback = self.get_feedback(pid["feedback_topic"])

                    control = pid["PID"].update_pid(setpoint, feedback, pid["kp"], pid["ki"], pid["kd"], pid["kf"])

                    # Publishes the rpm to ros for the proxy to send to the roboRio
                    data = Float32()
                    data.data = control
                    self.publish(pid["control_topic"], Float32, data)
                else:
                    rospy.logerr("Could not find the specified topics to subscribe to")

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

if __name__ == '__main__':
    robot = RobotPID()
    robot.main()
