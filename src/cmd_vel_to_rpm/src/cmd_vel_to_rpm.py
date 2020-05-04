#! /usr/bin/env python

import rospy
from std_msgs.msg import Float32, Float64, Bool
from geometry_msgs.msg import Twist
import tf
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
from nav_msgs.msg import Odometry
import time
import math

# Creates node
rospy.init_node('cmd_vel_to_rpm')

class CmdVelToRpm:
    def __init__(self):
        rospy.loginfo("Started cmd_vel_to_rpm node")

        rospy.Subscriber("/cmd_vel", Twist, self.cmd_vel_callback)
        self.cmd_vel_data = Twist()

        # Publishing data
        self.left_rpm_pub = rospy.Publisher("cmd_vel/left", Float64, queue_size=50)
        self.right_rpm_pub = rospy.Publisher("cmd_vel/right", Float64, queue_size=50)

        self.body_width = rospy.get_param("body_width", 0.5969)

        self.wheel_circum = rospy.get_param("wheel_diameter", 0.1524) * math.pi
    
    def main(self):
        # Needed for ros node
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            rate.sleep()

    def cmd_vel_callback(self, msg):
        self.cmd_vel_data = msg

        vx = self.cmd_vel_data.linear.x
        vth = self.cmd_vel_data.angular.z

        left_distance = ((vx*2)-(vth*self.body_width))/2.0
        right_distance = (vth * self.body_width) + left_distance

        right_rpm = (right_distance/self.wheel_circum) * 60.0
        left_rpm = (left_distance/self.wheel_circum) * 60.0

        right_data = Float64()
        left_data = Float64()

        right_data.data = right_rpm
        left_data.data = left_rpm

        self.right_rpm_pub.publish(right_data)
        self.left_rpm_pub.publish(left_data)

if __name__ == '__main__':
    rpm = CmdVelToRpm()
    rpm.main()