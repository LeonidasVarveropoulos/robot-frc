#!/usr/bin/python

import rospy
import tf
from std_msgs.msg import Float32, String, Bool
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
import math

class SwerveJoy:
    def __init__(self):
        rospy.init_node('swerve_joy')

        rospy.Subscriber("joy/linear_x", Float32, self.on_joy_x)
        self.linear_x = 0

        rospy.Subscriber("joy/linear_y", Float32, self.on_joy_y)
        self.linear_y = 0

        rospy.Subscriber("joy/angular_z", Float32, self.on_joy_z)
        self.angular_z = 0

        self.linear_scale = rospy.get_param("linear_scale", 4.0)
        self.angular_scale = rospy.get_param("angular_scale", 3.0)

        self.cmd_vel_pub = rospy.Publisher("/cmd_vel", Twist , queue_size=10)

    def on_joy_x(self, msg):
        self.linear_x = msg.data
    def on_joy_y(self, msg):
        self.linear_y = msg.data
    def on_joy_z(self, msg):
        self.angular_z = msg.data

    def main(self):
        rate = rospy.Rate(15)

        while not rospy.is_shutdown():
            cmd_vel = Twist()
            cmd_vel.linear.x = self.linear_x * self.linear_scale
            cmd_vel.linear.y = -self.linear_y * self.linear_scale
            cmd_vel.angular.z = self.angular_z * self.angular_scale

            self.cmd_vel_pub.publish(cmd_vel)

            rate.sleep()

if __name__ == '__main__':
    joy = SwerveJoy()
    joy.main()
