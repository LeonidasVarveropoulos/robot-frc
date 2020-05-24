#!/usr/bin/python

import rospy
import tf
from std_msgs.msg import Float32, String, Bool
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
import math

class UnityJoy:
    def __init__(self):
        rospy.init_node('unity_joy')
        rospy.Subscriber("joy", Joy, self.on_joy_input)

        self.linear_scale = rospy.get_param("linear_scale", 4.0)
        self.angular_scale = rospy.get_param("angular_scale", 3.0)

        self.cmd_vel_pub = rospy.Publisher("/cmd_vel", Twist , queue_size=10)

        self.shooter_state_pub = rospy.Publisher("/auto/shooter/state", String, queue_size=10)
        self.shooter_state = String()

        self.intake_state_pub = rospy.Publisher("/auto/intake/state", String, queue_size=10)
        self.intake_state = String()

    def on_joy_input(self, msg):
        cmd_vel = Twist()
        cmd_vel.linear.x = msg.axes[1] * self.linear_scale
        cmd_vel.angular.z = msg.axes[3] * self.angular_scale

        self.cmd_vel_pub.publish(cmd_vel)

        if (msg.buttons[2] == 1):
            if (self.intake_state.data == "deploy"):
                self.intake_state.data = "retract"
            else:
                self.intake_state.data = "deploy"      
        self.intake_state_pub.publish(self.intake_state)
        
        if (msg.axes[5] != 1 and str(msg.axes[5]) != "0.0"):
            self.shooter_state.data = "shoot"
        elif (self.shooter_state.data == "shoot"):
            self.shooter_state.data = "idle"
        elif (msg.buttons[0] == 1):
            if (self.shooter_state.data == "idle"):
                self.shooter_state.data = "prime"
            else:
                self.shooter_state.data = "idle"

        self.shooter_state_pub.publish(self.shooter_state)

    def main(self):
        rate = rospy.Rate(1)

        while not rospy.is_shutdown():              
            rate.sleep()

if __name__ == '__main__':
    joy = UnityJoy()
    joy.main()
