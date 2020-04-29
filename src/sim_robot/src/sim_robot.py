#!/usr/bin/python

import rospy
import tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import Float32, Float64, Bool
from geometry_msgs.msg import Twist
import math

class SimRobot:
    def __init__(self):
        self.x = 0
        self.y = 0
        self.th = 0

        self.actual_th = 0

        self.vx = 0
        self.vy = 0
        self.vth = 0

        self.rate = 20

    def cmd_vel_callback(self,msg):
        self.vx = msg.linear.x 
        self.vth = msg.angular.z


    def actual_odom_callback(self, msg):
        quat = (msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w)
        euler = tf.transformations.euler_from_quaternion(quat)
        self.actual_th = euler[2]

    def main(self):
        print("starting node")
        rospy.init_node('sim_robot')
        
        rospy.Subscriber("cmd_vel", Twist, self.cmd_vel_callback)
        rospy.Subscriber("robot_pose_ekf/odom_combined", PoseWithCovarianceStamped, self.actual_odom_callback)

        auto_state_pub = rospy.Publisher("/auto/state", Bool , queue_size=10)
        auto_state_data = Bool()
        auto_state_data.data = True

        odom_pub = rospy.Publisher("/sim_odom", Odometry , queue_size=10)
        odom_msg = Odometry()
        
        odom_msg.header.frame_id = "odom"
        odom_msg.child_frame_id = "base_link"
        
        odom_msg.pose.covariance = [0.01, 0.0, 0.0, 0.0, 0.0, 0.0,
                                    0.0, 0.01, 0.0, 0.0, 0.0, 0.0,
                                    0.0, 0.0, 0.01, 0.0, 0.0, 0.0,
                                    0.0, 0.0, 0.0, 0.0001, 0.0, 0.0,
                                    0.0, 0.0, 0.0, 0.0, 0.0001, 0.0,
                                    0.0, 0.0, 0.0, 0.0, 0.0, 0.0001]
                                            
        odom_msg.twist.covariance = [0.01, 0.0, 0.0, 0.0, 0.0, 0.0,
                                    0.0, 0.01, 0.0, 0.0, 0.0, 0.0,
                                    0.0, 0.0, 0.01, 0.0, 0.0, 0.0,
                                    0.0, 0.0, 0.0, 0.0001, 0.0, 0.0,
                                    0.0, 0.0, 0.0, 0.0, 0.0001, 0.0,
                                    0.0, 0.0, 0.0, 0.0, 0.0, 0.0001]

        rate = rospy.Rate(self.rate)

        while not rospy.is_shutdown():
            
            delta_x = (self.vx * math.cos(self.actual_th) - self.vy * math.sin(self.actual_th)) /self.rate
            delta_y = (self.vx * math.sin(self.actual_th) + self.vy * math.cos(self.actual_th)) /self.rate
            delta_th = self.vth/self.rate

            self.x += delta_x
            self.y += delta_y
            self.th += delta_th

            quaternion = tf.transformations.quaternion_from_euler(0, 0, self.th, 'sxyz')
                    
            odom_msg.pose.pose.position.x = self.x
            odom_msg.pose.pose.position.y = self.y
            odom_msg.pose.pose.position.z = 0

            odom_msg.pose.pose.orientation.x = quaternion[0]
            odom_msg.pose.pose.orientation.y = quaternion[1]
            odom_msg.pose.pose.orientation.z = quaternion[2]
            odom_msg.pose.pose.orientation.w = quaternion[3]
                            
            odom_msg.twist.twist.linear.x = self.vx
            odom_msg.twist.twist.angular.x = 0
            odom_msg.twist.twist.angular.y = 0
            odom_msg.twist.twist.angular.z = self.vth
                
            now  = rospy.get_rostime()
            odom_msg.header.stamp.secs = now.secs
            odom_msg.header.stamp.nsecs = now.nsecs
                
            odom_pub.publish(odom_msg)         
                    
            rate.sleep()

if __name__ == '__main__':
    sim = SimRobot()
    sim.main()
