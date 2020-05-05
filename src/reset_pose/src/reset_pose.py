#!/usr/bin/python

import rospy
import tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import Float32, Float64, Bool, Float32MultiArray
from geometry_msgs.msg import Twist
import math

# Creates the node
rospy.init_node('reset_pose')

class ResetPose:
    def __init__(self):
        self.reset_x = 0
        self.reset_y = 0
        self.reset_th = 0

        # Does not matter because code is run in callback
        self.rate = 10

        # A Dictionary of wanted topic name and data type
        self.input_odom = rospy.get_param('~input_odom', [])

        # Collection of ROS subscribers and actual data
        self._data = {}

        # List of ROS publishers and subscribers to avoid calling one twice
        self._publishers = {}
        self._subscribers = []

        self.robot_reset = False

    def subscribe(self, topic_name, data_type):
        """ This sets up the ros subscribers for incoming data """
        # Checks if subscriber exists, if not create one
        if topic_name in self._subscribers:
            return
        if data_type is Odometry:
            rospy.Subscriber(topic_name, data_type, self.odom_callback)
        else:
            rospy.Subscriber(topic_name, data_type, self._on_new_data)
        self._subscribers.append(topic_name)

    def publish(self, topic_name, data_type, data, queue = 10, latching = False):
        """ This publishes ros data """
        # Check if publisher exists, if not create and publish data
        if not topic_name in self._publishers:
            self._publishers[topic_name] = rospy.Publisher(topic_name, data_type, queue_size=queue, latch=latching)
        self._publishers[topic_name].publish(data)

    def get_data(self, topic_name, simple_data = True):
        """ This gets the subscribed ros data """

        if topic_name in self._data:
            if simple_data:
                return self._data[topic_name].data
            else:
                return self._data[topic_name]
        return [0,0,0]

    def odom_callback(self, msg):
        new_msg = msg

        quat = (new_msg.pose.pose.orientation.x, new_msg.pose.pose.orientation.y, new_msg.pose.pose.orientation.z, new_msg.pose.pose.orientation.w)
        euler = tf.transformations.euler_from_quaternion(quat)
        th = euler[2]

        # If true resets robot pose to 0,0,0
        if self.robot_reset == True:
            rospy.logwarn_throttle(15, "Robot Resetting Pose")
            self.reset_x = new_msg.pose.pose.position.x
            self.reset_y = new_msg.pose.pose.position.y
            self.reset_th = th

            self.robot_reset = False
        new_msg.pose.pose.position.x -= self.reset_x - self.get_data("/robot_set_pose")[0]
        new_msg.pose.pose.position.y -= self.reset_y - self.get_data("/robot_set_pose")[1]

        # Wrapping of the orientation
        th -= self.reset_th - self.get_data("/robot_set_pose")[2]
        if th < 0:
            th += (math.pi * 2)

        quaternion = tf.transformations.quaternion_from_euler(0, 0, th)
        new_msg.pose.pose.orientation.z = quaternion[2] 
        new_msg.pose.pose.orientation.w = quaternion[3]
        
        # Publishes to new topic that goes to the robot_pose_ekf
        self.publish(str(msg._connection_header["topic"]) + "_reset", Odometry, new_msg)
    
    def _on_new_data(self, msg):
        """ This is the callback function for the subscribers """
        self._data[str(msg._connection_header["topic"])] = msg
        self.robot_reset = True

    def main(self):

        # Sets up the subscribers
        for odom in self.input_odom:
            self.subscribe(odom["topic"], Odometry)
        self.subscribe("/robot_set_pose", Float32MultiArray)

        rate = rospy.Rate(self.rate)

        # Required for node
        while not rospy.is_shutdown():         
            rate.sleep()

if __name__ == '__main__':
    reset = ResetPose()
    reset.main()
