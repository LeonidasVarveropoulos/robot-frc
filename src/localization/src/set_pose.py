#!/usr/bin/python

import rospy
import tf
import tf2_ros
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import Float32, Float64, Bool, Float32MultiArray
from geometry_msgs.msg import Twist, TransformStamped
import math

# Creates the node
rospy.init_node('set_pose')

# Collection of ROS subscribers and actual data
global _data
_data = {}

# List of ROS publishers and subscribers to avoid calling one twice
global _publishers, _subscribers
_publishers = {}
_subscribers = []

global robot_reset
robot_reset = False

class SetPose:
    def __init__(self):
        self.reset_x = 0
        self.reset_y = 0
        self.reset_th = 0

    def subscribe(self, topic_name, data_type):
        """ This sets up the ros subscribers for incoming data """

        global _subscribers

        # Checks if subscriber exists, if not create one
        if topic_name in _subscribers:
            return
        if data_type is Odometry:
            rospy.Subscriber(topic_name, data_type, self.odom_callback)
        else:
            rospy.Subscriber(topic_name, data_type, self._on_new_data)
        _subscribers.append(topic_name)

    def publish(self, topic_name, data_type, data, queue = 10, latching = False):
        """ This publishes ros data """

        global _publishers

        # Check if publisher exists, if not create and publish data
        if not topic_name in _publishers:
            _publishers[topic_name] = rospy.Publisher(topic_name, data_type, queue_size=queue, latch=latching)
        _publishers[topic_name].publish(data)

    def get_data(self, topic_name, simple_data = True):
        """ This gets the subscribed ros data """

        global _data

        if topic_name in _data:
            if simple_data:
                return _data[topic_name].data
            else:
                return _data[topic_name]
        return [0,0,0]

    def odom_callback(self, msg):
        global robot_reset

        new_msg = msg

        quat = (new_msg.pose.pose.orientation.x, new_msg.pose.pose.orientation.y, new_msg.pose.pose.orientation.z, new_msg.pose.pose.orientation.w)
        euler = tf.transformations.euler_from_quaternion(quat)
        th = euler[2]

        # If true resets robot pose to 0,0,0
        if robot_reset == True:
            rospy.logwarn_throttle(15, "Robot Resetting Pose")
            self.reset_x = new_msg.pose.pose.position.x
            self.reset_y = new_msg.pose.pose.position.y
            self.reset_th = th

            self.set_start_pose()

            robot_reset = False

        new_msg.pose.pose.position.x -= self.reset_x
        new_msg.pose.pose.position.y -= self.reset_y

        # Wrapping of the orientation
        th -= self.reset_th
        if th < 0:
            th += (math.pi * 2)

        quaternion = tf.transformations.quaternion_from_euler(0, 0, th)
        new_msg.pose.pose.orientation.z = quaternion[2] 
        new_msg.pose.pose.orientation.w = quaternion[3]
        
        # Publishes to new topic that goes to the robot_pose_ekf
        self.publish(str(msg._connection_header["topic"]) + "_reset", Odometry, new_msg)
    
    def _on_new_data(self, msg):
        """ This is the callback function for the subscribers """

        global _data
        global robot_reset

        _data[str(msg._connection_header["topic"])] = msg
        robot_reset = True

    def set_start_pose(self):
        broadcaster = tf2_ros.StaticTransformBroadcaster()
        static_transformStamped = TransformStamped()

        static_transformStamped.header.stamp = rospy.Time.now()
        static_transformStamped.header.frame_id = "map"
        static_transformStamped.child_frame_id = "odom"

        static_transformStamped.transform.translation.x = self.get_data("/robot_set_pose")[0]
        static_transformStamped.transform.translation.y = self.get_data("/robot_set_pose")[1]
        static_transformStamped.transform.translation.z = 0.0

        quat = tf.transformations.quaternion_from_euler(
                   0.0, 0.0, self.get_data("/robot_set_pose")[2])
        static_transformStamped.transform.rotation.x = quat[0]
        static_transformStamped.transform.rotation.y = quat[1]
        static_transformStamped.transform.rotation.z = quat[2]
        static_transformStamped.transform.rotation.w = quat[3]

        broadcaster.sendTransform(static_transformStamped)

def main():
    # A Dictionary of wanted topic name and data type
    input_odom = rospy.get_param('~input_odom', [])

    # Sets up the subscribers
    for odom in input_odom:
        set_pose = SetPose()
        set_pose.subscribe(odom["topic"], Odometry)
        set_pose.subscribe("/robot_set_pose", Float32MultiArray)
        set_pose.set_start_pose()

    rate = rospy.Rate(5)

    # Required for node
    while not rospy.is_shutdown():         
        rate.sleep()

if __name__ == '__main__':
    main()