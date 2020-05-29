#!/usr/bin/python

import rospy
import tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped

class GlobalPose:
    def __init__(self):
        self.pose_pub = rospy.Publisher(rospy.get_param("output_pose", "/global_pose"), PoseWithCovarianceStamped , queue_size=10)

        rospy.Subscriber(rospy.get_param("input_pose", "/robot_pose_ekf/odom_combined"), PoseWithCovarianceStamped, self.pose_callback)

        self.pose_msg = PoseWithCovarianceStamped()
        
        self.pose_msg.header.frame_id = "map"
        
        self.pose_msg.pose.covariance = [0.01, 0.0, 0.0, 0.0, 0.0, 0.0,
                                    0.0, 0.01, 0.0, 0.0, 0.0, 0.0,
                                    0.0, 0.0, 0.01, 0.0, 0.0, 0.0,
                                    0.0, 0.0, 0.0, 0.0001, 0.0, 0.0,
                                    0.0, 0.0, 0.0, 0.0, 0.0001, 0.0,
                                    0.0, 0.0, 0.0, 0.0, 0.0, 0.0001]

        self.listener = tf.TransformListener()

    def pose_callback(self, msg):
        map_frame = "/map"
        base_link_frame = "/base_link"

        try:
            (trans,rot) = self.listener.lookupTransform(map_frame, base_link_frame, rospy.Time(0))
        except:
            return
        
        self.pose_msg.pose.pose.position.x = trans[0]
        self.pose_msg.pose.pose.position.y = trans[1]
        self.pose_msg.pose.pose.position.z = 0

        self.pose_msg.pose.pose.orientation.x = 0
        self.pose_msg.pose.pose.orientation.y = 0
        
        self.pose_msg.pose.pose.orientation.z = rot[2]
        self.pose_msg.pose.pose.orientation.w = rot[3]
        
        now  = rospy.get_rostime()
        self.pose_msg.header.stamp.secs = now.secs
        self.pose_msg.header.stamp.nsecs = now.nsecs
        
        self.pose_pub.publish(self.pose_msg)

    def main(self):
        rate = rospy.Rate(5)

        # Required for node
        while not rospy.is_shutdown():         
            rate.sleep()
        
if __name__ == '__main__':
    rospy.init_node('global_pose')
    global_pose = GlobalPose()
    global_pose.main()