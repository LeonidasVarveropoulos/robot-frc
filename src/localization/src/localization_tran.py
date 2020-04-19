#!/usr/bin/python

#import roslib; roslib.load_manifest('package_name')
import rospy
import tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped
#import time

class OdomTrans:
    def __init__(self):
        self.new_x_pose = 0
        self.new_y_pose = 0
        self.new_z_ori = 0
        self.new_w_ori = 0

    def odom_callback(self,rs_odom):
        global lx, ly, lz, ax, ay, az
        lx = rs_odom.twist.twist.linear.x
        ly = rs_odom.twist.twist.linear.y
        lz = rs_odom.twist.twist.linear.z
        
        ax = rs_odom.twist.twist.angular.x
        ay = rs_odom.twist.twist.angular.y
        az = rs_odom.twist.twist.angular.z

    def initial_callback(self,msg):
        self.new_x_pose = msg.pose.pose.position.x
        self.new_y_pose = msg.pose.pose.position.y

        self.new_z_ori = msg.pose.pose.orientation.z
        self.new_w_ori = msg.pose.pose.orientation.w

    def main(self):
        print("starting node")
        rospy.init_node('rs_to_odom')
        
        listener = tf.TransformListener()
        odom_sub = rospy.Subscriber("/t265/odom/sample", Odometry, self.odom_callback)
        initial_sub = rospy.Subscriber("/initialpose", PoseWithCovarianceStamped, self.initial_callback)

        odom_pub = rospy.Publisher("/base_pose_ground_truth", Odometry , queue_size=10)
        move_base_odom_pub = rospy.Publisher("/odom", Odometry , queue_size=10)
            
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
                                    
        while not rospy.is_shutdown():   
            try:    
                rospy.loginfo("odom info recieved %s"%(str([[lx,ly,lz],[ax,ay,az]])))
                rospy.loginfo("beginning odometry transformation")
            except NameError:
                continue
            else:
                break
        
        #lx, ly, lz, ax, ay, az = (0,0,0,0,0,0)
                                    
        #time.sleep(0.1) # allows listener buffer to load + callback variables
        rate = rospy.Rate(50)
        try:
            while not rospy.is_shutdown():
                try:
                    (trans,rot) = listener.lookupTransform('/t265_odom_frame', '/fake_base_link', rospy.Time(0))
                except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                    continue
                    
                # Negate to make it start "backwards"
                odom_msg.pose.pose.position.x = trans[0]
                odom_msg.pose.pose.position.y = trans[1]
                odom_msg.pose.pose.position.z = 0

                odom_msg.pose.pose.orientation.x = 0
                odom_msg.pose.pose.orientation.y = 0

                # Inorder to flip robot 180 degrees you must switch the z and w values: negate values to switch turn direction
                odom_msg.pose.pose.orientation.z = rot[2]
                odom_msg.pose.pose.orientation.w = rot[3]
                #print("z: ",rot[2],"    w: ", rot[3]) 
                            
                odom_msg.twist.twist.linear.x = -lx
                #odom_msg.twist.twist.linear.y = ly
                #odom_msg.twist.twist.linear.z = lz
                odom_msg.twist.twist.angular.x = ax
                odom_msg.twist.twist.angular.y = ay
                odom_msg.twist.twist.angular.z = az
                
                now  = rospy.get_rostime()
                odom_msg.header.stamp.secs = now.secs
                odom_msg.header.stamp.nsecs = now.nsecs
                
                odom_pub.publish(odom_msg)     

                # For move_base
                move_base_odom_pub.publish(odom_msg)       
                    
                rate.sleep()
                
        except rospy.ROSInterruptException:
            pass
if __name__ == '__main__':
    odom = OdomTrans()
    odom.main()
