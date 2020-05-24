#!/usr/bin/env python
from __future__ import print_function

import cv2
from matplotlib import pyplot as plt
import numpy as np
import roslib
import sys
import rospy
from std_msgs.msg import String
from std_msgs.msg import Float32, Float64, Bool
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge, CvBridgeError

from color_filter import ColorFilter
from contour_detector import ContourDetect

rospy.init_node('vision_targeting', anonymous=True)
class VisionTargeting:

  def __init__(self):
    self.bridge = CvBridge()

    # ROS img msg
    if (rospy.get_param("image_type", "Image") == "Image"):
      self.image_sub = rospy.Subscriber("d435/color/image_raw",CompressedImage,self.callback)
    else:
      self.image_sub = rospy.Subscriber("d435/color/image_raw",CompressedImage,self.callback)

    self.prime_sub = rospy.Subscriber("turret/primed", Bool,self.prime_callback)
    
    # Rotation PID values
    self.feedback_pub = rospy.Publisher("turret/feedback", Float32, queue_size=1)
    self.setpoint_pub = rospy.Publisher("turret/setpoint", Float32, queue_size=1)

    # This will be used to calculate hood angle and flywheel rpm
    self.y_offset_pub = rospy.Publisher("turret/y_offset", Float32, queue_size=1)

    self.cf = ColorFilter()
    self.cd = ContourDetect(rospy.get_param("~rotation_setpoint", 20))

    # Subscribe to ros topic to tell if it should track
    self.is_tracking = True

  def prime_callback(self,data):
    """ This lets the node know if it should be tracking """
    self.is_tracking = data

  def callback(self,data):
    """ This runs when a new img frame is detected """
    try:
      if (rospy.get_param("image_type", "Image") == "Image"):
        cv_image = self.bridge.compressed_imgmsg_to_cv2(data)
      else:
        cv_image = self.bridge.compressed_imgmsg_to_cv2(data)
    except CvBridgeError as e:
      print(e)

    # Color Filter the Video Stream (For Green)
    mask = self.cf.color_filter(cv_image, rospy.get_param("~low_hsv", [0,0,0]), rospy.get_param("~high_hsv", [0,0,0]))

    # Passes the mask to contour detector
    self.cd.contour_detect(mask.copy(), self.is_tracking)

    # Pid values for rotation
    r_feedback = Float32()
    r_setpoint = Float32()

    # This will be used to calculate hood angle and flywheel rpm
    y_offset = Float32()

    r_feedback.data = self.cd.get_rotation_pid()
    r_setpoint.data = rospy.get_param("~rotation_setpoint", 0)

    y_offset.data = self.cd.y_offset

    self.feedback_pub.publish(r_feedback)
    self.setpoint_pub.publish(r_setpoint)

    self.y_offset_pub.publish(y_offset)

    cv2.waitKey(3)

def main(args):
  dvt = VisionTargeting()
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
