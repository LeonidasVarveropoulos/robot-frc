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
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

from color_filter import ColorFilter
from contour_detector import ContourDetect

class VisionTargeting:

  def __init__(self):
    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("d435/color/image_raw",Image,self.callback)
    self.prime_sub = rospy.Subscriber("turret/primed", Bool,self.prime_callback)

    self.feedback_pub = rospy.Publisher("turret/feedback", Float32, queue_size=1)
    self.setpoint_pub = rospy.Publisher("turret/setpoint", Float32, queue_size=1)

    self.distance_pub = rospy.Publisher("turret/distance", Float32, queue_size=1)

    self.target_locked_pub = rospy.Publisher("turret/target_loc", Bool, queue_size=1)

    self.cf = ColorFilter()
    self.cd = ContourDetect()

    # Subscribe to ros topic to tell if it should track
    self.is_tracking = True

  def prime_callback(self,data):
    self.is_tracking = data

  def callback(self,data):
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)

    #(rows,cols,channels) = cv_image.shape

    # Color Filter the Video Stream (For Green)
    mask = self.cf.color_filter(cv_image,[52.1685761047 , 72.1198754541 , 96.1428685482],[95.5321144864 , 255 , 255],"mask")

    # Prev [50, 125,125],[90, 255,255]
    #Low Values: ( 52.1685761047 , 72.1198754541 , 96.1428685482
    #High Values: ( 95.5321144864 , 255 , 255
    
    #cv2.imshow('Image window', mask)
    #cv2.imshow('Image', cv_image)

    self.cd.contour_detect(mask.copy(), self.is_tracking)

    target_locked_data = Bool()

    r_feedback = Float32()
    r_setpoint = Float32()

    distance = Float32()

    target_locked_data.data = self.cd.is_target_locked()

    r_feedback.data = self.cd.get_rotation_pid().feedback
    r_setpoint.data = 20.0#self.cd.get_rotation_pid().setpoint

    distance.data = self.cd.distance

    self.feedback_pub.publish(r_feedback)
    self.setpoint_pub.publish(r_setpoint)

    self.distance_pub.publish(distance)

    cv2.waitKey(3)

def main(args):
  dvt = VisionTargeting()
  rospy.init_node('vision_targeting', anonymous=True)
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
