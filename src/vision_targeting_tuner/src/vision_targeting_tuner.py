#!/usr/bin/env python
from __future__ import print_function

import cv2
from matplotlib import pyplot as plt
from matplotlib.widgets import Slider, Button, RadioButtons
import numpy as np
import roslib
import sys
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge, CvBridgeError

# Setting up the sliders
axcolor = 'lightgoldenrodyellow'
ax1 = plt.axes([0.25, 0.1, 0.65, 0.03])
ax2 = plt.axes([0.25, 0.15, 0.65, 0.03])
ax3 = plt.axes([0.25, 0.2, 0.65, 0.03])
ax4 = plt.axes([0.25, 0.25, 0.65, 0.03])
ax5 = plt.axes([0.25, 0.3, 0.65, 0.03])
ax6 = plt.axes([0.25, 0.35, 0.65, 0.03])

low_h = Slider(ax1, 'low_h', 0, 255)
low_s = Slider(ax2, 'low_s', 0, 255)
low_v = Slider(ax3, 'low_v', 0, 255)

high_h = Slider(ax4, 'high_h', 0, 255)
high_s = Slider(ax5, 'high_s', 0, 255)
high_v = Slider(ax6, 'high_v', 0, 255)

class VisionTargetingTuner:

  def __init__(self):
    self.bridge = CvBridge()
    
    # Subscribes to img
    self.image_sub = rospy.Subscriber("d435/color/image_raw",CompressedImage,self.callback)
    
    # Default values
    self.low_h_val = 0
    self.low_s_val = 0
    self.low_v_val = 0

    self.high_h_val = 0
    self.high_s_val = 0
    self.high_v_val = 0

  def callback(self,data):
    # Converts to opencv image
    try:
      cv_image = self.bridge.compressed_imgmsg_to_cv2(data)
    except CvBridgeError as e:
      print(e)
    
    frame = cv2.GaussianBlur(cv_image, (5,5),0)

    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # HSV color range filter
    color_low = [self.low_h_val, self.low_s_val, self.low_v_val]
    color_high = [self.high_h_val, self.high_s_val, self.high_v_val]

    lower_color = np.array(color_low)
    upper_color = np.array(color_high)

    # Filtered out img
    mask = cv2.inRange(hsv,lower_color,upper_color)

    # Shows img
    cv2.imshow("Mask",mask)

    cv2.waitKey(3)

  def update(self,val):

      # Gets the new values
      self.low_h_val = low_h.val
      self.low_s_val = low_s.val
      self.low_v_val = low_v.val

      self.high_h_val = high_h.val
      self.high_s_val = high_s.val
      self.high_v_val = high_v.val

      # Logs HSV values for use in vision targeting
      rospy.loginfo("Low Values: (%s, %s, %s)" %(self.low_h_val, self.low_s_val, self.low_v_val))
      rospy.loginfo("High Values: (%s, %s, %s)" %(self.high_h_val, self.high_s_val, self.high_v_val))

def main(args):
  rospy.init_node('vision_targeting_tuner', anonymous=True)
  dvt = VisionTargetingTuner()

  # Updates the values
  low_h.on_changed(dvt.update)
  low_s.on_changed(dvt.update)
  low_v.on_changed(dvt.update)

  high_h.on_changed(dvt.update)
  high_s.on_changed(dvt.update)
  high_v.on_changed(dvt.update)

  plt.show()
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)