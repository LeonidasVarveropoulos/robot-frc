import cv2
from matplotlib import pyplot as plt
import numpy as np

class ColorFilter:
    def __init__(self):
        pass

    def color_filter(self, frame, color_low, color_high):
        """ This filters the img based on hsv values """
        
        # Blurs for better contour detection
        frame = cv2.GaussianBlur(frame, (5,5),0)
        
        # Converts from bgr to hsv img
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # Max and min values to filter
        lower_color = np.array(color_low)
        upper_color = np.array(color_high)

        # Filters the hsv values
        mask = cv2.inRange(hsv,lower_color,upper_color)
        
        # Returns the filtered img
        return mask