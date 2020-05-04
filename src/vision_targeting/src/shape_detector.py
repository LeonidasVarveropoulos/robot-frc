import cv2
import numpy as np

class DetectShape:

    def __init__(self):
        pass
    
    def detect(self,contour,perimeter):
        """ Returns true if the given contour has a certain number of sides """
        
        # Values for the filter
        peri = perimeter
        approx = cv2.approxPolyDP(contour,0.04 * peri, True)

        if (len(approx) == 7 or 6):
            return True
        else:
            return False