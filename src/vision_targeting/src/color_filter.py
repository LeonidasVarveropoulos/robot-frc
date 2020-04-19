import cv2
from matplotlib import pyplot as plt
import numpy as np

class ColorFilter:
    def __init__(self):
        pass

    def color_filter(self,frame,color_low,color_high,output):

        frame = cv2.GaussianBlur(frame, (5,5),0)

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        #kernel = np.ones((60,60),np.uint8)
        #kernel2 = np.ones((10,10),np.uint8)

        lower_color = np.array(color_low)
        upper_color = np.array(color_high)

        mask = cv2.inRange(hsv,lower_color,upper_color)
        #res = cv2.bitwise_and(frame,frame, mask= mask)

        #opening = cv2.morphologyEx(res, cv2.MORPH_OPEN, kernel2)
        #closing = cv2.morphologyEx(res, cv2.MORPH_CLOSE, kernel)

        #norm_img = cv2.cvtColor(res,cv2.COLOR_HSV2BGR)
        #final_gray = cv2.cvtColor(norm_img,cv2.COLOR_BGR2GRAY)

        #cv2.imshow("Mask",mask)
        
        #if output == "res":
            #return res
        if output == "mask":
            return mask
        #if output == "gray":
            #return final_gray