import cv2
import numpy as np
import math
from shape_detector import DetectShape

class ContourDetect:

    def __init__(self):
        self.detect_shape = DetectShape()
        self.last_x = 0
        self.last_y = 0

        # Change if camera is not in line with shooter
        self.camera_offset = 0

        # Feedback of PID
        self.x_offset = 0
        self.wanted_x = 0

        # Used for distance
        self.y_offset = 0
        self.distance = 0

        # Change to change dynamicly
        self.x_threshold = 100
        self.y_threshold = 100

        self.target_locked = False

    def contour_detect(self, mask, is_tracking):

        height, width = mask.shape

        # Setpoint of PID
        self.wanted_x = width/2 + self.camera_offset

        if (is_tracking):
            # Detects edges to make easier for contours
            edges = cv2.Canny(mask, 100, 150, apertureSize=3)

            # List of all raw contours includes noise
            _, contours, _= cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            # Where we eliminate some of the shape noise
            contour_shape = []

            # Filters
            if (len(contours) >= 1):

                # Filters by shape
                for contour in contours:
                    perimeter = cv2.arcLength(contour,True)

                    if (self.detect_shape.detect(contour, perimeter)):
                        contour_shape.append(contour)

                contour_biggest = [contour_shape[0]]

                # Filter by biggest contour
                if (len(contour_shape) >= 1):
                    for contour in contour_shape:
                        (x, y, w, h) = cv2.boundingRect(contour)
                        (big_x, big_y, big_w, big_h) = cv2.boundingRect(contour_biggest[0])
                        area = self.get_area(w,h)
                        biggest_area = self.get_area(big_w, big_h) 
                        if (area > biggest_area):
                            contour_biggest = []
                            contour_biggest.append(contour)

                if (len(contour_biggest) >= 1):
                    self.target_locked = True
                    (x,y,w,h) = cv2.boundingRect(contour_biggest[0])
                    cv2.rectangle(mask, (x ,y - h), (x + w, y + h), (255, 0, 0), 2)

                    cv2.circle(mask, (self.get_center(x, y+h,w, h).x, self.get_center(x, y-h, w, h).y), 10, (255, 0, 0))

                    self.x_offset = self.get_x_offset(self.get_center(x, y-h, w, h).x, width)
                    self.y_offset = self.get_y_offset(self.get_center(x, y-h, w, h).y, height)

                    self.distance = self.get_distance(self.y_offset)
                    #print(self.distance)

                    self.last_x = x
                    self.last_y = y
                else:
                    self.target_locked = False
            else:
                self.target_locked = False
        else:
            self.target_locked = False
        
        # Need to comment out for not testing
        #cv2.imshow('Contour image',mask)

    def get_center(self, x, y, w, h):
        return Point(x + (w/2), y + h)

    def get_x_offset(self, x, width):
        return x - (self.wanted_x)

    def get_y_offset(self, y, height):
        return y - (height/2)

    def get_rotation_pid(self):
        return RotationPID(self.x_offset, self.wanted_x)

    def get_distance(self, y_offset):
        print(y_offset)
        return ((263.95) / (1.0 + ( 46.8851 * math.pow(math.e, -0.008061 * y_offset) )) ) + 4.88508
    
    def get_area(self, w, h):
        return (w * h) * 2

    def is_target_locked(self):
        return self.target_locked


class Point:
    def __init__(self, x, y):
        self.x = x
        self.y = y

class RotationPID:
    def __init__(self, x_offset, wanted_x):
        self.feedback = x_offset
        self.setpoint = wanted_x