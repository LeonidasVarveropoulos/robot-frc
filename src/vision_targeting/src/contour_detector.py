import cv2
import numpy as np
import math
from shape_detector import DetectShape

class ContourDetect:

    def __init__(self, camera_offset):
        self.detect_shape = DetectShape()
        self.last_x = 0
        self.last_y = 0

        # Change if camera is not in line with shooter
        self.camera_offset = camera_offset

        # Feedback of PID
        self.x_offset = 0
        self.wanted_x = 0

        # Used for distance
        self.y_offset = 0

        self.target_locked = False

    def contour_detect(self, mask, is_tracking):
        """ Filters the contours to a single contour and gets data from it """

        height, width = mask.shape

        # Setpoint of PID
        self.wanted_x = width/2 + self.camera_offset

        if (is_tracking):
            # Detects edges to make easier for contours
            edges = cv2.Canny(mask, 100, 150, apertureSize=3)

            # List of all raw contours includes noise
            _, contours, _= cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            # Where we eliminate some of the contours by shape
            contour_shape = []

            # Filters if there is a contour detected
            if (len(contours) >= 1):

                # Filters by shape
                for contour in contours:
                    perimeter = cv2.arcLength(contour,True)

                    # Filters based on shape
                    if (self.detect_shape.detect(contour, perimeter)):
                        contour_shape.append(contour)

                # Filter by biggest contour
                if (len(contour_shape) >= 1):
                    selected_contour = contour_shape[0]

                    for contour in contour_shape:
                        # Gets the dimensions
                        (x, y, w, h) = cv2.boundingRect(contour)
                        (big_x, big_y, big_w, big_h) = cv2.boundingRect(selected_contour)

                        # Gets area of each contour
                        area = self.get_area(w,h)
                        biggest_area = self.get_area(big_w, big_h)

                        # Checks which one is bigger and makes it the selected contour
                        if (area > biggest_area):
                            selected_contour = contour
                else:
                    selected_contour = None

                # Use filtered contour to get data
                if (selected_contour is not None):
                    self.target_locked = True

                    # Gets the position and dimensions of the contour
                    (x,y,w,h) = cv2.boundingRect(selected_contour)

                    # Draws rectangle around contour
                    cv2.rectangle(mask, (x ,y - h), (x + w, y + h), (255, 0, 0), 2)

                    # Draws circle around center of contour
                    cv2.circle(mask, (self.get_center(x, y+h,w, h).x, self.get_center(x, y-h, w, h).y), 10, (255, 0, 0))
                    
                    # Updates feedback for pids
                    self.x_offset = self.get_x_offset(self.get_center(x, y-h, w, h).x)
                    self.y_offset = self.get_y_offset(self.get_center(x, y-h, w, h).y, height)

                    self.last_x = x
                    self.last_y = y
                else:
                    self.target_locked = False
            else:
                self.target_locked = False
        else:
            self.target_locked = False
        # cv2.imshow("Mask",mask)

    def get_center(self, x, y, w, h):
        """ This returns a Point which is the center of the contour """
        return Point(x + (w/2), y + h)

    def get_x_offset(self, x):
        """ Returns the x_offset used as feedback to the pid """
        return x - (self.wanted_x)

    def get_y_offset(self, y, height):
        """ Returns the y_offset from the bottom of the frame used in a regression """
        y_offset = y - (height/2)

        return ((263.95) / (1.0 + ( 46.8851 * math.pow(math.e, -0.008061 * y_offset) )) ) + 4.88508

    def get_rotation_pid(self):
        """ Returns the x_offset used as feedback in a pid """
        return self.x_offset
    
    def get_area(self, w, h):
        """ Returns the area """
        return (w * h) * 2

    def is_target_locked(self):
        """ Checks if the filter it tracking a target """
        return self.target_locked


class Point:
    def __init__(self, x, y):
        self.x = x
        self.y = y
