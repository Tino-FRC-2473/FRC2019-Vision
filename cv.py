# add imports
import math
import cv2
import numpy as np
import os

# finds angle between robot's heading and the perpendicular to the targets
class VisionTargetDetector:

    # initilaze variables
    def __init__(self):

        self.angle = -1

        #os.system('sudo sh camerasettings.sh')
        self.camera = cv2.VideoCapture(0) # change dev/video[port_number] in camerasettings.sh
        _, frame = self.camera.read()

        self.SCREEN_HEIGHT, self.SCREEN_WIDTH = frame.shape[:2]
        self.FIELD_OF_VIEW_RAD = 70.42 * math.pi / 180.0

        # calculates focal length based on a right triangle representing the "image" side of a pinhole camera
        # ABC where A is FIELD_OF_VIEW_RAD/2, a is SCREEN_WIDTH/2, and b is the focal length
        self.FOCAL_LENGTH_PIXELS = (self.SCREEN_WIDTH / 2.0) / math.tan(self.FIELD_OF_VIEW_RAD / 2.0)

    def calc_dist(self, length):
        if (length > 0):
            # calculated with ratios in the pinhole camera model
            # 5.5 is the length of the reflective tape in inches
            return (self.FOCAL_LENGTH_PIXELS * 5.5) / length
        return -1

    def get_closest_rects(self, r1, r2, r3):
        dist1 = math.hypot(r1.x-r2.x, r1.y - r2.y)
        dist2 = math.hypot(r1.x-r3.x, r1.y - r3.y)
        dist3 = math.hypot(r2.x-r3.x, r2.y - r3.y)
        maximum = max(dist1, dist2, dist3)

        if maximum == dist1: return r1, r2
        elif maximum == dist2: return r1, r3
        else: return r2, r3

    def calc_ang_deg(self, pinX):
        return self.calc_ang_rad(pinX) * 180.0 / math.pi

    def calc_ang_rad(self, pinX):
        pinDistToCenter = pinX - self.SCREEN_WIDTH / 2
        return math.atan(pinDistToCenter / self.FOCAL_LENGTH_PIXELS)

    def calc_pin_pos(self, rect1_point1, rect1_point2, rect1_point3, rect1_point4, rect2_point1, rect2_point2, rect2_point3, rect2_point4):
        x = (rect1_point1.x + rect1_point2.x + rect1_point3.x + rect1_point4.x + rect2_point1.x + rect2_point2.x + rect2_point3.x + rect2_point4.x) / 8.0
        y = (rect1_point1.y + rect1_point2.y + rect1_point3.y + rect1_point4.y + rect2_point1.y + rect2_point2.y + rect2_point3.y + rect2_point4.y) / 8.0
        return (int(x), int(y))

    def run_cv(self):
        _, frame = self.camera.read()
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        low_green = np.array([60, 90.0, 50.0])
        high_green = np.array([87, 255, 229.0])

        mask = cv2.inRange(hsv, low_green, high_green)
        _, contours,_ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        contours.sort(key=cv2.contourArea, reverse=True)
        maxContours = []
        rotatedBoxes = []
        rotated_rect1 = None
        rotated_rect2 = None

        for c in contours[:3]:
            x, y, w, h = cv2.boundingRect(c)
            area = cv2.contourArea(c)
            maxContours.append(Rectangle(x, y, w, h, area))
            if(area > 500):
                rect = cv2.minAreaRect(c)
                _,_, rot_angle = rect
                box = cv2.boxPoints(rect)
                box = np.int0(box)
                rotatedBoxes.append(RotatedRectangle(box, area, rot_angle))

        if(len(rotatedBoxes) > 1):
            rotated_rect1 = rotatedBoxes[0]
            rotated_rect2 = rotatedBoxes[1]

            cv2.drawContours(frame, [rotated_rect1.box], 0, (0, 0, 255), 2)
            # cv2.putText(frame, "Rect1: " + str(rotated_rect1.rot_angle), (rotated_rect1.box[0][0], rotated_rect1.box[0][1]), cv2.FONT_HERSHEY_DUPLEX, 2, 255)

            cv2.drawContours(frame, [rotated_rect2.box], 0, (0, 0, 255), 2)
            # cv2.putText(frame, "Rect2: " + str(rotated_rect2.rot_angle), (rotated_rect2.box[0][0], rotated_rect2.box[0][1]), cv2.FONT_HERSHEY_DUPLEX, 2, 255)

            top_point1 = max(rotated_rect1.box, key=lambda x: x[1])
            top_point2 = max(rotated_rect2.box, key=lambda x: x[1])

            bottom_point1 = min(rotated_rect1.box, key=lambda x: x[1])
            bottom_point2 = min(rotated_rect2.box, key=lambda x: x[1])

            top_dist = math.hypot(top_point2[0] - top_point1[0], top_point2[1] - top_point1[1])
            bottom_dist = math.hypot(bottom_point2[0] - bottom_point1[0], bottom_point2[1] - bottom_point1[1])

            # draws lines connecting the two top points of the targets and the two bottom points
            cv2.line(frame, (top_point1[0], top_point1[1]), (top_point2[0], top_point2[1]), (255, 0, 0), 5)
            cv2.line(frame, (bottom_point1[0], bottom_point2[1]), (bottom_point2[0], bottom_point2[1]), (255, 0, 0), 5)

        if (len(maxContours) < 2):
            cv2.imshow("Contours", mask)
            cv2.imshow("Frame", frame)
            cv2.waitKey(3)
            return -1, -1

        rect1 = maxContours[0]
        rect2 = maxContours[1]
        rect3 = None

        if len(maxContours) < 3:
            rect3 = Rectangle(0, 0, 0, 0, 0)
        else:
            rect3 = maxContours[2]

        oneRect = False

        if rect3.area != 0:
            rect1, rect2 = self.get_closest_rects(rect1, rect2, rect3)

        self.pinX = 0
        self.pinY = 0

        # draw rectangles on two biggest green part found, draws green rectangles
        if(rect1.area > 0 and len(rotatedBoxes) > 1):
            # cv2.rectangle(frame, (rect1.x, rect1.y), (rect1.x + rect1.width, rect1.y + rect1.height), (0, 255, 0), thickness=3)
            # cv2.rectangle(frame, (rect2.x, rect2.y), (rect2.x + rect2.width, rect2.y + rect2.height), (0,255,0), thickness=3)

            # finds the approximate position of the pin and draws a blue circle in that position
            #self.pinX, self.pinY = self.calc_pin_pos(rect1.x, rect1.y, rect1.x + rect1.width, rect1.y + rect1.height, rect2.x, rect2.y, rect2.x + rect2.width, rect2.y + rect2.height)
            self.pinX, self.pinY = self.calc_pin_pos(rotated_rect1.point1, rotated_rect1.point2, rotated_rect1.point3, rotated_rect1.point4, rotated_rect2.point1,rotated_rect2.point2, rotated_rect2.point3, rotated_rect1.point4)
            cv2.circle(frame, (self.pinX, self.pinY), 1, (255, 0, 0), thickness=5)

        # one rectangle case, when second rectagle is too small/nonexistent
        else:
            oneRect = True
            if(rect1.x + rect1.width/2.0 > self.SCREEN_WIDTH / 2.0):
                self.pinX = rect1.x + 5.125/5*rect1.height
            else:
                self.pinX = rect1.x - 3.125/5*rect1.height
		#understand why these specific numbers are used in this algorithm

        angle = self.calc_ang_deg(self.pinX)
        distance = self.calc_dist((rect1.height + rect2.height) / 2.0)
        if (oneRect):
             distance = self.calc_dist(rect1.height)

        cv2.putText(frame, "ANG: " + str(angle), (0, 50), cv2.FONT_HERSHEY_DUPLEX, 1, (255, 255, 255))
        cv2.putText(frame, "DIST: " + str(distance), (0, 120), cv2.FONT_HERSHEY_DUPLEX, 1, (255, 255, 255))

        cv2.imshow("Contours", mask)
        cv2.imshow("Frame", frame)

        cv2.waitKey(3)

        return angle, distance

# holds rectangle properties
class Rectangle:

    def __init__(self, x, y, width, height, area):
        self.x = x
        self.y = y
        self.width = width
        self.height = height
        self.area = area

class RotatedRectangle:

    # box is a 2d list of rectangle coordiantes
    def __init__(self, box, area, rot_angle):
        self.box = box
        self.area = area
        self.rot_angle = rot_angle

        points = []
        for coordinates in box:
            points.append(Point(coordinates))

        points.sort(key= lambda x: x.y)
        self.point1 = points[0]
        self.point2 = points[1]
        self.point3 = points[2]
        self.point4 = points[3]


class Point:

    def __init__(self, coordinates):
        self.x = coordinates[0]
        self.y = coordinates[1]
