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

        # inside camerasettings.sh dev/video[port_number]
        os.system('sudo sh camerasettings.sh')
        self.camera = cv2.VideoCapture(1)
        _, frame = self.camera.read()

        self.SCREEN_HEIGHT, self.SCREEN_WIDTH = frame.shape[:2]

        self.FIELD_OF_VIEW_RAD = 70.42 * math.pi / 180.0
        self.FOCAL_LENGTH_PIXELS = (self.SCREEN_WIDTH / 2.0) / math.tan(self.FIELD_OF_VIEW_RAD / 2.0) #angle constant is the same as focal length of camera (pixels)

    def calc_dist(self, length):
        if(length > 0):
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

    def calc_pin_pos(self, x1, y1, x2, y2, x3, y3, x4, y4):
        x = (x1 + x2 + x3 + x4) / 4.0
        y = (y1 + y2 + y3 + y4) / 4.0
        return (int(x), int(y))

    def run_cv(self):
        _, frame = self.camera.read()
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        low_green = np.array([60, 90.0, 50.0])
        high_green = np.array([87, 255, 229.0])

        mask = cv2.inRange(hsv, low_green, high_green)
        _, contours,_ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        def sort_by_area(c):
            return cv2.contourArea(c)

        contours.sort(key=sort_by_area, reverse=True)
        maxContours = []
        rotatedBoxes = []

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

        if(len(rotatedBoxes) >=2):
            rotated_rect1 = rotatedBoxes[0]
            rotated_rect2 = rotatedBoxes[1]
            cv2.imshow("Frame", frame)
            print "printing rect1"
            print rotated_rect1.area
            print rotated_rect1.box
            print rotated_rect1.rot_angle

            print "printing rect2"
            print rotated_rect2.area
            print rotated_rect2.box
            print rotated_rect2.rot_angle

            cv2.drawContours(frame,[rotated_rect1.box],0,(0,0,255),2)
            cv2.putText(frame, "Rect1: " + str(rotated_rect1.rot_angle) , (rotated_rect1.box[0][0], rotated_rect1.box[0][1]), cv2.FONT_HERSHEY_DUPLEX, 2, 255)

            cv2.drawContours(frame,[rotated_rect2.box],0,(0,0,255),2)
            #angle is not right. idk what it outputs
            cv2.putText(frame, "Rect2: " + str(rotated_rect2.rot_angle), (rotated_rect2.box[0][0], rotated_rect2.box[0][1]), cv2.FONT_HERSHEY_DUPLEX, 2, 255)



            top_point1 = max(rotated_rect1.box, key=lambda x: x[1])
            top_point2 = max(rotated_rect2.box, key=lambda x: x[1])

            bottom_point1 = min(rotated_rect1.box, key=lambda x: x[1])
            bottom_point2 = min(rotated_rect2.box, key=lambda x: x[1])


            top_dist = math.hypot(top_point2[0] - top_point1[0], top_point2[1] - top_point1[1])
            bottom_dist = math.hypot(bottom_point2[0] - bottom_point1[0], bottom_point2[1] - bottom_point1[1])
            print "top dist: " + str(top_dist)
            print "bottom dist: " + str(bottom_dist)


            #i dont know how the points work
            cv2.line(frame, (top_point1[0], top_point1[1]), (top_point2[0], top_point2[1]),(255,0,0),5)
            cv2.line(frame, (bottom_point1[0], bottom_point2[1]), (bottom_point2[0], bottom_point2[1]),(255,0,0),5)

        if (len(maxContours) < 2):
            cv2.imshow("Contours", mask)
            cv2.imshow("Frame", frame)
            cv2.waitKey(3)
            return -1, -1

        rect1 = maxContours[0]
        rect2 = maxContours[1]
        rect3 = None

        if len(maxContours) < 3 :
            rect3 = Rectangle(0, 0, 0, 0, 0)
        else :
            rect3 = maxContours[2]

        oneRect = False

        if rect3.area != 0:
            rect1, rect2 = self.get_closest_rects(rect1, rect2, rect3)

        self.pinX = 0
        self.pinY = 0

        #draw rectangles on two biggest green part found, draws green rectangles
        if(rect1.area > 0):
            cv2.rectangle(frame, (rect1.x, rect1.y), (rect1.x + rect1.width, rect1.y + rect1.height), (0, 255, 0), thickness=3)
            #sets threshold for second rectangle length
            if(rect2.height > 0.3 * rect1.width and rect2.width > 0): #make this if statement cleaner
                cv2.rectangle(frame, (rect2.x, rect2.y), (rect2.x + rect2.width, rect2.y + rect2.height), (0,255,0), thickness=3)

                #finds the approximate position of the pin and draws a blue circle in that position
                self.pinX, self.pinY = self.calc_pin_pos(rect1.x, rect1.y, rect1.x + rect1.width, rect1.y + rect1.height, rect2.x, rect2.y, rect2.x + rect2.width, rect2.y + rect2.height)
                cv2.circle(frame, (self.pinX, self.pinY), 1, (255, 0, 0), thickness=5)

            #one rectangle case, when second rectagle is too small/nonexistent
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

        cv2.putText(frame, "ANG: " + str(angle), (0, 50), cv2.FONT_HERSHEY_DUPLEX, 2, 255)
        cv2.putText(frame, "DIST: " + str(distance), (0, 120), cv2.FONT_HERSHEY_DUPLEX, 2, 255)

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

    def __init__(self, box, area, rot_angle):
        self.box = box
        self.area = area
        self.rot_angle = rot_angle
