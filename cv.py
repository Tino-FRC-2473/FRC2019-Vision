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

        os.system('sudo sh camerasettings.sh')
        self.camera = cv2.VideoCapture(1)
        _, frame = self.camera.read()

        self.SCREEN_HEIGHT, self.SCREEN_WIDTH = frame.shape[:2]

        self.FIELD_OF_VIEW_RAD = 70.42 * math.pi / 180.0
        self.DIST_CONSTANT = 3300 # need to test this
        self.ANGLE_CONST = (self.SCREEN_WIDTH / 2.0) / math.tan(self.FIELD_OF_VIEW_RAD / 2.0)

    def calcDist(self, length):
        if(length > 0):
            return self.DIST_CONSTANT / length
        return -1

    def getClosestRects(self, r1, r2, r3):
        dist1 = math.hypot(r1.x-r2.x, r1.y - r2.y)
        dist2 = math.hypot(r1.x-r3.x, r1.y - r3.y)
        dist3 = math.hypot(r2.x-r3.x, r2.y - r3.y)
        maximum = max(dist1, dist2, dist3)

        if maximum == dist1: return r1, r2
        elif maximum == dist2: return r1, r3
        else: return r2, r3

    def calcAngleDeg(self, pinX):
        return self.calcAngleRad(pinX) * 180.0 / math.pi

    def calcAngleRad(self, pinX):
        pinDistToCenter = pinX - self.SCREEN_WIDTH / 2
        return math.atan(pinDistToCenter / self.ANGLE_CONST)

    def calcPinPosition(self, x1, y1, x2, y2, x3, y3, x4, y4):
        x = (x1 + x2 + x3 + x4) / 4.0
        y = (y1 + y2 + y3 + y4) / 4.0
        return (int(x), int(y))

    def runCV(self):
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

        for c in contours[:3]:
            x, y, w, h = cv2.boundingRect(c)
            area = cv2.contourArea(c)
            maxContours.append(Rectangle(x, y, w, h, area))

        if (len(maxContours) < 2) :
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
            rect1, rect2 = self.getClosestRects(rect1, rect2, rect3)

        self.pinX = 0
        self.pinY = 0
        #draw rectangles on two biggest green part found, draws green rectangles
        if(rect1.area > 0):
            cv2.rectangle(frame, (rect1.x, rect1.y), (rect1.x + rect1.width, rect1.y + rect1.height), (0, 255, 0), thickness=3)
            #sets threshold for second rectangle length
            if(rect2.height > 0.3 * rect1.width and rect2.width > 0):
                cv2.rectangle(frame, (rect2.x, rect2.y), (rect2.x + rect2.width, rect2.y + rect2.height), (0,255,0), thickness=3)

                #finds the approximate position of the pin and draws a blue circle in that position
                self.pinX, self.pinY = self.calcPinPosition(rect1.x, rect1.y, rect1.x + rect1.width, rect1.y + rect1.height, rect2.x, rect2.y, rect2.x + rect2.width, rect2.y + rect2.height)
                cv2.circle(frame, (self.pinX, self.pinY), 1, (255, 0, 0), thickness=5)

            #one rectangle case, when second rectagle is too small/nonexistent
            else:
                oneRect = True
                if(rect1.x + rect1.width/2.0 > self.SCREEN_WIDTH / 2.0):
                    self.pinX = rect1.x + 5.125/5*rect1.height
                else:
                    self.pinX = rect1.x - 3.125/5*rect1.height


        angle = self.calcAngleDeg(self.pinX)
        distance = self.calcDist((rect1.height + rect2.height) / 2.0)
        if (oneRect):
             distance = self.calcDist(rect1.height)

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
