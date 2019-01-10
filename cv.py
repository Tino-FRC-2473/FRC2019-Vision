import math
import cv2
import numpy as np
import os

#finds and returns the angle to Deep Space Vision Targets
class VisionTargetDetector:

    #initializes the variables used in this class
    def __init__(self):
        #rectSize = 10; # set the rext size

        #For angle
        #calc the constant
        self.FIELD_OF_VIEW_RAD = 70.42 * math.pi / 180.0;
        #note: total not just half of screen
        #edges always distorted
        self.SCREEN_WIDTH = 0;
        self.ANGLE_CONST = 0;

        #calc angle
        self.angle = -1
        self.SCREEN_HEIGHT = 0;
        self.pinX = 0;
        self.pinY = 0;
        self.pinDistToCenter = 0;

        self.TOP = False
        self.BOTTOM = True
        # angle;

        os.system('sudo sh camerasettings.sh')
        self.camera = cv2.VideoCapture(0)
        # camera = cv2.VideoCapture(1)
        _, frame = self.camera.read()
        self.SCREEN_HEIGHT, self.SCREEN_WIDTH = frame.shape[:2]
        self.ANGLE_CONST = (self.SCREEN_WIDTH / 2.0) / math.tan(self.FIELD_OF_VIEW_RAD / 2.0)

    def getClosestRects(r1, r2, r3):
        dist1 = math.hypot(r1[1]-r2[1], r1[2]-r2[2])
        dist2 = math.hypot(r1[1]-r3[1], r1[2]-r3[2])
        dist3 = math.hypot(r2[1]-r3[1], r2[2]-r3[2])
        maximum = max(dist1, dist2, dist3)

        if maximum == dist1: return r1, r2
        elif maximum == dist2: return r1, r3
        else: return r2, r3

    def getRectPos(self, y1, h1, y3, h2):
        rectOnBottom = False
        if(y1 < y3):
            
            if( ((y1 + h1) - (y3 + h2)) < (y3 - y1) ):
                #deltaH = (y1 + h1) - (y3 + h2)
                rectOnBottom = True
            else:
                #deltaH = y3 - y1
                rectOnBottom = False

        else:
            #deltaH = y1 - y3
            if( ((y3 + h2) - (y1 + h1)) < (y1 - y3) ):
                #deltaH = (y3 + h2) - (y1 + h1)
                rectOnBottom = True
            else:
                #deltaH = y1 - y3
                rectOnBottom = False

        return rectOnBottom

    #calculates the angle in degrees
    #we need to turn to be centered with the back of the board
    def calcAngleDeg(self, pinX):
        return self.calcAngleRad(pinX) * 180.0 / math.pi

    #calculates the angle in radians
    #we need to turn to be centered with the back of the board
    def calcAngleRad(self, pinX):
        pinDistToCenter = self.calcPinDist(pinX)
        #returns it in radians
        return math.atan(pinDistToCenter / self.ANGLE_CONST)

    #helper method to calculate the horizontal distance
    #between the center of the screen and the peg in pixels
    def calcPinDist(self, pinX):
        #if the peg is on the right side of the screen, POSITIVE
        #peg on left side of screen, NEGATIVE
        return (pinX - self.SCREEN_WIDTH / 2);
        #return math.fabs(pinX - SCREEN_WIDTH / 2);

    #calculates the approximate position of the peg on the screen
    def pinPosition(self, x1, y1, x2, y2, x3, y3, x4, y4):
        x = (x1 + x2 + x3 + x4) / 4.0;
        y = (y1 + y2 + y3 + y4) / 4.0;
        return (int(x), int(y))

    #returns the calculated angle
    def runCV(self):
        #gets a frame/picture of what the camera is getting
        _, frame = self.camera.read()

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        low_green = np.array([60, 90.0, 50.0]) 
        high_green = np.array([87, 255, 229.0])

        #makes mask
        mask = cv2.inRange(hsv, low_green, high_green)

        #shows mask
        #cv2.imshow("Mask", mask)

        #find contours based on mask
        _, contour,_ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        #initializes rectangle variables with a dummy value
        max_rect = (0, 0, 0, 0, 0)
        smax_rect = (0, 0, 0, 0, 0)
        tmax_rect = (0, 0, 0, 0, 0)

        #finds the three biggest rectangles
        for cnt in contour:
            x,y,w,h = cv2.boundingRect(cnt)

            area = cv2.contourArea(cnt)

            if(area >= max_rect[0]):
                tmax_rect = smax_rect
                smax_rect = max_rect
                max_rect = (area, x, y, w, h)
            elif(area >= smax_rect[0]):
                tmax_rect = smax_rect
                smax_rect = (area, x, y, w, h)
            elif(area >= tmax_rect[0]):
                tmax_rect = (area, x, y, w, h)

        
        oneRect = False
        
        #find two closest rectangles
        if tmax_rect[0] != 0:
            max_rect, smax_rect = self.getClosestRects(max_rect, smax_rect, tmax_rect)

        #draw rectangles on two biggest green part found, draws green rectangles
        if(max_rect[0] > 0):
            cv2.rectangle(frame,(max_rect[1],max_rect[2]),(max_rect[1]+max_rect[3],max_rect[2]+max_rect[4]),(0,255,0),thickness=3)
            #sets threshold for second rectangle length
            if(smax_rect[4] > 0.3 * max_rect[3] and smax_rect[3] > 0):
                cv2.rectangle(frame,(smax_rect[1],smax_rect[2]),(smax_rect[1]+smax_rect[3],smax_rect[2]+smax_rect[4]),(0,0,255),thickness=3)

                #finds the approximate position of the pin and draws a blue circle in that position
                self.pinX, self.pinY = self.pinPosition(max_rect[1], max_rect[2], max_rect[1]+max_rect[3], max_rect[2]+max_rect[4], smax_rect[1], smax_rect[2], smax_rect[1]+smax_rect[3], smax_rect[2]+smax_rect[4])
                cv2.circle(frame, (self.pinX, self.pinY), 1, (255, 0, 0), thickness=5)

            #one rectangle case, when second rectagle is too small/nonexistent
            else:
                oneRect = True
                if(max_rect[1] + max_rect[3]/2.0 > self.SCREEN_WIDTH / 2.0):
                    self.pinX = max_rect[1] + 5.125/5*max_rect[4]
                else:
                    self.pinX = max_rect[1] - 3.125/5*max_rect[4]


        angle = self.calcAngleDeg(self.pinX)


        #displays data on the screen such as the angle
        cv2.putText(frame, "ANG: " + str(angle), (0, 50), cv2.FONT_HERSHEY_SIMPLEX, 2, 255)

        #displays data on the console such as the angle
        #print "Angle: " + str(angle)
        #print "--------------------"

        #displays the screen showing the contours
        #cv2.imshow("Contours", mask)

        #displays the frame, which we use to visualize rectangles with the drawing
        #cv2.imshow("Frame", frame)

        #adding lag time so we can look at the data more carefully
        cv2.waitKey(3)

        #returns and angle we calculated
        return angle