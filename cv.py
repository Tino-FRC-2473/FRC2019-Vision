#ADD IMPORTS
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
        self.camera = cv2.VideoCapture(2)
        # camera = cv2.VideoCapture(1)
        _, frame = self.camera.read()
        self.SCREEN_HEIGHT, self.SCREEN_WIDTH = frame.shape[:2]
        self.ANGLE_CONST = (self.SCREEN_WIDTH / 2.0) / math.tan(self.FIELD_OF_VIEW_RAD / 2.0)


    def getRectPos(self, y1, h1, y3, h2):
        rectOnBottom = False
        if(y1 < y3):
            
            if( ((y1 + h1) - (y3 + h2)) < (y3 - y1) ):
                #deltaH = (y1 + h1) - (y3 + h2)
                rectOnBottom = True
            else:
                #deltaH = y3 - y1
                rectOnBottom = False


            #lengthRight = h1 - 2 * deltaH
            #print "deltaH: " + str(deltaH) + ", h1: " + str(h1)
            #length = (h1 + lengthRight) / 2.0
            #if(length == (h1 - deltaH)):
            #print "length: " + str(length)
            #else:
            #    print "fail"
        #if sec rect is higher
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

        # zero_red = np.array([0, 100, 100]);
        # low_red = np.array([10, 255, 255]);
        # high_red = np.array([160, 100, 100]);
        # pi_red = np.array([180, 255, 255]);

        # #make mask
        # maskLow = cv2.inRange(hsv, zero_red, low_red)
        # maskHigh = cv2.inRange(hsv, high_red, pi_red)
        # mask = maskLow + maskHigh

        #low_white = np.array([118 - 10,22.95 - 20,181.05 - 30]);
        #high_white = np.array([123 + 10, 22.95 + 20, 181.05 + 30]);
        #mask = cv2.inRange(hsv, low_white, high_white);

        #green thresholds
        #180, 17, 100
        #180, 1, 100
        #low_green = np.array([50, 50, 50])
        #high_green = np.array([90, 63.75, 255])
        low_green = np.array([60, 90.0, 50.0]) 
        high_green = np.array([87, 255, 229.0])

        #makes mask
        mask = cv2.inRange(hsv, low_green, high_green)

        #shows mask
        #cv2.imshow("Mask", mask)

        #find contours based on mask
        _, contour,_ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        #initializes rectangle variables with a dummy value
        max_area = 0
        mx,my,mw,mh = 0, 0, 0, 0 #variables x,y,width,height associated with the biggest rectangle

        secmax_area = 0
        sx,sy,sw,sh = 0, 0, 0, 0 #variables x,y,width,height associated with the second biggest rectangle

        thirdmax_area = 0
        tx,ty,tw,th = 0, 0, 0, 0 #variables x,y,width,height associated with the third biggest rectangle

        #initializes rectangle variables for side cases
        modmx, modmy, modmw, modmh = mx, my, mw, mh
        modsx, modsy, modsw, modsh = sx, sy, sw, sh

        #finds the two biggest rectangles
        for cnt in contour:
            #gets the x, y of the top left corner and 
            #width and height of the rectangle
            x,y,w,h = cv2.boundingRect(cnt)
            #cv2.rectangle(frame,(x,y),(x+w,y+h),(0,150,255),thickness=3)

            #gets area of the rectangle
            area = cv2.contourArea(cnt)
            #print str(area)

            #checks if the rectangle is greater than the max_area so far and
            #sets the variables to accommodate the change
            if(area >= max_area):
                thirdmax_area = secmax_area
                secmax_area = max_area
                max_area = area
                
                tx = sx
                ty = sy
                tw = sw
                th = sh

                sx = mx
                sy = my
                sw = mw
                sh = mh

                mx = x
                my = y
                mw = w
                mh = h
            #checks if the rectangle is greater than the secmax_area so far and
            #sets the variables to accommodate the change
            elif(area >= secmax_area):
                thirdmax_area = secmax_area
                secmax_area = area
                
                tx = sx
                ty = sy
                tw = sw
                th = sh

                sx = x
                sy = y
                sw = w
                sh = h
            #checks if the rectangle is greater than the thirdmax_area so far and
            #sets the variables to accommodate the change
            elif(area >= thirdmax_area):
                thirdmax_area = area

                tx = x
                ty = y
                tw = w
                th = h



        sideCase = False
        oneRect = False

        #draw rectangles on two biggest green part found, draws green rectangles
        if(max_area > 0):
            #draws a green rect on the biggest rectangle found
            cv2.rectangle(frame,(mx,my),(mx+mw,my+mh),(0,255,0),thickness=3)
            #sets threshold for second rectangle length
            if(sh > 0.3 * mh and sw > 0):
                #draws a red rect on the second biggest rectangle found
                cv2.rectangle(frame,(sx,sy),(sx+sw,sy+sh),(0,0,255),thickness=3)

                #sets rectangle variables of side cases 
                #to the current max and secmax rectangle variacles
                modmx, modmy, modmw, modmh = mx, my, mw, mh
                modsx, modsy, modsw, modsh = sx, sy, sw, sh
                
                #sets a threshold for side cases 
                #determines if the case can qualify as a side cases
                lengthThres = 0.905
                if(sh < lengthThres * mh or mh < lengthThres * sh):

                    #if third rectangle exists
                    if(thirdmax_area > 0):
                        errorThres = 3
                        #draws the third retangle in cyan
                        cv2.rectangle(frame,(tx,ty),(tx+tw,ty+th),(255,255,0),thickness=3)
                        

                        #check if the third rectangle is in the secmax rect area 
                        #(should be this case 100% of time and not be above the max rect)
                        if(tx >= sx-errorThres and tx+tw <= sx+sw+errorThres and
                            ty >= my-errorThres and ty+th <= my+mh+errorThres and
                            (ty <= sy+errorThres or ty+th >= sy+sh-errorThres)):
                            sideCase = True
                            
                            #third rect on top of the max rect
                            if(ty < sy):
                                modsy = ty
                                modsh = sy+sh-ty
                            #third rect is below the max rect
                            else:
                                modsh = ty+th-sy

                        #check if the third rectangle is in the max rect area
                        elif(tx >= mx-errorThres and tx+tw <= mx+mw+errorThres and
                            ty >= sy-errorThres and ty+th <= sy+sh+errorThres and
                            (ty <= my+errorThres or ty+th >= my+mh-errorThres)):
                            sideCase = True
                            
                            #third rect on top of the max rect
                            if(ty < my):
                                modmy = ty
                                modmh = my+mh-ty
                            #third rect is below the max rect
                            else:
                                modmh = ty+th-my
                    

                    #checking the peg covering the top case
                    #if side case is not true yet
                    if(not sideCase):
                        if(mh > sh): #probs 100% of the time
                            #scale the sh so that the ratio of the heights 
                            #is equal to the ratio of the widths
                            modsh = int(float(sw) / mw * mh)
                            sideCase = True
                        else:
                            #scale the mh so that the ratio of the heights 
                            #is equal to the ratio of the widths
                            modmh = int(float(mw) / sw * sh)
                            sideCase = True
                        
                        #print "smaller than threshold but not third rect"

                #finds out if the third rectangle or where the peg covers the rectanges
                #would be on the top or bottom        
                rectPos = self.getRectPos(my, mh, sy, sh)
                print "rectPos: " + str(rectPos)

                #draws magenta rectangles for side case situations 
                #to visulize the rectangles we are doing the calculations to
                if(modsh <= 0 or modmh <= 0):
                   print "modsh or modmh 0"
                elif(sideCase):
                #else:
                    cv2.putText(frame, "modmh: " + str(modmh) + ", modsh: " + str(modsh), (0, 200), cv2.FONT_HERSHEY_SIMPLEX, 2, 255)
                    if(rectPos == self.TOP):
                        cv2.rectangle(frame,(modsx,modsy),(modsx+modsw,modsy+modsh),(255,0,255),thickness=1)
                        cv2.rectangle(frame,(modmx,modmy),(modmx+modmw,modmy+modmh),(255,0,255),thickness=1)
                        print "top"


                    else:
                        modsy = sy + sh - modsh
                        modmy = my + mh - modmh
                        cv2.rectangle(frame,(modsx,modsy),(modsx+modsw,modsy+modsh),(255,0,255),thickness=1)
                        cv2.rectangle(frame,(modmx,modmy),(modmx+modmw,modmy+modmh),(255,0,255),thickness=1)
                        #cv2.putText(frame, "modmh: " + str(modmh) + ", modsh: " + str(modsh), (50, 200), cv2.FONT_HERSHEY_SIMPLEX, 2, 255)
                        print "bottom"
                
                cv2.putText(frame, "width: " + str(mw) + ", " + str(sw), (0, 150), cv2.FONT_HERSHEY_SIMPLEX, 2, 255)

                #draws the new rectangles, purple


                """#draws diagonal lines, yellow lines
                #top left corner to bottom right corner
                #max rect is on left of secmax rect
                if(mx < sx):
                    #top left corner to bottom right corner
                    cv2.line(frame, (mx, my), (sx+sw, sy+sh), (0, 255, 255), thickness=5)
                    #top right corner to bottm left corner
                    cv2.line(frame, (sx+sw, sy), (mx, my+mh), (0, 255, 255), thickness=5)
                else: #secmax rect is on left of max rect
                    #top left corner to bottom right corner
                    cv2.line(frame, (sx, sy), (mx+mw, my+mh), (0, 255, 255), thickness=5)
                    #top right corner to bottm left corner
                    cv2.line(frame, (mx+mw, my), (sx, sy+sh), (0, 255, 255), thickness=5)"""

                #finds the approximate position of the pin and draws a blue circle in that position
                self.pinX, self.pinY = self.pinPosition(mx, my, mx+mw, my+mh, sx, sy, sx+sw, sy+sh)
                cv2.circle(frame, (self.pinX, self.pinY), 1, (255, 0, 0), thickness=5)

                #diagPinX, diagPinY = crossPinPos(modmx, modmy, modmw, modmh, modsx, modsy, modsw, modsh)
                #cv2.circle(frame, (diagPinX, diagPinY), 1, (0, 0, 0), thickness=5)

            #one rectangle case, when second rectagle is too small/nonexistent
            else:
                #print "one rect case"
                oneRect = True
                #if the rectangle is on the right side of the screen
                if(mx + mw/2.0 > self.SCREEN_WIDTH / 2.0):
                    self.pinX = mx + 5.125/5*mh
                else:
                    self.pinX = mx - 3.125/5*mh
                #print self.pinX


        #sets the angle variable which we will return
        #finds the angle the robot needs to turn
        #for the robot to be centered at the peg
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

