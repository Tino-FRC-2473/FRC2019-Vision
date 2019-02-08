# add imports
import math
import cv2
import numpy as np
import os

# finds angle between robot's heading and the perpendicular to the targets
class VisionTargetDetector:

    # initilaze variables
    def __init__(self):

        self.angle = -99
        self.distance_to_target = -1

        os.system('sudo sh camerasettings.sh')
        self.camera = cv2.VideoCapture(1) # change dev/video[port_number] in camerasettings.sh
        _, frame = self.camera.read()


        # height of a vision target
        self.TARGET_HEIGHT = 5.5 * math.cos(math.radians(14.5)) + 2 * math.sin(math.radians(14.5))
        # intialize screen width and screen height
        self.SCREEN_HEIGHT, self.SCREEN_WIDTH = frame.shape[:2]
        # intialize angle of field of view in radians
        self.FIELD_OF_VIEW_RAD = 70.42 * math.pi / 180.0

        # calculates focal length based on a right triangle representing the "image" side of a pinhole camera
        # ABC where A is FIELD_OF_VIEW_RAD/2, a is SCREEN_WIDTH/2, and b is the focal length
        self.FOCAL_LENGTH_PIXELS = (self.SCREEN_WIDTH / 2.0) / math.tan(self.FIELD_OF_VIEW_RAD / 2.0)

    # calculates distance to target
    def calc_dist(self, length):
        if (length > 0):
            # calculated with ratios in the pinhole camera model
            return (self.FOCAL_LENGTH_PIXELS * self.TARGET_HEIGHT) / length

        return -1


    # given three rectangles, returns the two rectangles that should be facing eachother
    # def get_closest_rects(self, rect1, rect2, rect3):
    #         rects = [rect1, rect2, rect3]
    #         # sorts rectangles from left to rift
    #         rects.sort(key = lambda b: b.point4.x)
    #
    #         r1 = rects[0]
    #         r2 = rects[1]
    #         r3 = rects[2]
    #
    #         r1_top = r1.point3
    #         r2_top = r2.point3
    #
    #         r1_bottom = r1.point1
    #         r2_bottom = r2.point1
    #
    #
    #         top_distance = math.hypot(r1_top.x - r2_top.x, r1_top.y - r2_top.y)
    #         bottom_distance = math.hypot(r1_bottom.x - r2_bottom.x, r1_bottom.y - r2_bottom.y)
    #
    #         if(top_distance < bottom_distance):
    #             return rect1, rect2
    #         else:
    #             return rect2, rect3

    def get_closest_rects(self, rotated_boxes):

        pairs =[]

        counter = 0
        for rect in rotated_boxes:
            if(counter + 1 < len(rotated_boxes)):
                next_rect = rotated_boxes[counter + 1]
                r1_top = rect.point2
                r2_top = next_rect.point2

                r1_bottom = rect.point4
                r2_bottom = next_rect.point4

                top_distance = math.hypot(r1_top.x - r2_top.x, r1_top.y - r2_top.y)
                bottom_distance = math.hypot(r1_bottom.x - r2_bottom.x, r1_bottom.y - r2_bottom.y)

                if(top_distance < bottom_distance):
                    pairs.append(Pair(rect, next_rect))

                counter += 1



        target_pair = pairs[len(pairs)/2]

        return target_pair.left_rect, target_pair.right_rect

    # pin = Target
    def calc_ang_deg(self, pinX):
        return self.calc_ang_rad(pinX) * 180.0 / math.pi

    def calc_ang_rad(self, pinX):
        pin_dist_to_center = pinX - self.SCREEN_WIDTH / 2
        return math.atan(pin_dist_to_center/ self.FOCAL_LENGTH_PIXELS)

    # given 8 points (4 from rect1 and 4 from rect2), returns the average of all the x cooridnates and all the y coordinates
    def calc_pin_pos(self, rect1_point1, rect1_point2, rect1_point3, rect1_point4, rect2_point1, rect2_point2, rect2_point3, rect2_point4):
        x = (rect1_point1.x + rect1_point2.x + rect1_point3.x + rect1_point4.x + rect2_point1.x + rect2_point2.x + rect2_point3.x + rect2_point4.x) / 8.0
        y = (rect1_point1.y + rect1_point2.y + rect1_point3.y + rect1_point4.y + rect2_point1.y + rect2_point2.y + rect2_point3.y + rect2_point4.y) / 8.0
        return (int(x), int(y))

    def run_cv(self):
        _, frame = self.camera.read()
        # frame = cv2.imread('../test_pic.png')
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        low_green = np.array([60, 90.0, 50.0])
        high_green = np.array([87, 255, 229.0])

        mask = cv2.inRange(hsv, low_green, high_green)
        _, contours,_ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        #sorts contours by area
        #contours.sort(key = cv2.contourArea, reverse = True)

        def find_x(countour):
            x, _, _, _ = cv2.boundingRect(countour)
            return x

        contours.sort(key = lambda countour: find_x(countour))

        rotated_boxes = []
        rotated_rect1 = None
        rotated_rect2 = None
        rotated_rect3 = None
        self.pinX = 0
        self.pinY = 0

        for c in contours:
            area = cv2.contourArea(c)
            rect = cv2.minAreaRect(c)
            _,_, rot_angle = rect
            box = cv2.boxPoints(rect)
            box = np.int0(box)
            if area > 100:
                rotated_boxes.append(RotatedRectangle(box, area, rot_angle))

        if(len(rotated_boxes) > 1):
            # rotated_rect1 = rotated_boxes[0]
            # rotated_rect2 = rotated_boxes[1]
            # if(len(rotated_boxes) > 2) :
            #     rotated_rect3 = rotated_boxes[2]
            #     rotated_rect1, rotated_rect2 = self.get_closest_rects(rotated_rect1, rotated_rect2, rotated_rect3)

            rotated_rect1, rotated_rect2 = self.get_closest_rects(rotated_boxes)

            r1_point1, r1_point2, r1_point3, r1_point4 = rotated_rect1.point1, rotated_rect1.point2, rotated_rect1.point3, rotated_rect1.point4
            r2_point1, r2_point2, r2_point3, r2_point4 = rotated_rect2.point1, rotated_rect2.point2, rotated_rect2.point3, rotated_rect2.point4

            self.pinX, self.pinY = self.calc_pin_pos(r1_point1, r1_point2, r1_point3, r1_point4, r2_point1, r2_point2, r2_point3, r2_point4)

            # draws center circle
            cv2.circle(frame, (self.pinX, self.pinY), 1, (255, 0, 0), thickness=5)

        # if there are less than two rectangles, return -99, -1

        if(len(rotated_boxes)  == 1):
            rotated_rect1 = rotated_boxes[0]
            angle_to_rect = self.calc_ang_deg(rotated_rect1.point1.x)
            distance_to_target = 1.5 * self.calc_dist(rotated_rect1.get_height())
            return angle_to_rect, distance_to_target



        if (len(rotated_boxes) < 2):
            cv2.imshow("Contours", mask)
            cv2.imshow("Frame", frame)
            cv2.waitKey(3)
            return -99, -1



        angle = self.calc_ang_deg(self.pinX)
        self.angle_to_target = angle

        # multiplying 1.5 yields consistently more accurage results
        distance = 1.5 * self.calc_dist((rotated_rect1.get_height() + rotated_rect2.get_height()) / 2.0)
        self.distance_to_target = distance


        cv2.drawContours(frame, [rotated_rect1.box], 0, (0, 0, 255), 2)
        cv2.drawContours(frame, [rotated_rect2.box], 0, (0, 0, 255), 2)

        cv2.putText(frame, "ANG: " + str(angle), (0, 50), cv2.FONT_HERSHEY_DUPLEX, 1, (255, 255, 255))
        cv2.putText(frame, "DIST: " + str(distance), (0, 120), cv2.FONT_HERSHEY_DUPLEX, 1, (255, 255, 255))

        cv2.imshow("Contours", mask)
        cv2.imshow("Frame", frame)

        cv2.waitKey(3)

        return angle, distance


# this class defines a rotated rectantle around a countour
class RotatedRectangle:

    # box is a 2d list of rectangle coordiantes
    def __init__(self, box, area, rot_angle):
        self.box = box
        self.area = area
        self.rot_angle = rot_angle

        points = []
        for coordinates in box:
            points.append(Point(coordinates))

        #sorts points based on y value
        points.sort(key= lambda x: x.y)
        #lowest point
        self.point1 = points[0]
        #second lowest point
        self.point2 = points[1]
        #third lowest point
        self.point3 = points[2]
        #highest point
        self.point4 = points[3]

    def get_width(self):
        return abs(self.point3.x - self.point2.x)

    def get_height(self):
        return abs(self.point4.y - self.point1.y)

# this class defines a point
class Point:

    def __init__(self, coordinates):
        self.x = coordinates[0]
        self.y = coordinates[1]

class Pair:
    def __init__(self, left_rect, right_rect):
        self.left_rect = left_rect
        self.right_rect= right_rect
