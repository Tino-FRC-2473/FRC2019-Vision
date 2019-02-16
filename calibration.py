import numpy as np
import cv2

#filler method for the trackbar so that we pass in a method that doesn't actually have a function
def nothing(x):
    pass

cap = cv2.VideoCapture(0)

'''
creating trackbars in a separate window to allow for a faster way of detecting green
or whatever color we are searching for
'''
cv2.namedWindow("Trackbars")
cv2.createTrackbar("L - Y", "Trackbars", 0, 1, nothing)
cv2.createTrackbar("L - U", "Trackbars", -0.5, 0.5, nothing)
cv2.createTrackbar("L - V", "Trackbars", -0.5, 0.5, nothing)
cv2.createTrackbar("U - Y", "Trackbars", 0, 1, nothing)
cv2.createTrackbar("U - U", "Trackbars", -0.5, 0.5, nothing)
cv2.createTrackbar("U - V", "Trackbars", -0.5, 0.5, nothing)




while(True):
    _, frame = cap.read()
    frame_hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2YUV)

    #linking the trackbars to variables to be used later
    l_h = cv2.getTrackbarPos("L - Y", "Trackbars")
    l_s = cv2.getTrackbarPos("L - U", "Trackbars")
    l_v = cv2.getTrackbarPos("L - V", "Trackbars")
    u_h = cv2.getTrackbarPos("U - Y", "Trackbars")
    u_s = cv2.getTrackbarPos("U - U", "Trackbars")
    u_v = cv2.getTrackbarPos("U - V", "Trackbars")


    #using the trackbars to change the lower and upper bounds for the mask
    #once done with figuring out the correct set of hsv values, we can use them in the cv.py file.
    lower_color = np.array([l_h, l_s, l_v])
    upper_color = np.array([u_h, u_s, u_v])
    mask = cv2.inRange(frame_hsv, lower_color, upper_color)
    result = cv2.bitwise_and(frame, frame, mask=mask)


    cv2.imshow('cap', frame)
    #cv2.imshow("mask", mask)
    cv2.imshow("result", result)

    #press 'q' to exit
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break


cap.release()
cv2.destroyAllWindows()
