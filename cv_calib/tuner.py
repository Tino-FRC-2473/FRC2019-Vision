import cv2
import numpy as np

cam = cv2.VideoCapture('output.avi')

cv2.namedWindow('Calibrations')

def nothing(x):
  pass

cv2.createTrackbar("H-L", "Calibrations",60,255,nothing)
cv2.createTrackbar("H-H", "Calibrations",87,255,nothing)
cv2.createTrackbar("S-L", "Calibrations",90,255,nothing)
cv2.createTrackbar("S-H", "Calibrations",255,255,nothing)
cv2.createTrackbar("V-L", "Calibrations",50,255,nothing)
cv2.createTrackbar("V-H", "Calibrations",229,255,nothing)

ret, frame = cam.read()
while True:
	#ret, frame = cam.read()
	hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    	
	hl = cv2.getTrackbarPos("H-L", "Calibrations")
	hh = cv2.getTrackbarPos("H-H", "Calibrations")
	sl = cv2.getTrackbarPos("S-L", "Calibrations")
	sh = cv2.getTrackbarPos("S-H", "Calibrations")
	vl = cv2.getTrackbarPos("V-L", "Calibrations")
	vh = cv2.getTrackbarPos("V-H", "Calibrations")
	
	low_green = np.array([hl, sl, vl])
	high_green = np.array([hh, sh, vh])
	mask = cv2.inRange(hsv, low_green, high_green)
	mask = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
	
	both = np.hstack((frame, mask))
	#both = np.concatenate((frame, mask), axis=0)

	cv2.imshow('Calibrations', both)
	
	key = cv2.waitKey(1) & 0xFF
	if key == ord('q'):
		break
	elif key == ord('w'):
		print('low: ['+str(hl)+', '+str(sl)+', '+str(vl)+']')
		print('high: ['+str(hh)+', '+str(sh)+', '+str(vh)+']')
		print('------------------')
		ret, frame = cam.read()
    	
