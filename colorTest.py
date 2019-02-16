import numpy as np
import cv2

frame = cv2.imread("../Desktop/white3.png")

yuv = cv2.cvtColor(frame, cv2.COLOR_BGR2YUV)

low_green = np.array([0, 0, 0])
high_green = np.array([255, 160, 100])
y, u, v = cv2.split(yuv)

y_avg = np.average(yuv[:,:,0])
u_avg = np.average(yuv[:,:,1])
v_avg = np.average(yuv[:,:,2])


y_std = np.std(yuv[:,:,0])
u_std = np.std(yuv[:,:,1])
v_std = np.std(yuv[:,:,2])

print ("y:", y_avg, y_std)
print ("u:", u_avg, u_std)
print ("v:", v_avg, v_std)
mask = cv2.inRange(yuv, low_green, high_green)

