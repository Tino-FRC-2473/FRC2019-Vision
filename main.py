import serial
import time
from cv import VisionTargetDetector

s = serial.Serial(port='/dev/ttyTHS2', baudrate=9600)

camera_front = 0 #port number for front camera
camera_back = 1 #port number for back camera

detectorFront = VisionTargetDetector(camera_front)
detectorBack = VisionTargetDetector(camera_back)

def format_num(num):
    num = int(num)
    if num < -99: return "-99"
    if num > 999: return "999"
    if num < 0:
	return "-"+("%02d" % -num)
    else:
	return "%03d" % num

while True:
    af, df = detectorFront.run_cv()
    ab, db = detectorBack.run_cv()
    af = format_num(af)
    df = format_num(df)
    ab = format_num(ab)
    db = format_num(db)
    send_str = "S"+af+df+ab+db+"E"
    s.write(send_str)
    print(send_str)

s.close()
    
