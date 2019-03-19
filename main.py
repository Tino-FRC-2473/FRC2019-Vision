import serial
import time
from cv import VisionTargetDetector

s = serial.Serial(port='/dev/ttyTHS2', baudrate=9600)

camera_front = "0" #port number for front camera
#camera_back = "1" #port number for back camera

dTf = VisionTargetDetector(camera_front)
#dTb = VisionTargetDetector(camera_back)

def format_num(num):
    num = int(num)
    if num < -99: return "-99"
    if num > 999: return "999"
    if num < 0:
	return "-"+("%02d" % -num)
    else:
	return "%03d" % num

while True:
    '''af, df = detectorFront.run_cv()
    ab, db = detectorBack.run_cv()
    af = format_num(af)
    df = format_num(df)
    ab = format_num(ab)
    db = format_num(db)
    send_str = "S"+af+df+ab+db+"E"
    s.write(send_str)
    print(send_str)'''
    pf = dTf.run_cv()
    #pb = dTb.run_cv()
    while len(pf) is not 3:
	pf.append((-99, -99, -99))

    #while len(pb) is not 3:
    #    pb.append((-99, -99, -99))

    send_str = "S"
    for a, d, x in pf:
	send_str += format_num(a)
	send_str += format_num(d)
	send_str += format_num(x/2.0)
    
    '''for a, d, x in pb:
        send_str += format_num(a)
        send_str += format_num(d)
        send_str += format_num(x/2.0)'''

    send_str += "E"
    #print(send_str)
    s.write(send_str)
    #print pf
    #print pb

s.close()
    
