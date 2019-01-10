import socket
import time
from cv import VisionTargetDetector

detector = VisionTargetDetector()

HOST = ''
PORT = 5801

s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

s.bind((HOST, PORT))
print "listening..."
s.listen(1)
conn, addr = s.accept()
print 'Connected by', addr

while True:
    time.sleep(0.1)
    angle = detector.runCV()
    print str(angle)
    conn.sendall(str(angle)+'\n')
