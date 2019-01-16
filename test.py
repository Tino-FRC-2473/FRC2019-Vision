import socket
import time
from cv import VisionTargetDetector

detector = VisionTargetDetector()

while True:
    time.sleep(0.1)
    angle, distance = detector.runCV()
    print str(angle), str(distance)
