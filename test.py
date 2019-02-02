import socket
import time
from cv import VisionTargetDetector

detector = VisionTargetDetector()

while True:
    time.sleep(1)
    angle, distance, angle_to_parallel = detector.run_cv()
    print str(angle), str(distance), str(angle_to_parallel)
