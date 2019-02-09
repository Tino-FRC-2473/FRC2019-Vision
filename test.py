import socket
import time
import os
import subprocess
import cv2
import argparse
from datetime import datetime
from cv import VisionTargetDetector

# "python test.py 0" to run from camera in port 0
# "python test.py video.mp4" to run from the video recording video.mp4
# "python test.py --out output.mp4" to save output to a recording file

parser = argparse.ArgumentParser()
parser.add_argument("input", help="read from the given camera or video", default="0")
parser.add_argument("-o", "--out", help="save video output", action="store_true")
args = parser.parse_args()

output = ""

print "Reading from", args.input

if args.out:
    output = "videos/" + datetime.now().strftime('%Y-%m-%d-%H:%M:%S') + ".avi"
    print "Writing to", output
    # if videos folder doesn't exist, create it
    if not os.path.isdir("videos"):
        subprocess.call(["mkdir", "videos"])
        print "made videos"


detector = VisionTargetDetector(args.input, output)

with detector as d:
    while True:
        angle, distance = d.run_cv()

# close input and writer
detector.release_cv_objects()

if args.out:
    print "Video writing successful"
