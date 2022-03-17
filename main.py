import numpy as np
import warnings

import asyncio

from computerVision import computerVision
from ballDetection2 import ballDetection2


white_upper = np.array([179,15,255])
white_lower = np.array([0,0,200])
red_lower = np.array([[160, 100, 20]])
red_upper = np.array([179, 255, 255])
orange_lower = np.array([0,180,100])
orange_upper = np.array([70,255,255])
green_lower = np.array([25, 52, 72])
green_upper = np.array([102, 255, 255])

ranges = [
   [green_lower,green_upper]
]


warnings.filterwarnings("ignore")
async def getNearestBall(speed=1):
    # first argument is either False or an array of integers corresponding to coco classes
    # 37 is for sports balls :)
    cv = computerVision(ranges)
    cv.start()
    motor_input = 0
    y = 9999
    x = None
    while (1):
        detections = None
        current_x = None
        detections = cv.detections
        y = 9999
        if (len(cv.detections) > 0):
            for detection in detections:
                if (detection[1] < y):
                    y = detection[0]
                    current_x = detection[0]
                    current_x = (current_x - 0.5) * 20
            if (current_x is not None):
                print(current_x)
        else:
            print("no Objects detected")
        await asyncio.sleep(speed)


#
# cv = computerVision(ranges)
# cv.start()
asyncio.run(getNearestBall(1))
getNearestBall(speed = 1)