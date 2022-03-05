import numpy as np
from computerVision import computerVision

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

cv = computerVision(ranges)
cv.start()
