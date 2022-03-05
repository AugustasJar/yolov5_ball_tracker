from imutils.video import VideoStream
import numpy as np
import cv2 #pip install opencv-contrib-python
import imutils
import time
import math
from threading import Thread

class computerVision:

    def __init__(self, ranges):
        self.vs = VideoStream(0)
        self.vs.start()
        time.sleep(1.0)
        self.minimum_countour_area = 300
        self.ranges = ranges
        self.gaussian_blur_kernel = (7, 7)
        self.image_width = 300
        self.active = True
        self.detections = []

        self.videoLoop()

    def start(self):
        Thread(target=self.videoLoop(),args=()).start()
        return self


    def stop(self):
        self.active = False

    def videoLoop(self):
        while self.active:
            # grab the current frame
            frame = self.vs.read()
            if frame is None:
                break

            frame = imutils.resize(frame, width=self.image_width)
            mask = self.create_mask(frame, self.ranges)

            cnts = self.findCountours(mask)
            frame = self.displayDetection(frame, cnts)
            self.addDetections(cnts)

            cv2.imshow("Frame", frame)

            key = cv2.waitKey(100) & 0xFF
            if key == ord("q"):
                break
        self.vs.stop()
        cv2.destroyAllWindows()

    # proprecesses the image based on a hsv range (LowerBounds, upperBounds)
    def create_mask(self, image, ranges):
        hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        blurred = cv2.GaussianBlur(hsv_image, self.gaussian_blur_kernel, 0)
        image = None
        for range in ranges:
            mask = cv2.inRange(blurred, range[0], range[1])
            mask = mask
            mask = cv2.erode(mask, None, iterations=2)
            mask = cv2.dilate(mask, None, iterations=5)
            if (image is not None):
                image = cv2.add(image, mask)
            else:
                image = mask
        return image

    # dumb contour detection, need to add template matching or something.
    def findCountours(self, mask):
        cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cnts = imutils.grab_contours(cnts)
        l = list()
        for c in cnts:
            area = cv2.contourArea(c)
            if area > 300:
                ((x, y), radius) = cv2.minEnclosingCircle(c)
                if math.pi * pow(int(radius), 2) - area < area * 0.90:
                    l.append(c)
        return l

    # displays contours on the screen
    def displayDetection(self, frame, cnts):
        if len(cnts) > 0:
            for c in cnts:
                if (cv2.contourArea(c) > 300):
                    ((x, y), radius) = cv2.minEnclosingCircle(c)
                    M = cv2.moments(c)
                    center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
                    # cv2.circle(frame, (int(x), int(y)), int(radius),(0, 255, 255), 2)
                    cv2.circle(frame, center, 5, (0, 0, 255), -1)
        return frame

    def addDetections(self, cnts):
        l = []
        if len(cnts) > 0:
            for c in cnts:
                M = cv2.moments(c)
                center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
                l.append(center)
        self.detections = l



