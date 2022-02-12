import cv2
import numpy as np
import torch
from imutils.video import VideoStream
import time
from threading import Thread

class ballDetection():
    def __init__(self,classes,camera):
        self.classes = classes
        self.model = torch.hub.load('ultralytics/yolov5', 'yolov5n', pretrained=True)
        self.vs = VideoStream(camera)
        self.vs.start()
        self.model.conf = 0.25  # NMS confidence threshold
        self.model.iou = 0.45  # NMS IoU threshold
        self.model.agnostic = False  # NMS class-agnostic
        self.model.multi_label = False  # NMS multiple labels per box
        # self.model.classes = classes  # (optional list) filter by class, i.e. = [0, 15, 16] for COCO persons, cats and dogs
        # self.model.max_det = 10  # maximum number of detections per image
        self.model.amp = False  # Automatic Mixed Precision (AMP) inference
        self.font = cv2.FONT_HERSHEY_SIMPLEX
        self.currrent_frame = None
        self.stopped = False
        self.nearestBallX = 9999
    def start(self):
        Thread(target=self.capture, args=()).start()
        return self

    def stop(self):
        self.stopped = True
    def capture(self):
        while not self.stopped:
            # grab the current frame
            start = time.time()
            frame = self.vs.read()
            width,height,_ = frame.shape
            results = self.model(frame)
            results = results.pandas()
            results = results.xyxy
            results = results[0].values
            tempNearestBall = self.nearestBallX
            for result in results:
                x1,y1,x2,y2,confidence,index,class_name = result
                x1 = int(x1)
                x2 = int(x2)
                y1 = int(y1)
                y2 = int(y2)
                middle_point_x = int((x1+x2)/2)
                middle_point_y = int((y1+y2)/2)
                middle_point = (middle_point_x,middle_point_y)

                if (tempNearestBall > middle_point_x):
                    tempNearestBall = middle_point_x
                frame = cv2.rectangle(frame,(x1,y1),(x2,y2),(0,0,255),1)

                frame = cv2.putText(frame,class_name + str(round(confidence,2)),(x1,y1-10),self.font,1,(255,255,255),1)
            self.nearestBallX = tempNearestBall
            self.currrent_frame = frame
            self.currrent_frame = frame

            cv2.imshow("Frame", frame)
            end = time.time()
            print("inference time:", end -start)
            key = cv2.waitKey(50) & 0xFF
            if key == ord("q"):
                break
        self.vs.stop()
        cv2.destroyAllWindows()


    def singleShot(self, img):
        start = time.time()
        self.current_frame = cv2.imread(img)
        result = self.model(img)
        end = time.time()
        print("inference time: ", end-start)
        result.show()

        results = result.pandas()
        results = results.xyxy
        results = results[0].values
        for result in results:
            x1, y1, x2, y2, confidence, index, class_name = result
            x1 = int(x1)
            x2 = int(x2)
            y1 = int(y1)
            y2 = int(y2)
            middle_point_x = int((x1 + x2) / 2)
            middle_point_y = int((y1 + y2) / 2)
            middle_point = (middle_point_x, middle_point_y)
            if (self.determineBallType(middle_point,x2-middle_point_x) == 1):
                type = 'tennis'
            else:
                type = 'ping pong'
            print("ball type: ",type,' at ', middle_point_x )


    def determineBallType(self,middle_point,r):
        avg_colour = np.array([0,0,0])
        half_radius = int(r/2)
        hsv_frame = cv2.cvtColor(self.current_frame,cv2.COLOR_BGR2HSV)


        for i in range(middle_point[0] - half_radius, middle_point[0] + half_radius):
            for j in range(middle_point[1] - half_radius, middle_point[1] + half_radius):
                avg_colour+=hsv_frame[j][i]


        total = 4*half_radius*half_radius
        avg_colour[0]=avg_colour[0]*(360/180)
        avg_colour= (avg_colour)/(total)
        print("avg color: ",avg_colour)
        if(
                (avg_colour[0] > 69 and avg_colour[0] < 162) and
                avg_colour[1] > 36
            ):
            return 1
        else:
            return 0