import cv2
import numpy as np
import torch
from imutils import resize
from imutils.video import VideoStream
import time
from threading import Thread

class ballDetection2():
    def __init__(self,classes,camera):

        #classes parameter takes an array of integers representing coco classes
        self.classes = classes
        self.model = torch.hub.load('ultralytics/yolov5', 'yolov5n', pretrained=True)
        self.vs = VideoStream(camera)
        self.vs.start()

        # model parameters
        self.model.conf = 0.25  # NMS confidence threshold
        self.model.iou = 0.45  # NMS IoU threshold
        self.model.agnostic = False  # NMS class-agnostic
        self.model.multi_label = False  # NMS multiple labels per box
        self.model.classes = classes  # (optional list) filter by class, i.e. = [0, 15, 16] for COCO persons, cats and dogs
        # self.model.max_det = 10  # maximum number of detections per image
        self.model.amp = False  # Automatic Mixed Precision (AMP) inference

        self.font = cv2.FONT_HERSHEY_SIMPLEX
        self.stopped = False
        self.current_frame = None
        self.image_width = 640
        self.has_detection = False
        self.last_detection = None
        self.last_detection_type = np.array([])

    def start(self):
        Thread(target=self._capture, args=()).start()
        return self

    def stop(self):
        self.stopped = True

    def _capture(self):
        while not self.stopped:
            start = time.time()
            self.current_frame = self.vs.read()
            image = resize(self.current_frame,width = self.image_width)
            width,height,_ = image.shape
            results = self.model(image)
            results = results.pandas()
            results = results.xyxy
            results = results[0].values

            current_last_detection = []
            if results is not None:
                for result in results:
                    x1, y1, x2, y2, confidence, index, class_name = result
                    x1 = int(x1)
                    x2 = int(x2)
                    y1 = int(y1)
                    y2 = int(y2)

                    middle_point_x = int((x1 + x2) / 2)
                    middle_point_y = int((y1 + y2) / 2)
                    middle_point = (middle_point_x, middle_point_y)
                    middle_point_normalized = (middle_point_x/width,middle_point_y/height)
                    type = self.determineBallType(middle_point, x2 - middle_point_x)
                    current_last_detection.append(middle_point_normalized)

                    image = cv2.rectangle(image,(x1,y1),(x2,y2),(0,0,255),1)
                    image = cv2.putText(image,class_name + str(round(confidence,2)),(x1,y1-10),self.font,1,(255,255,255),1)

                if (current_last_detection):
                    self.has_detection = True
                    self.last_detection = current_last_detection
                else:
                    self.has_detection = False
            end = time.time()
            print("excecution time: ", end-start)
            cv2.imshow("Frame", image)
            key = cv2.waitKey(10) & 0xFF
            if key == ord("q"):
                self.stopped = True
                self.vs.stop()


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
        if(
                (avg_colour[0] > 69 and avg_colour[0] < 162) and
                avg_colour[1] > 36
            ):
            return 1
        else:
            return 0
