import asyncio
# pip install -qr https://raw.githubusercontent.com/ultralytics/yolov5/master/requirements.txt
import warnings

from ballDetection2 import ballDetection2

warnings.filterwarnings("ignore")


async def getNearestBall(speed=1):
    # first argument is either False or an array of integers corresponding to coco classes
    # 37 is for sports balls :)
    bd = ballDetection2(False, 0)
    bd.start()
    motor_input = 0
    y = 9999
    x = None
    while (1):
        detections = None
        if (bd.has_detection):
            detections = bd.last_detection
            # type = 0 - ping pong ,type = 1 - tennis
            for detection in detections:
                interim = detection[0]
                if (detection[1] < y):
                    y = detection[1]
                    x = detection[0]
            x = (x + 0.5) * 20
            print(detections)
        else:
            print("no Objects detected")
        await asyncio.sleep(speed)


asyncio.run(getNearestBall())

# bd.singleShot('test.jpg')
# bd.singleShot('test2.jpg')
# bd.singleShot('test3.jpg')
# bd.singleShot('test5.jpg')
# bd.singleShot('test7.jpg')
