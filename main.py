import asyncio

from ballDetection2 import ballDetection2
# pip install -qr https://raw.githubusercontent.com/ultralytics/yolov5/master/requirements.txt
import warnings

warnings.filterwarnings("ignore")

async def getNearestBall(speed = 1):
    # first argument is either False or an array of integers corresponding to coco classes
    bd = ballDetection2([37], 0)
    bd.start()
    while(1):
        detections = None
        if (bd.has_detection):
            detections = bd.last_detection
            # type = 0 - ping pong ,type = 1 - tennis
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
