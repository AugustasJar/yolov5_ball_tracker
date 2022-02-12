import asyncio

from ballDetection import ballDetection
# pip install -qr https://raw.githubusercontent.com/ultralytics/yolov5/master/requirements.txt
async def getNearestBall():
    bd = ballDetection(False, 0)
    bd.start()
    while(1):
        location = None
        location = bd.nearestBallX
        print(location)
        await asyncio.sleep(1)

asyncio.run(getNearestBall())


# bd.singleShot('test.jpg')
# bd.singleShot('test2.jpg')
# bd.singleShot('test3.jpg')
# bd.singleShot('test5.jpg')
# bd.singleShot('test7.jpg')
