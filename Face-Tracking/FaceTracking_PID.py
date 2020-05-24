import time
import cv2
import serial
import asyncio
import threading

class PID:
    def __init__(self, kP=0.1, kI=0.0, kD=0.0):
        # initialize gains
        self.kP = kP
        self.kI = kI
        self.kD = kD

    def initialize(self):
        # intialize the current and previous time
        # self.currTime = time.time()
        # self.prevTime = self.currTime

        # initialize the previous error
        self.prevError = 0

        # initialize the term result variables
        self.cP = 0
        self.cI = 0
        self.cD = 0

    def update(self, error,deltaTime):
        # grab the current time and calculate delta time
        self.currTime = time.time()

        # delta error
        deltaError = error - self.prevError

        # proportional term
        self.cP = error

        # integral term
        self.cI += error * deltaTime

        # derivative term and prevent divide by zero
        self.cD = (deltaError / deltaTime) if deltaTime > 0 else 0
        # # save previous time and error for the next update
        self.prevError = error

        # sum the terms and return
        return sum([
            self.kP * self.cP,
            self.kI * self.cI,
            self.kD * self.cD])


def conter():
    if cap.isOpened():
        res, img = cap.read()
        (h,w) = img.shape[:2]
        return h//2,w//2

def inrange(num,min=0,max=180):
    return (num > min and num < max)

async def face_capture():
    global center_x #人脸中心的坐标
    global center_y
    global deltatime
    global Scan
    global flag
    while cap.isOpened():
        pretime = time.time()
        res, img = cap.read()
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        Scan = face.detectMultiScale(gray,scaleFactor=1.3,minNeighbors=3)
        # print(type(Scan))
        for (x, y, w, h) in Scan:
            cv2.rectangle(img, (x, y), (x + w, y + h), (255, 0, 0), 2)  # 依靠对角线来定矩形
            center_x = int(x + w/2)
            center_y = int(y + h / 2)

        cv2.imshow("CaptureFace",img)
        deltatime = time.time()-pretime
        # print(deltatime)
        await  asyncio.sleep(0.01)
        key = cv2.waitKey(50) & 0xff
        if key == 27:  # ESC的ASCALL码是27
            flag = 1
            break
    cap.release()
    cv2.destroyAllWindows()

async def caculatePID():

    print("Start")
    global pan_angle
    global tilt_angle
    p1,i1,d1 = 0.020,0.055,0.00031
    # p1,i1,d1 = 0,0,0
    # p2,i2,d2 = 0,0,0
    p2,i2,d2 = 0.022,0.0021,0.00038
    pid_pan = PID(p1, i1, d1)
    pid_tilt = PID(p2, i2, d2)
    while flag == 0:
        pid_pan.initialize()
        pid_tilt.initialize()
        if isinstance(Scan, tuple) == False:
            head_pan = pid_pan.update(cx-center_x,deltaTime=deltatime)
            head_tilt = pid_tilt.update(cy-center_y,deltaTime=deltatime)
        else:
            head_pan = head_tilt = 0
        if inrange(pan_angle + head_pan):
            pan_angle = int(pan_angle + head_pan)
        if inrange(tilt_angle-head_tilt):
            tilt_angle = int(tilt_angle-head_tilt)
        string = "X"+str(pan_angle)+"Y"+str(tilt_angle)
        print(string)
        ser.write(str(string).encode())
        await  asyncio.sleep(deltatime)
    else:pass


async def main():
    await asyncio.gather(
        face_capture(),
        caculatePID(),
    )

if __name__ == '__main__':
    serialPort = 'COM5'
    baudRate = 9600
    ser = serial.Serial(serialPort, baudRate)
    center_x = cener_y = 0
    # head_pan = 0
    deltatime = 0
    pan_angle = 20
    tilt_angle = 120
    Scan=[]
    flag = 0
    cap = cv2.VideoCapture(1,cv2.CAP_DSHOW)
    face = cv2.CascadeClassifier(r'haarcascade_frontalface_default.xml')
    cy,cx = conter()
    print(cx,cy)
    asyncio.run(main())