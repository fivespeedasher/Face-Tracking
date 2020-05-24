# Face-Tracking
Face-tracking on Arduino by Python
# Face-tracking on Arduino by Python


## 硬件选择

***

1. 两个20G舵机组成的而自由度云台

2. Arduino开发板

3. 7.2V锂电池

4. 12V降压升压模块

## Arduino平台

***

由于使用串口传输（单字节传入）加上板子处理能力有限，Arduino端仅仅是实现驱动舵机转动，其他的计算都在Python端上完成。
在定义变量、初始化之后，就可以等待传入了

``` c
void loop()
{
  if (Serial.available() > 0)
  {
    if(Serial.read() == 'X')
    {
      x = Serial.parseInt();
      pan.write(x);
      Serial.println(x);
      if(Serial.read() == 'Y')
      {
        y = Serial.parseInt();
        tilt.write(y);
        Serial.println(y);
      }
    }
    while(Serial.available() > 0)
    {
      Serial.read();
    }
  }
}
```

## Python

***

* 双线程

  * 一条线程是占用摄像头来获得Δt和人脸与图像中心的偏差

    ``` python
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
        ```

  * 另一条是占用串口，把计算好的角度传入，来控制舵机

    ``` python
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
    ```

  * 一开始觉得双线程会快很多，但其实它跟函数调用一样的，都是先入先出，不过这样写起来的好处是：避免了大片的程序。美观而且方便之后的调整

* PID

  * 不要去纠结怎么把偏差对应成角度，交给PID去处理就好啦。

    ``` python
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
    ```

  * 先调整Kp只要偏差大时舵机可以勉强跟上就行，不要求速度很快，但不能跟过头。

  * 调Kd，因为积分环节是误差积分累加*kd，所以需要等一会，让积分项加上去。在比例控制期间，需要做的只是观察什么时候可以稳下来，选择自己能接受的调节时间，可以允许跟超。

  * 最后调Ki，微分环节是对误差的预测，对于变化剧烈的情况下，相当于给了比例环节一个补充，调到朝着一个方向运动时不会跟跑即可。
