#include <Servo.h> 
 
Servo pan;
Servo tilt;

char inByte = 0; //串口接收的数据
int x=0;
int y=0;
String temp = "";
 
void setup() 
{
  pan.attach(9);    //定义舵机的引脚为9，舵机只能是10，或者9引脚
  tilt.attach(10);
  Serial.begin(9600);  //设置波特率
}
 

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
