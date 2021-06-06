#include <PID_v1.h> // Import thư viện PID của Arduino
                    // download tại https://github.com/br3ttb/Arduino-PID-Library

uint8_t speed_robot=240;  // set (tốc độ của robot) speed_robot (pwm value), 0 < speed_robot < 256
int8_t check_out=0;

/// define sensor pinout
#define line_1      A0 // trái (hoặc ngược lại)
#define line_2      A1
#define line_3      A2 // giữa
#define line_4      A3 
#define line_5      A4 // phải
const int InA = 8;  // Động cơ trái
const int InB = 9;
const int InC = 10; // Động cơ phải
const int InD = 11;

//
double Setpoint=0, Input, Output;
uint8_t flag_zero=0;

//double Kp=15, Ki=0.045, Kd=12;  
double Kp=18.5, Ki=0.04, Kd=19.5; 

PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);
int sensor;

void setup()
{
  pinMode(InA, OUTPUT);
  pinMode(InB, OUTPUT);
  pinMode(InC, OUTPUT);
  pinMode(InD, OUTPUT);
  pinMode(6,1); // chân băm xung cho động cơ EnA
  pinMode(5,1); // chân băm xung cho động cơ EnB
  pinMode(line_1,INPUT);
  pinMode(line_2,INPUT);
  pinMode(line_3,INPUT);
  pinMode(line_4,INPUT);
  pinMode(line_5,INPUT);
  digitalWrite(InA, LOW);  // Bánh trái đi lên
  digitalWrite(InB, HIGH);
  digitalWrite(InC, LOW);  // Bánh phải đi lên
  digitalWrite(InD, HIGH);
  Input =0; // mặc định line giữa =0, luôn luôn điều chỉnh xe để input=0
  
  myPID.SetSampleTime(1); // thời gian lấy mẫu phụ thuộc tốc độ xe, lấy mẫu càng nhanh càng tốt (ms)
  myPID.SetMode(AUTOMATIC); 
  myPID.SetOutputLimits(-speed_robot,speed_robot); // giá trị tốc độ, -speed tức bánh bên trái quay max, bên phải ngừng quay
  
}

// băm xung điều khiển tốc độ 2 bánh xe, từ đó ta điều khiển được hướng rẽ, tốc dộ..... thông qua 1 biến duy nhất
void motorControl(int16_t duty_value)
{
  int16_t speed_a, speed_b;
  int speed_zero;
  speed_zero = speed_robot/2;
  if(duty_value>1) 
  {
    speed_b=-speed_zero;
    speed_a=duty_value;
  }
  else if(duty_value==0) 
  {
    speed_a=speed_b=0;
    
  }
  else if(duty_value<-1) 
  {
    speed_a=-speed_zero;
    speed_b=-duty_value;
  } 
      analogWrite(5,speed_b+speed_zero); //trai EnB
      analogWrite(6,speed_a+speed_zero); //phai EnA
}     

//Đèn tắt là 0v, đèn sáng (vào line đen) là 5V, từ 5 cảm biến ta có 9 vị trí 4 3 2 1 0 -1 -2 -3 -4
 void scan_sensor()
{
  if(digitalRead(line_5)==0) sensor=-4;
  else if((digitalRead(line_4)==0)&&(digitalRead(line_5)==0)) sensor=-3;
  else if(digitalRead(line_4)==0) sensor=-2;
  else if((digitalRead(line_3)==0)&&(digitalRead(line_4)==0)) sensor=-1;
  else if(digitalRead(line_3)==0) sensor=0; // vào line giữa
  else if((digitalRead(line_2)==0)&&(digitalRead(line_3)==0)) sensor=1;
  else if(digitalRead(line_2)==0) sensor=2;
  else if((digitalRead(line_1)==0)&&(digitalRead(line_2)==0)) sensor=3;
  else if(digitalRead(line_1)==0) sensor=4;
  
}


void loop() {
  Setpoint=0;
  scan_sensor(); // đọc dữ liệu từ cảm biến
  Input = sensor;
  myPID.Compute(); // tính toán
  motorControl(Output); // điều khiển bánh xe để robot đi đúng đường
 
}
