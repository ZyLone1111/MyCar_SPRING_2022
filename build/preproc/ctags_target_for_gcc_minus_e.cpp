# 1 "c:\\Program Files\\arduino-1.8.19\\MYCAR\\mecanum_move\\mecanum_move.ino"
# 2 "c:\\Program Files\\arduino-1.8.19\\MYCAR\\mecanum_move\\mecanum_move.ino" 2
# 3 "c:\\Program Files\\arduino-1.8.19\\MYCAR\\mecanum_move\\mecanum_move.ino" 2
# 4 "c:\\Program Files\\arduino-1.8.19\\MYCAR\\mecanum_move\\mecanum_move.ino" 2
# 5 "c:\\Program Files\\arduino-1.8.19\\MYCAR\\mecanum_move\\mecanum_move.ino" 2

static int startPWM = 100;

volatile double pos;
volatile double rpm;
volatile int PPR;
volatile int gearratio;
volatile int CPR;

bool SLL, SL, SM, SR, SRR;
int detectNum;
static enum { MID,
              LEFT1,
              RIGHT1,
              LEFT2,
              RIGHT2,
              ALL,
              UNDETECTED } movingStatus,
    last_movingStatus;
char *Tmove[] = {"MID", "LEFT1", "RIGHT1", "LEFT2", "RIGHT2", "ALL", "UNDETECTED"};

double kp, ki, kd;
int pwmRestrict = 100;

QGPMaker_MotorShield AFMS = QGPMaker_MotorShield();

// IICSensorBar I2C address :

const byte IIC_ADDRESS = 0x3F;
SensorBar io;
// Pin definition:
const byte INPUT_PIN_S0 = 0;
const byte INPUT_PIN_S1 = 1;
const byte INPUT_PIN_S2 = 2;
const byte INPUT_PIN_S3 = 3;
const byte INPUT_PIN_S4 = 4;

// Moter definition:
QGPMaker_DCMotor *DCMotor_2 = AFMS.getMotor(2); //左后
QGPMaker_DCMotor *DCMotor_4 = AFMS.getMotor(4); //右前
QGPMaker_DCMotor *DCMotor_1 = AFMS.getMotor(1); //左前
QGPMaker_DCMotor *DCMotor_3 = AFMS.getMotor(3); //右后

QGPMaker_Servo *Servo1 = AFMS.getServo(1); //获取1号舵机
QGPMaker_Servo *Servo2 = AFMS.getServo(2); //获取2号舵机
QGPMaker_Servo *Servo3 = AFMS.getServo(3); //获取3号舵机
QGPMaker_Servo *Servo4 = AFMS.getServo(4); //获取4号舵机

QGPMaker_Encoder Encoder1(1);
QGPMaker_Encoder Encoder2(2);
QGPMaker_Encoder Encoder3(3);
QGPMaker_Encoder Encoder4(4);

/******************************************

               Motors

******************************************/
# 61 "c:\\Program Files\\arduino-1.8.19\\MYCAR\\mecanum_move\\mecanum_move.ino"
void forward(int startPWM)
{
  DCMotor_1->setSpeed(startPWM);
  DCMotor_1->run(1);
  DCMotor_2->setSpeed(startPWM);
  DCMotor_2->run(1);
  DCMotor_3->setSpeed(startPWM);
  DCMotor_3->run(1);
  DCMotor_4->setSpeed(startPWM);
  DCMotor_4->run(1);
}
void turnLeft(int startPWM = 200)
{
  DCMotor_1->setSpeed(startPWM*1.1);
  DCMotor_1->run(3);
  DCMotor_2->setSpeed(startPWM);
  DCMotor_2->run(3);
  DCMotor_3->setSpeed(startPWM*0.7);
  DCMotor_3->run(1);
  DCMotor_4->setSpeed(startPWM*1.3);
  DCMotor_4->run(1);
}
void turnRight(int startPWM = 200)
{
  DCMotor_1->setSpeed(startPWM);
  DCMotor_1->run(1);
  DCMotor_2->setSpeed(startPWM);
  DCMotor_2->run(1);
  DCMotor_3->setSpeed(startPWM);
  DCMotor_3->run(3);
  DCMotor_4->setSpeed(startPWM);
  DCMotor_4->run(3);
}
void moveLeft(int startPWM = 200)
{
  DCMotor_1->setSpeed(startPWM);
  DCMotor_1->run(2);
  DCMotor_2->setSpeed(startPWM);
  DCMotor_2->run(1);
  DCMotor_3->setSpeed(startPWM);
  DCMotor_3->run(2);
  DCMotor_4->setSpeed(startPWM);
  DCMotor_4->run(1);
}
void moveRight(int startPWM = 200)
{
  DCMotor_1->setSpeed(200);
  DCMotor_1->run(1);
  DCMotor_2->setSpeed(200);
  DCMotor_2->run(2);
  DCMotor_3->setSpeed(200);
  DCMotor_3->run(1);
  DCMotor_4->setSpeed(200);
  DCMotor_4->run(2);
}
void backward(int startPWM = 200)
{
  DCMotor_1->setSpeed(200);
  DCMotor_1->run(2);
  DCMotor_2->setSpeed(200);
  DCMotor_2->run(2);
  DCMotor_3->setSpeed(200);
  DCMotor_3->run(2);
  DCMotor_4->setSpeed(200);
  DCMotor_4->run(2);
}
void stopMoving()
{
  DCMotor_1->setSpeed(0);
  DCMotor_1->run(3);
  DCMotor_2->setSpeed(0);
  DCMotor_2->run(3);
  DCMotor_3->setSpeed(0);
  DCMotor_3->run(3);
  DCMotor_4->setSpeed(0);
  DCMotor_4->run(3);
}
void straightForward()
{
  DCMotor_1->setSpeed(220);
  DCMotor_1->run(1);
  DCMotor_2->setSpeed(190);
  DCMotor_2->run(1);
  DCMotor_3->setSpeed(170);
  DCMotor_3->run(1);
  DCMotor_4->setSpeed(200);
  DCMotor_4->run(1);
}

void show_rpm()
{
  Serial.print(Encoder1.getRPM());
  Serial.print('>');
  Serial.print(Encoder2.getRPM());
  Serial.print('>');
  Serial.print(Encoder3.getRPM());
  Serial.print('>');
  Serial.println(Encoder4.getRPM());
  delay(50);
}

void show_signal()
{
  Serial.print(Encoder1.read());
  Serial.print('>');
  Serial.print(Encoder2.read());
  Serial.print('>');
  Serial.print(Encoder3.read());
  Serial.print('>');
  Serial.println(Encoder4.read());
  delay(50);
}
/******************************************

              PID algrithm，但是不想用

******************************************/
# 176 "c:\\Program Files\\arduino-1.8.19\\MYCAR\\mecanum_move\\mecanum_move.ino"
static double bias, PWM, last_bias, last_bias2;
int calPID1(int velocity, int target)
{

  bias = target - velocity;
  Serial.print(bias);
  Serial.print(",");
  Serial.print(last_bias);
  Serial.print(",");
  PWM += kp * (bias - last_bias) + ki * bias + kd * (bias - 2 * last_bias + last_bias2);

  // Serial.print(PWM);
  // Serial.print(",");
  //引入上限
  if (PWM > pwmRestrict)
  {
    PWM = pwmRestrict;
  }
  if (PWM < -pwmRestrict)
  {
    PWM = -pwmRestrict;
  }
  last_bias2 = last_bias2;
  last_bias = bias;
  Serial.print(PWM);
  Serial.print(",");
  return PWM;
}
void pid_control1(int targetVolocity)
{
  int temp1 = Encoder1.getRPM();
  int pidOutput1 = -calPID1(temp1, targetVolocity);
  if (pidOutput1 > 0)
  {
    DCMotor_1->setSpeed(pidOutput1 + startPWM);
    DCMotor_1->run(1);
  }
  else if (pidOutput1 == 0)
  {
    DCMotor_1->setSpeed(pidOutput1 + startPWM);
    DCMotor_1->run(4);
  }
  else
  {
    DCMotor_1->setSpeed(pidOutput1 + startPWM);
    DCMotor_1->run(2);
  }
  Serial.print(pidOutput1 + startPWM);
  Serial.print(",");
  Serial.println(temp1);
}
void pid_control2(int targetVolocity)
{
  int temp2 = Encoder2.getRPM();
  int pidOutput2 = calPID1(temp2, targetVolocity);
  if (pidOutput2 > 0)
  {
    DCMotor_2->setSpeed(pidOutput2 + startPWM);
    DCMotor_1->run(2);
  }
  else if (pidOutput2 == 0)
  {
    DCMotor_2->setSpeed(pidOutput2 + startPWM);
    DCMotor_2->run(4);
  }
  else
  {
    DCMotor_2->setSpeed(pidOutput2 + startPWM);
    DCMotor_2->run(1);
  }
}
void pid_control3(int targetVolocity)
{
  int temp3 = Encoder3.getRPM();
  int pidOutput3 = calPID1(temp3, targetVolocity);
  if (pidOutput3 > 0)
  {
    DCMotor_3->setSpeed(pidOutput3 + startPWM);
    DCMotor_3->run(2);
  }
  else if (pidOutput3 == 0)
  {
    DCMotor_3->setSpeed(pidOutput3 + startPWM);
    DCMotor_3->run(4);
  }
  else
  {
    DCMotor_3->setSpeed(pidOutput3 + startPWM);
    DCMotor_3->run(1);
  }
}
void pid_control4(int targetVolocity)
{
  int temp4 = Encoder4.getRPM();
  int pidOutput4 = calPID1(temp4, targetVolocity);
  if (pidOutput4 > 0)
  {
    DCMotor_4->setSpeed(pidOutput4 + startPWM);
    DCMotor_4->run(2);
  }
  else if (pidOutput4 == 0)
  {
    DCMotor_4->setSpeed(pidOutput4 + startPWM);
    DCMotor_4->run(4);
  }
  else
  {
    DCMotor_4->setSpeed(pidOutput4 + startPWM);
    DCMotor_4->run(1);
  }
}
void pid_control_all(int targetVolocity1, int targetVolocity2, int targetVolocity3, int targetVolocity4)
{
  pid_control1(targetVolocity1);
  pid_control2(targetVolocity2);
  pid_control3(targetVolocity3);
  pid_control4(targetVolocity4);
}

/******************************************

              Servos

******************************************/
# 299 "c:\\Program Files\\arduino-1.8.19\\MYCAR\\mecanum_move\\mecanum_move.ino"
void init_pos()
{
  Servo1->writeServo(45);
  Servo2->writeServo(45);
  Servo3->writeServo(90);
  Servo4->writeServo(0);
}

void fetch()
{
  // Motion_1
  Servo1->writeServo(90);
  Servo2->writeServo(90);
  delay(1000);
  // Motion_2
  Servo1->writeServo(150);
  Servo3->writeServo(150);
  delay(1000);
  // Motion_3
  Servo4->writeServo(60);
  delay(1000);
  // Motion_4
  Servo1->writeServo(45);
  Servo2->writeServo(45);
  Servo3->writeServo(90);
}

void place()
{
  Servo1->writeServo(150);
  Servo2->writeServo(90);
  Servo3->writeServo(150);
  delay(1000);
  Servo4->writeServo(0);
}

void readPos()
{
  Serial.print("Position of S1,S2,S3,S4 is: ");
  Serial.print(Servo1->readDegrees());
  Serial.print(">");
  Serial.print(Servo2->readDegrees());
  Serial.print(">");
  Serial.print(Servo3->readDegrees());
  Serial.print(">");
  Serial.println(Servo4->readDegrees());
}

/******************************************

              Tasks

******************************************/
# 351 "c:\\Program Files\\arduino-1.8.19\\MYCAR\\mecanum_move\\mecanum_move.ino"
void getStatus()
{
  SRR = (io.digitalRead(INPUT_PIN_S0));
  SR = (io.digitalRead(INPUT_PIN_S1));
  SM = (io.digitalRead(INPUT_PIN_S2));
  SL = (io.digitalRead(INPUT_PIN_S3));
  SLL = (io.digitalRead(INPUT_PIN_S4));
  detectNum = ((SRR == 0) + (SR == 0) + (SM == 0) + (SL == 0) + (SLL == 0));

  if (SRR == 0x1 && SR == 0x1 && SM == 0x0 && SL == 0x1 && SLL == 0x1)
    movingStatus = MID;
  else if (SRR == 0x1 && SR == 0x0 && SM == 0x1 && SL == 0x1 && SLL == 0x1)
    movingStatus = RIGHT1;
  else if (SRR == 0x1 && SR == 0x1 && SM == 0x1 && SL == 0x0 && SLL == 0x1)
    movingStatus = LEFT1;
  else if (SRR == 0x0 && SR == 0x1 && SM == 0x1 && SL == 0x1 && SLL == 0x1)
    movingStatus = RIGHT2;
  else if (SRR == 0x1 && SR == 0x1 && SM == 0x1 && SL == 0x1 && SLL == 0x0)
    movingStatus = LEFT2;
  else if (SRR == 0x0 && SR == 0x0 && SM == 0x0 && SL == 0x0 && SLL == 0x0)
    movingStatus = ALL;
  else
    movingStatus = UNDETECTED;
  Serial.println(detectNum);
  Serial.println(Tmove[movingStatus]);
  Serial.println(Tmove[last_movingStatus]);
}

void lineFollow1()
{
  switch (movingStatus)
  {
  case MID:
    forward(80);
    break;
  case LEFT1:
    turnLeft(30);
    break;
  case RIGHT1:
    turnRight(30);
    break;
  case LEFT2:
    turnLeft(60);
    break;
  case RIGHT2:
    turnRight(60);
    break;
  default:
    forward(40);
    break;
  }
}

void lineFollow(int vForward1=40, int vForward2=40, int vLeft1=30, int vRight1=30, int vLeft2=60, int vRight2=60, int lapse=20)
{
  SRR = (io.digitalRead(INPUT_PIN_S0));
  SR = (io.digitalRead(INPUT_PIN_S1));
  SM = (io.digitalRead(INPUT_PIN_S2));
  SL = (io.digitalRead(INPUT_PIN_S3));
  SLL = (io.digitalRead(INPUT_PIN_S4));

  Serial.print("SO,S1,S2,S3,S4 status: ");
  // Read the pin to print either 0 or 1
  Serial.print(SLL);
  Serial.print(SL);
  Serial.print(SM);
  Serial.print(SR);
  Serial.println(SRR);

  if (SM == 0x0)
  {
    forward(vForward1);
    Serial.println("forward");
  }
  else if (SL == 0x0)
  {
    turnLeft(vLeft1);
    Serial.println("left1");
  }
  else if (SR == 0x0)
  {
    turnRight(vRight1);
    Serial.println("right1");
  }
  else if (SLL == 0x0)
  {
    turnLeft(vLeft2);
    Serial.println("left2");
  }
  else if (SRR == 0x0)
  {
    turnRight(vRight2);
    Serial.println("right2");
  }
  else
  {
    forward(vForward2);
  }
  delay(lapse);
}

//走过n条horizen黑线
void through_horizen(int n, void (*f)(int x), int pwm)
{
  Serial.println("Test!");
  int cnt = 0;
  int num;
  f(pwm);
  while (cnt < n)
  {
    getStatus();
    if (detectNum >= 3)
    {
      cnt++;
      Serial.print("cnt=");
      Serial.println(cnt);
      if (cnt != n)
        delay(500); //最后一次精准停在线上
    }
    // last_movingStatus=movingStatus;
  }
  stopMoving();
  delay(1000);
  return;
}
//
void set_middle(void (*f)(int x), int pwm, int delay_time)
{
  while (1)
  {
    getStatus();
    f(pwm);
    if (movingStatus == MID)
    {
      stopMoving();
      delay(delay_time);
      return;
    }
  }
}

//微调函数
void micro_movement(void (*func)(int x), int pwm, int lasting_time)
{
  func(pwm);
  delay(lasting_time);
  stopMoving();
}
// 90deg转弯
void turn_90_deg(void (*func)(int x), int pwm, int last_time)
{
  func(pwm);
  delay(last_time);
  return;
}

void set_horizon(){
  while(true)
  {
    getStatus();
    if(detectNum==5){
      stopMoving();
      return;
    }
    else{
      lineFollow();
    }
  }

}
/******************************************

              SetupConfig

******************************************/
# 525 "c:\\Program Files\\arduino-1.8.19\\MYCAR\\mecanum_move\\mecanum_move.ino"
void setup()
{
  AFMS.begin(50); // 50是啥我不清楚
  Serial.begin(9600);
  if (!io.begin(IIC_ADDRESS))
  {
    Serial.println("Failed to communicate.");
    while (1)
      ;
  }
  io.pinMode(INPUT_PIN_S0, 0x0);
  io.pinMode(INPUT_PIN_S1, 0x0);
  io.pinMode(INPUT_PIN_S2, 0x0);
  io.pinMode(INPUT_PIN_S3, 0x0);
  io.pinMode(INPUT_PIN_S4, 0x0);

  init_pos();
  delay(1000);

  pos = 0;
  rpm = 0;
  PPR = 12;
  gearratio = 90;
  CPR = (PPR * 4) * gearratio;
  kp = 1;
  ki = 0;
  kd = 0;
}

void loop()
{
  // moveLeft(40);
  // show_rpm();

  // set_horizon();
  // delay(5000);

  through_horizen(5, forward, 40);
  delay(1000);
  micro_movement(backward, 40, 500);
  delay(1000);
  set_middle(moveLeft, 40, 2000);
  delay(1000);
  set_horizon();
  delay(100000);


  // ATTENTION:以下流程建立在小车走直线(包括forward(),backward(),moveLeft,moveRight,etc)的基础上,所以需要调直线.

  // 难度系数1.5

  // 找到第五条线
  // void through_horizen(int n,void (*f)(int x),int pwm)

  // 找到右边第一条中线
  // void set_middle(void (*f)(int x), int pwm, int delay_time)
  // set_middle(moveLeft, 40, 2000);

  // through_horizen(4, forward, 40);

  // // 向后退一点，需要动态调整lasing_time与pwm使得小车与物块的距离使得机械爪舒适抓到物块
  // // void mirco_movement(void (*func)(int x), int pwm, int lasting_time)
  // micro_movement(backward, 40, 100);
  // delay(1000);

  // // 向右转90,需要精准,需要调整pwm与last_time
  // // void turn_90_deg(void (*func)(int x),int pwm,int last_time)
  // turn_90_deg(turnLeft, 40, 2000);

  // // 重新找到中线
  // set_middle(moveRight, 40, 2000);

  // // 精准找到中间
  // micro_movement(moveLeft, 40, 3000);

  // // 走直线到第一个投放点
  // through_horizen(4, forward, 40);

  // // 向后退一点，需要动态调整lasing_time与pwm使得小车与物块的距离使得机械爪舒适抓到物块
  // micro_movement(backward, 40, 100);

  return;
}
