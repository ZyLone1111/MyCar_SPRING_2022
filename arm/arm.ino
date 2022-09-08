#include <Wire.h>
#include "QGPMaker_MotorShield.h"

QGPMaker_MotorShield AFMS = QGPMaker_MotorShield(); //创建驱动器对象
QGPMaker_Servo *Servo1 = AFMS.getServo(1); //获取1号舵机
QGPMaker_Servo *Servo2 = AFMS.getServo(2); //获取2号舵机
QGPMaker_Servo *Servo3 = AFMS.getServo(3); //获取3号舵机
QGPMaker_Servo *Servo4 = AFMS.getServo(4); //获取4号舵机


void setup() {
    Serial.begin(9600);           // set up Serial library at 9600 bps
    Serial.println("DC Motor test!");

        AFMS.begin(50); 
}

void init_pos(){
    Servo1->writeServo(45);
    Servo2->writeServo(45);
    Servo3->writeServo(90);
    Servo4->writeServo(0); 
}

void fetch(){
    //Motion_1
    Servo1->writeServo(90);
    Servo2->writeServo(90);
    delay(1000);
    //Motion_2
    Servo1->writeServo(150);
    Servo3->writeServo(150);
    delay(1000);
    //Motion_3
    Servo4->writeServo(60);
    delay(1000);
    //Motion_4
    Servo1->writeServo(45);
    Servo2->writeServo(45);
    Servo3->writeServo(90);
}

void place(){
    Servo1->writeServo(150);
    Servo2->writeServo(90);
    Servo3->writeServo(150);
    delay(1000);
    Servo4->writeServo(0);
}

void readPos(){
    Serial.print("Position of S1,S2,S3,S4 is: ");
    Serial.print(Servo1->readDegrees());
    Serial.print(">");
    Serial.print(Servo2->readDegrees());
    Serial.print(">");
    Serial.print(Servo3->readDegrees());
    Serial.print(">");
    Serial.println(Servo4->readDegrees());
}

void test3(){
    int pos=0;
    while(pos<180){
        Servo3->writeServo(pos);
        pos++;
        readPos();
        delay(20);
    }
    while(pos>0){
        Servo3->writeServo(pos);
        pos--;
        readPos();
        delay(20);
    }
}

void loop() {
    init_pos();
    delay(5000);
    fetch();
    delay(5000);
    place();
    delay(500);
}
