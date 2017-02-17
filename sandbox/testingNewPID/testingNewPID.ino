#include "DualVNH5019MotorShield.h"
#include "PinChangeInt.h"
#include <PID_v1.h>


DualVNH5019MotorShield md;

#define pinEncoderL 3
#define pinEncoderR 5
#define pinSwitch 8

unsigned long duration1, duration2;
double encoderCountLeft = 0, encoderCountRight = 0;

double target_Tick = 0, last_tick = 0; 
double error = 0, integral = 0;
unsigned long lastTime=0;
double lastInput = 0;
double ITerm =0;

double pid = 0;
PID myPID(&encoderCountRight, &pid, &encoderCountLeft, 15, 0, 0, DIRECT);


void setup(){
  Serial.begin(9600);
  
  md.init();
  pinMode(pinEncoderL, INPUT);
  pinMode(pinEncoderR, INPUT);
  pinMode(pinSwitch, INPUT);
//  myPID.SetSampleTime(5);
  PCintPort::attachInterrupt(pinEncoderL, incLeft, RISING);
  PCintPort::attachInterrupt(pinEncoderR, incRight, RISING);
  
  //md.setSpeeds(100, 100);
}

void loop(){
  //delay(3000);
//  Serial.println(String(encoderCountLeft) + ", " + String(encoderCountRight));
  moveForward(120);
//  rotateRight(810);
//  duration1 = pulseIn(encoderPin1, HIGH);
//  duration2 = pulseIn(encoderPin2, HIGH);
//        if (Serial.available() > 0) {
//                // read the incoming byte:
//                int incomingByte = Serial.parseInt();
//
//                // say what you got:
//                Serial.print("I received: ");
//                Serial.println(incomingByte, DEC);
//
//                rotateRight(int(incomingByte));
//        }
//  Serial.println(String(encoderCountLeft) + ", " + String(encoderCountRight));
  delay(1000000);
}

void incLeft() {
  encoderCountLeft++;
}

void incRight() {
  encoderCountRight++;
}

double tuneWithPID() {
  Serial.println(String(encoderCountLeft) + ", " + String(encoderCountRight) + ", " + String(encoderCountLeft - encoderCountRight));
  double kp, ki, kd, p, i, d;

  kp=15; // trial and error 
  ki=0;
  kd=0;


  error = encoderCountLeft - encoderCountRight;
  integral += error;

  p = error * kp;
  i = integral * ki;
  d = (last_tick - encoderCountLeft) * kd;
  pid = p + i + d;

  last_tick = encoderCountLeft;
  
  return pid;
}

void moveForward(double cmDis) {
  pid = 0;
  encoderCountLeft = 0, encoderCountRight = 0;   
  error = 0, integral = 0, last_tick = 0;

  target_Tick = cmDis * 30.25; // Caliberated to 30.25 ticks per cm 
  
  while (encoderCountLeft < target_Tick ){
//    pid = tuneWithPID();
//   Serial.println(encoderCountLeft);
//   Serial.println(encoderCountRight);
//   if( myPID.Compute()){
//    Serial.println("PID WORKS");
//   }
//    pid = tuneWithPID2();
    pid = tuneWithPID();
    Serial.println(pid);
    md.setSpeeds(200 - pid, 200 + pid);
  }
  
   md.setBrakes(400, 400);
}

int rotateRight(double angle) {
  pid = 0;
  encoderCountRight = 0, encoderCountLeft = 0; 
  error = 0, integral = 0;

  if (angle <= 90) target_Tick = angle * 4.47;
  else if (angle <=180 ) target_Tick = angle * 4.62;
  else if (angle <=360 ) target_Tick = angle * 4.675;
  else target_Tick = angle * 4.65;

  while (encoderCountLeft < target_Tick ) {
    pid = tuneWithPID();
    md.setSpeeds(200 - pid, -(200 + pid));
  }
  md.setBrakes(400,400);
}


double tuneWithPID2(){
   double ki=0,kp=15,kd=0;
   unsigned long now = millis();
   unsigned long timeChange = (now - lastTime);
      /*Compute all the working error variables*/
      double input = encoderCountRight;
      double error = encoderCountLeft - input;
      ITerm+= (ki * error);
      double dInput = (input - lastInput);
      /*Remember some variables for next time*/
      lastInput = encoderCountRight;
      lastTime = now;
 
      /*Compute PID Output*/
      double output = kp * error + ITerm- kd * dInput;
      return output;
      
}

