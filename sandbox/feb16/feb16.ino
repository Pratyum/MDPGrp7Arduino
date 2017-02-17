#include "DualVNH5019MotorShield.h"
#include "PinChangeInt.h"

DualVNH5019MotorShield md;

#define pinEncoderL 3
#define pinEncoderR 5

unsigned long duration1, duration2;
volatile long encoderCountLeft = 0, encoderCountRight = 0;

double targetTick = 0, lastTick = 0; 
double error = 0, integral = 0;

double error1 = 0, error2 = 0, error3 = 0;

int pid = 0;


void setup(){
  Serial.begin(9600);
  
  md.init();
  pinMode(pinEncoderL, INPUT);
  pinMode(pinEncoderR, INPUT);
  
  PCintPort::attachInterrupt(pinEncoderL, incLeft, RISING);
  PCintPort::attachInterrupt(pinEncoderR, incRight, RISING);
  
  //md.setSpeeds(100, 100);
}

void loop(){
  //delay(3000);
  Serial.println(String(encoderCountLeft) + ", " + String(encoderCountRight));
  moveForward(10000);
  //rotateRight(360);
  Serial.println(String(encoderCountLeft) + ", " + String(encoderCountRight));
  delay(100000);
}

void incLeft() {
  encoderCountLeft++;
}

void incRight() {
  encoderCountRight++;
}

double tuneWithPID() {
  Serial.println(String(encoderCountLeft) + ", " + String(encoderCountRight) + ", " + String(encoderCountLeft - encoderCountRight));
  double k1, k2, k3, p, i, d;

  k1=23.59864429;
  k2=-37.52924142;
  k3=14.92081435;

  error3 = error2;
  error2 = error1;
  error1 = encoderCountLeft - encoderCountRight;

  p = k1 * error1;
  i = k2 * error2;
  d = k3 * error3;
  
  pid = p + i + d;
  
  return pid;
}

void moveForward(double cmDis) {
  pid = 0;
  encoderCountLeft = 0, encoderCountRight = 0;   
  error1 = 0, error2 = 0, error3 = 0, integral = 0, lastTick = 0;

  targetTick = cmDis * 50.5;
  
  while (encoderCountLeft < targetTick ){
    pid = tuneWithPID();
    md.setSpeeds(200 - pid, 200 + pid);
  }
  
   md.setBrakes(400, 400);
}

int rotateRight(double angle) {
  pid = 0;
  encoderCountRight = 0, encoderCountLeft = 0; 
  error = 0, integral = 0;

  if (angle <= 90) targetTick = angle * 4.47;
  else if (angle <=180 ) targetTick = angle * 4.62;
  else if (angle <=360 ) targetTick = angle * 4.675;
  else targetTick = angle * 4.65;

  while (encoderCountLeft < targetTick ) {
    pid = tuneWithPID();
    md.setSpeeds(200 - pid, -(200 + pid));
  }
  md.setBrakes(400,400);
}

