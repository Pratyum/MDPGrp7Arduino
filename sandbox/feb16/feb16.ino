#include "DualVNH5019MotorShield.h"
#include "PinChangeInt.h"

DualVNH5019MotorShield md;

#define pinEncoderL 3
#define pinEncoderR 5

unsigned long duration1, duration2;
volatile long encoderCountLeft = 0, encoderCountRight = 0;

double target_Tick = 0, last_Tick = 0; 
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
//  Serial.println(String(encoderCountLeft) + ", " + String(encoderCountRight));
  moveForward(120);
  //rotateRight(360);
//  Serial.println(String(encoderCountLeft) + ", " + String(encoderCountRight));
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

  k1=15;
  k2=0;
  k3=0;

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
  error = 0, integral = 0, last_Tick = 0;

  target_Tick = cmDis * 30.25; // Caliberated to 30.25 ticks per cm

  while (encoderCountLeft < target_Tick ) {
    pid = tuneWithPID();
    if(!checkForObstcle()){  
      md.setSpeeds(200 - pid, 200 + pid);
    }
  }

  md.setBrakes(400, 400);
}


void moveBackward(double cmDis) {
  pid = 0;
  encoderCountLeft = 0, encoderCountRight = 0;
  error = 0, integral = 0, last_Tick = 0;

  target_Tick = cmDis * 30.25; // Caliberated to 30.25 ticks per cm

  while (encoderCountLeft < target_Tick ) {
    pid = tuneWithPID();
    md.setSpeeds(-(200 - pid), -(200 + pid));
  }

  md.setBrakes(400, 400);
}

int rotateRight(double angle) {
  pid = 0;
  encoderCountRight = 0, encoderCountLeft = 0;
  error = 0, integral = 0;

  if (angle <= 90) target_Tick = angle * 4.47;
  else if (angle <= 180 ) target_Tick = angle * 4.62;
  else if (angle <= 360 ) target_Tick = angle * 4.675;
  else target_Tick = angle * 4.65;

  while (encoderCountLeft < target_Tick ) {
    pid = tuneWithPID();
    md.setSpeeds(200 - pid, -(200 + pid));
  }
  md.setBrakes(400, 400);
}

int rotateLeft(double angle) {
  pid = 0;
  encoderCountRight = 0, encoderCountLeft = 0;
  error = 0, integral = 0;

  if (angle <= 90) target_Tick = angle * 4.41;
  else if (angle <= 180 ) target_Tick = angle * 4.51;
  else if (angle <= 360 ) target_Tick = angle * 4.51;
  else target_Tick = angle * 4.65;

  while (encoderCountLeft < target_Tick ) {
    pid = tuneWithPID();
    md.setSpeeds(-(200 - pid), (200 + pid));
  }
  md.setBrakes(400, 400);
}

