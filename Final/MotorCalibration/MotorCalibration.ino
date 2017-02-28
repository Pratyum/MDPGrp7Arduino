#include "DualVNH5019MotorShield.h"
#include "PinChangeInt.h"

#define pinEncoderL 3
#define pinEncoderR 5

DualVNH5019MotorShield md;

volatile long encoderCountLeft, encoderCountRight;
long prevTick;
double integral;

void setup() {
  Serial.begin(9600);
  md.init();
  
  pinMode(pinEncoderL, INPUT);
  pinMode(pinEncoderR, INPUT);

  PCintPort::attachInterrupt(pinEncoderL, incLeft, RISING);
  PCintPort::attachInterrupt(pinEncoderR, incRight, RISING);
}

void loop() {
  
}

void incLeft() {
  encoderCountLeft++;
}

void incRight() {
  encoderCountRight++;
}

double computePID() {
  //Serial.println(String(encoderCountLeft) + ", " + String(encoderCountRight) + ", " + String(encoderCountLeft - encoderCountRight));
  double kp, ki, kd, p, i, d, error, pid;

  kp = 14.3; // trial and error
  ki = 0;
  kd = 0;

  error = encoderCountLeft - encoderCountRight;
  integral += error;

  p = kp * error;
  i = ki * integral;
  d = kd * (prevTick - encoderCountLeft);
  pid = p + i + d;

  prevTick = encoderCountLeft;

  return pid;
}

void forward(double cm) {
  double pid;
  int targetTick;
  integral = 0;
  encoderCountLeft = encoderCountRight = prevTick = 0;
  
  targetTick = cm * 29; // Caliberated to 30.25 ticks per cm

  while (encoderCountLeft < targetTick ) {
    pid = computePID();
    if (mode == 0) {
      md.setSpeeds(200 - pid, 200 + pid);
    }
    else if (mode == 1) {
      md.setSpeeds(100 - pid, 100 + pid);
    }
  }

  md.setBrakes(400, 400);
  delay(100);
}


void reverse(double cm) {
  double pid;
  int targetTick;
  integral = 0;
  encoderCountLeft = encoderCountRight = prevTick = 0;

  targetTick = cm * 29; // Caliberated to 30.25 ticks per cm

  while (encoderCountLeft < targetTick ) {
    pid = computePID();
    if (mode == 0) {
      md.setSpeeds(-(200 - pid), -(200 + pid));
    }
    else if (mode == 1) {
      md.setSpeeds(-(100 - pid), -(100 + pid));
    }
  }
  md.setBrakes(400, 400);
  delay(100);
}

void rotateRight(double deg) {
  double pid;
  int targetTick;
  integral = 0;
  encoderCountLeft = encoderCountRight = prevTick = 0;

  if (deg <= 90) targetTick = deg * 4.423;
  else if (deg <= 180 ) targetTick = deg * 4.62;
  else if (deg <= 360 ) targetTick = deg * 4.675;
  else targetTick = deg * 4.65;

  while (encoderCountLeft < targetTick ) {
    pid = computePID();
    if (mode == 0) {
      md.setSpeeds(200 - pid, -(200 + pid));
    }
    else if (mode == 1) {
      md.setSpeeds(100 - pid, -(100 + pid));
    }
    
  }
  md.setBrakes(400, 400);
  delay(100);
}

void rotateLeft(double deg) {
  double pid;
  int targetTick;
  integral = 0;
  encoderCountLeft = encoderCountRight = prevTick = 0;

  if (deg <= 90) targetTick = deg * 4.424;
  else if (deg <= 180 ) targetTick = deg * 4.51;
  else if (deg <= 360 ) targetTick = deg * 4.51;
  else targetTick = deg * 4.65;

  while (encoderCountLeft < targetTick ) {
    pid = computePID();
    if (mode == 0) {
      md.setSpeeds(-(200 - pid), (200 + pid));
    }
    else if (mode == 1) {
      md.setSpeeds(-(100 - pid), (100 + pid));
    }
  }
  md.setBrakes(400, 400);
  delay(100);
}


