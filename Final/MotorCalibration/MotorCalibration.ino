#include "DualVNH5019MotorShield.h"
#include "PinChangeInt.h"

#define pinEncoderL 3
#define pinEncoderR 5

DualVNH5019MotorShield md;

#define SPEED_MOVE 300
#define SPEED_SPIN 300
#define SPEED_CALIBRATE 100
#define MOTOR_MULTIPLIER 0.92

//29.70
/*
423
835
820
820
422
412
//
*/
volatile long encoderCountLeft, encoderCountRight;
long prevTick;
double integral;

int mode = 0;

void setup() {
  Serial.begin(9600);
  md.init();
  
  pinMode(pinEncoderL, INPUT);
  pinMode(pinEncoderR, INPUT);

  PCintPort::attachInterrupt(pinEncoderL, incLeft, RISING);
  PCintPort::attachInterrupt(pinEncoderR, incRight, RISING);
}

void loop() {
  forward(50);
//  reverse(120);
//  rotateRight(90);
//  rotateRight(90);
//  rotateRight(90);
//  rotateRight(90);
//  rotateLeft(90);
  delay(1000000);
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

  kp = 15; // trial and error
  ki = 0.005;
  kd = 0.025;

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
  
  targetTick = cm * 29.7; // Caliberated to 30.25 ticks per cm

  if (mode == 0) {
    while (encoderCountLeft < min(50, targetTick)) {
      pid = computePID();
      md.setSpeeds((0.5 * SPEED_MOVE * MOTOR_MULTIPLIER) - pid, (0.5 * SPEED_MOVE) + pid);
    }
    while (encoderCountLeft < targetTick  - 50) {
      pid = computePID();
      md.setSpeeds((SPEED_MOVE * MOTOR_MULTIPLIER) - pid, SPEED_MOVE + pid);
    }
    while (encoderCountLeft < targetTick) {
      pid = computePID();
      md.setSpeeds((0.5 * SPEED_MOVE * MOTOR_MULTIPLIER) - pid, (0.5 * SPEED_MOVE) + pid);
    }
  }
  else if (mode == 1) {
    while (encoderCountLeft < targetTick) {
      pid = computePID();
      md.setSpeeds((SPEED_CALIBRATE * MOTOR_MULTIPLIER) - pid, SPEED_CALIBRATE + pid);
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

  targetTick = cm * 30.3; // Caliberated to 30.25 ticks per cm

  if (mode == 0) {
    while (encoderCountLeft < min(50, targetTick)) {
      pid = computePID();
      md.setSpeeds(-((0.5 * SPEED_MOVE * MOTOR_MULTIPLIER) - pid), -((0.5 * SPEED_MOVE) + pid));
    }
    while (encoderCountLeft < targetTick - 50) {
      pid = computePID();
      md.setSpeeds(-((SPEED_MOVE * MOTOR_MULTIPLIER) - pid), -(SPEED_MOVE + pid));
    }
    while (encoderCountLeft < targetTick) {
      pid = computePID();
      md.setSpeeds(-((0.5 * SPEED_MOVE * MOTOR_MULTIPLIER) - pid), -((0.5 * SPEED_MOVE) + pid));
    }
  }
  else if (mode == 1) {
    while (encoderCountLeft < targetTick) {
      pid = computePID();
      md.setSpeeds(-((SPEED_CALIBRATE * MOTOR_MULTIPLIER) - pid), -(SPEED_CALIBRATE + pid));
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

  if (deg <= 90) targetTick = deg * 4.68;
  else if (deg <= 180 ) targetTick = deg * 4.56;
  else if (deg <= 360 ) targetTick = deg * 4.675;
  else targetTick = deg * 4.65;

  if (mode == 0) {
    while (encoderCountLeft < min(50, targetTick)) {
      pid = computePID();
      md.setSpeeds((0.5 * SPEED_SPIN * MOTOR_MULTIPLIER) - pid, -((0.5 * SPEED_SPIN) + pid));
    }
    while (encoderCountLeft < targetTick - 50) {
      pid = computePID();
      md.setSpeeds((SPEED_SPIN * MOTOR_MULTIPLIER) - pid, -(SPEED_SPIN + pid));
    }
    while (encoderCountLeft < targetTick) {
      pid = computePID();
      md.setSpeeds((0.5 * SPEED_SPIN * MOTOR_MULTIPLIER) - pid, -((0.5 * SPEED_SPIN) + pid));
    }
  }
  else if (mode == 1) {
    while (encoderCountLeft < targetTick) {
      pid = computePID();
      md.setSpeeds((SPEED_CALIBRATE * MOTOR_MULTIPLIER) - pid, -(SPEED_CALIBRATE + pid));
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

  if (mode == 0) {
    while (encoderCountLeft < min(50, targetTick)) {
      pid = computePID();
      md.setSpeeds(-((0.5 * SPEED_SPIN * MOTOR_MULTIPLIER) - pid), ((0.5 * SPEED_SPIN) + pid));
    }
    while (encoderCountLeft < targetTick - 50) {
      pid = computePID();
      md.setSpeeds(-((SPEED_SPIN * MOTOR_MULTIPLIER) - pid), (SPEED_SPIN + pid));
    }
    while (encoderCountLeft < targetTick) {
      pid = computePID();
      md.setSpeeds(-((0.5 * SPEED_SPIN * MOTOR_MULTIPLIER) - pid), ((0.5 * SPEED_SPIN) + pid));
    }
  }
  else if (mode == 1) {
    while (encoderCountLeft < targetTick) {
      pid = computePID();
      md.setSpeeds(-((SPEED_CALIBRATE * MOTOR_MULTIPLIER) - pid), (SPEED_CALIBRATE + pid));
    }
  }

  md.setBrakes(400, 400);
  delay(100);
}


