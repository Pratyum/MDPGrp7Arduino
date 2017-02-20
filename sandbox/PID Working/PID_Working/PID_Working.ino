#include "DualVNH5019MotorShield.h"
#include "PinChangeInt.h"
#include "SharpIR.h"

DualVNH5019MotorShield md;

#define pinEncoderL 3
#define pinEncoderR 5
#define pinSwitch 8
#define sensorFrontRight 1
#define sensorFrontLeft 2

#define MODEL 1080 // 1080 (Short), 20150 (Long)

SharpIR sensor1(sensorFrontRight, 200, 99, MODEL);
SharpIR sensor2(sensorFrontLeft, 200, 99, MODEL);
enum obstacleStates {
  LEFT_ROW,
  CENTER_ROW,
  RIGHT_ROW,
  NO_OBSTACLE
};

obstacleStates sensorState = NO_OBSTACLE;

unsigned long duration1, duration2;
volatile long encoderCountLeft = 0, encoderCountRight = 0;

double target_Tick = 0, last_tick = 0;
double error = 0, integral = 0;

int pid = 0;


void setup() {
  Serial.begin(9600);

  md.init();
  pinMode(pinEncoderL, INPUT);
  pinMode(pinEncoderR, INPUT);
  pinMode(pinSwitch, INPUT);

  PCintPort::attachInterrupt(pinEncoderL, incLeft, RISING);
  PCintPort::attachInterrupt(pinEncoderR, incRight, RISING);

  //md.setSpeeds(100, 100);
}

void loop() {
  //delay(3000);
  //  Serial.println(String(encoderCountLeft) + ", " + String(encoderCountRight));
  int distance = 120;
  moveLookingForward(distance);
//  rotateLeft(90);
//  delay(5000);
//  rotateRight(90);
  delay(1000000);
}
void moveLookingForward(int distance) {
  int left_offset =0, right_offset =0;
  while (distance > 0) {
    readFrontSensors();
    //    readSideSensors();
    switch (sensorState) {
      case NO_OBSTACLE:
        Serial.println("No Obstacle");
        moveForward(30);
        break;
      case LEFT_ROW:
        rotateRight(90);
        moveForward(10);
        rotateLeft(90);
        moveForward(30);
        left_offset += 1;
        break;
      case RIGHT_ROW:
        rotateLeft(90);
        moveForward(10);
        rotateRight(90);
        moveForward(30);
        right_offset += 1;
        break;
      case CENTER_ROW:
        rotateRight(90);
        moveForward(20);
        rotateLeft(90);
        moveForward(30);
        left_offset += 2;
        break;
    }
    distance -= 30;
  }
  if (left_offset > 0) {
    rotateLeft(90);
    while(left_offset > 0){
      moveForward(10);
      left_offset --;
    }
    rotateRight(90);
  }

  if (right_offset > 0) {
    rotateRight(90);
    while(right_offset > 0){
      moveForward(10);
      right_offset--;
    }
    rotateLeft(90);
  }
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

  kp = 15; // trial and error
  ki = 0;
  kd = 0;


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

  while (encoderCountLeft < target_Tick ) {
    pid = tuneWithPID();
    md.setSpeeds(200 - pid, 200 + pid);
  }

  md.setBrakes(400, 400);
}


void moveBackward(double cmDis) {
  pid = 0;
  encoderCountLeft = 0, encoderCountRight = 0;
  error = 0, integral = 0, last_tick = 0;

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

  if (angle <= 90) target_Tick = angle * 4.45;
  else if (angle <= 180 ) target_Tick = angle * 4.51;
  else if (angle <= 360 ) target_Tick = angle * 4.51;
  else target_Tick = angle * 4.65;

  while (encoderCountLeft < target_Tick ) {
    pid = tuneWithPID();
    md.setSpeeds(-(200 - pid), (200 + pid));
  }
  md.setBrakes(400, 400);
}


bool checkForObstcle() {
  //TODO : Add the third sensor
  Serial.print(calibrateSensorValue(sensor1.distance()));
  //  Serial.print(",");
  //  Serial.println(sensor2.distance());
  Serial.println();
  if (calibrateSensorValue(sensor1.distance()) <= 30 && calibrateSensorValue(sensor1.distance()) > 0) {
    Serial.println("Obstacle detected");
    return true;
  }
  return false;
}


int calibrateSensorValue(int val) {
  /**
   * 20cm distance has the most accurate reading
   * with each increment of 10cm, there will be 2cm additional increment in readings
   */
  //   Serial.println("val"+ String(val));
  return (val + 4) / 1.2;
}

int obstaclePosition(int val) {
  /**
   * range of values are from
   * (1.2n - 4) - (4 + 0.1n) to (1.2n - 4) + (4 + 0.1n)
   */
  int i;
  for (i = 1; i <= 10; i++) {
    if (val <= (13 * i)) {
      return i;
    }
  }
  return 0;
}
void readFrontSensors() {
  if (calibrateSensorValue(sensor1.distance()) <= 30 && calibrateSensorValue(sensor1.distance()) >= 0 ) {
    sensorState = RIGHT_ROW;
  }else if(calibrateSensorValue(sensor2.distance()) <= 30 && calibrateSensorValue(sensor2.distance()) >= 0){
    sensorState = LEFT_ROW;
  }else{
    sensorState = NO_OBSTACLE;
  }
  //TODO: Check Right Sensor Values
  //TODO: Check Center Sensor Values
  //    if(caliberateSensorValue(sensor3.distance()) <= 30 && caliberateSensorValue(sensor3.distance())){
  //      sensorState = CENTER_ROW;
  //    }

}


