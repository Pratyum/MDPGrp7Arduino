#include "DualVNH5019MotorShield.h"
#include "PinChangeInt.h"
#include "SharpIR.h"

DualVNH5019MotorShield md;

#define pinEncoderL 3
#define pinEncoderR 5

#define sensorFrontLeft 1
#define sensorFrontCenter 2
#define sensorFrontRight 3

#define MODEL 1080 // 1080 (Short), 20150 (Long)


SharpIR sensor1(sensorFrontLeft, 200, 99, MODEL);
SharpIR sensor2(sensorFrontCenter, 200, 99, MODEL);
SharpIR sensor3(sensorFrontRight, 200, 99, MODEL);

enum obstacleStates {
  LEFT_ROW,
  CENTER_ROW,
  RIGHT_ROW,
  NO_OBSTACLE,
  WALL
};

obstacleStates sensorState = NO_OBSTACLE;

volatile long encoderCountLeft = 0, encoderCountRight = 0;

double target_Tick = 0, last_tick = 0;
double error = 0, integral = 0;

int pid = 0;

void setup() {
  Serial.begin(9600);

  md.init();
  pinMode(pinEncoderL, INPUT);
  pinMode(pinEncoderR, INPUT);

  PCintPort::attachInterrupt(pinEncoderL, incLeft, RISING);
  PCintPort::attachInterrupt(pinEncoderR, incRight, RISING);

  //md.setSpeeds(100, 100);
}

void loop() {
  //delay(3000);
  //  Serial.println(String(encoderCountLeft) + ", " + String(encoderCountRight));
   int distance = 120;
    moveLookingForward(distance);
//   moveForward(distance);
//  readFrontSensors();
//  rotateLeft(90);
//  Serial.println(readFrontSensors());
  delay(1000000);
}

void moveLookingForward(int distance) {
  //int left_offset =0, right_offset =0;
  int offset = 0, obstaclePosition = 0, distAfterObstacle = 0;
  
  while (distance > 0) {
    obstaclePosition = readFrontSensors();
    if (distance <= (obstaclePosition * 10)) {
      moveForward(distance);
      distance = 0;
      break;
    }
    if ((offset != 0) && (distAfterObstacle >= 40)) {
        if (offset > 0) {
          rotateLeft(90);
          while (offset > 0) {
            moveForward(10);
            offset--;
          }
          rotateRight(90);
        }
        else if (offset < 0) {
          rotateRight(90);
          while (offset < 0) {
            moveForward(10);
            offset++;
          }
          rotateLeft(90);
        }
        continue;
    }
    //    readSideSensors();
    switch (sensorState) {
      case NO_OBSTACLE:
        Serial.println("No Obstacle");
        moveForward(30);
        if (offset != 0) {
          distAfterObstacle += 30;
        }
        distance -= 30;
        break;
        
      case LEFT_ROW:
        Serial.println("Left Obstacle");
        moveForward((obstaclePosition - 1) * 10);
        rotateRight(90);
        moveForward(10);
        rotateLeft(90);
        moveForward(10);
        distAfterObstacle += 10;
        distance -= (obstaclePosition * 10);
        offset++;
        break;
        
      case RIGHT_ROW:
        Serial.println("Right Obstacle");
        moveForward((obstaclePosition - 1) * 10);
        rotateLeft(90);
        moveForward(10);
        rotateRight(90);
        moveForward(10);
        distAfterObstacle += 10;
        distance -= (obstaclePosition * 10);
        offset--;
        break;
        
      case CENTER_ROW:
        Serial.println("Center Obstacle");
        moveForward((obstaclePosition - 1) * 10);
        rotateRight(90);
        moveForward(20);
        rotateLeft(90);
        moveForward(10);
        distAfterObstacle += 10;
        distance -= (obstaclePosition * 10);
        offset += 2;
        break;
      case WALL:
        moveForward((obstaclePosition-1)*10);

    }
  }

  if (offset > 0) {
    rotateLeft(90);
    while (offset > 0) {
      moveForward(10);
      offset--;
    }
    rotateRight(90);
  }
  else if (offset < 0) {
    rotateRight(90);
    while (offset < 0) {
      moveForward(10);
      offset++;
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

  kp = 14.3; // trial and error
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

  if (angle <= 90) target_Tick = angle * 4.423;
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

  if (angle <= 90) target_Tick = angle * 4.424;
  else if (angle <= 180 ) target_Tick = angle * 4.51;
  else if (angle <= 360 ) target_Tick = angle * 4.51;
  else target_Tick = angle * 4.65;

  while (encoderCountLeft < target_Tick ) {
    pid = tuneWithPID();
    md.setSpeeds(-(200 - pid), (200 + pid));
  }
  md.setBrakes(400, 400);
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
int readFrontSensors() {
  double time = millis();
  double sensor1Val=0,sensor2Val=0,sensor3Val=0;
  sensor1Val = calibrateSensorValue(sensor1.distance());
  sensor2Val = calibrateSensorValue(sensor2.distance());
  sensor3Val = calibrateSensorValue(sensor3.distance());
  double end_time = millis();
  Serial.print("Time Taken: ");
  Serial.print(end_time-time);
  Serial.println();
  Serial.println("Values(1,2,3):"+ String(sensor1Val)+","+ String(sensor2Val)+","+ String(sensor3Val));
  Serial.println("ObstacleValues(1,2,3):"+ String(obstaclePosition(sensor1Val))+","+ String(obstaclePosition(sensor2Val))+","+ String(obstaclePosition(sensor3Val)));
  
  if ((abs(sensor1Val-sensor2Val)<=5)&&(abs(sensor2Val-sensor3Val)<=5)&&(abs(sensor3Val-sensor1Val)<=5)&&(sensor1Val<=40)){
    sensorState = WALL;
    return obstaclePosition(sensor1Val);
  }else if ((sensor1Val <= 35) && ( sensor1Val >= 0 )) {
    sensorState = LEFT_ROW;
    Serial.print("LeftRow: ");
    // Serial.print(calibrateSensorValue(sensor1.distance()));
    // Serial.print(", ");
    // Serial.println(sensor1.distance());
    return obstaclePosition(sensor1Val);
  }else if((sensor2Val <= 30) && ( sensor2Val >= 0 )){
    sensorState = CENTER_ROW;
    Serial.print("CenterRow: ");
    // Serial.print(calibrateSensorValue(sensor2.distance()));
    // Serial.print(", ");
    // Serial.println(sensor2.distance());
    return obstaclePosition(sensor2Val);
  }else if((sensor3Val <= 30) && ( sensor3Val >= 0 )){
    sensorState = RIGHT_ROW;
    Serial.print("RightRow: ");
    // Serial.print(calibrateSensorValue(sensor3.distance()));
    // Serial.print(", ");
    // Serial.println(sensor3.distance());
    return obstaclePosition(sensor3Val);
  }else{
    sensorState = NO_OBSTACLE;
    return 0;
  }
  return 0;
  //TODO: Check Right Sensor Values
  //TODO: Check Center Sensor Values
  //    if(caliberateSensorValue(sensor3.distance()) <= 30 && caliberateSensorValue(sensor3.distance())){
  //      sensorState = CENTER_ROW;
  //    }

}


