#include "DualVNH5019MotorShield.h"
#include "PinChangeInt.h"
#include "SharpIR.h"

DualVNH5019MotorShield md;

#define pinEncoderL 3
#define pinEncoderR 5
#define pinSwitch 8
#define sensorFrontRight 1
#define sensorFrontLeft 2
#define sensorFrontCenter 3


#define MODEL 1080 // 1080 (Short), 20150 (Long)

SharpIR sensor1(sensorFrontRight, 200, 99, MODEL);
SharpIR sensor2(sensorFrontLeft, 200, 99, MODEL);
SharpIR sensor3(sensorFrontCenter, 200, 99, MODEL);

enum obstacleStates {
  LEFT_ROW,
  CENTER_ROW,
  RIGHT_ROW,
  NO_OBSTACLE,
  WALL
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
  boolean bool_check = false;
  
  while (distance > 0) {
    readFrontSensors();
    //    readSideSensors();
    switch (sensorState) {
      case NO_OBSTACLE:
        Serial.println("No Obstacle");
        while(left_offset != 0 || right_offset != 0){
          if (left_offset != 0){
            rotateLeft(90);
            moveForward(10);
            rotateRight(90);
            left_offset--; 
          }
          if (right_offset != 0){
            rotateRight(90);
            moveForward(10);
            rotateLeft(90);
            right_offset--;
          }
        }
        moveForward(10);
        distance -= 10;
        break;
            
      case LEFT_ROW:
        Serial.println("Left");
        rotateRight(90);
        bool_check = check_if_can_avoid();
        if (bool_check){
            Serial.println("Conventional Way");
            //go conventional
            moveForward(10);
            rotateLeft(90);
            moveForward(40);
            rotateLeft(90);
            moveForward(10);
            rotateRight(90);
            //Clear Obstacle, bot in front of block
            distance -= 40;
            Serial.println("Bot ahead of block");
        }else{
            //go alternative
            rotateLeft(180);
            moveForward(30);
            rotateRight(90);
            moveForward(40);
            rotateRight(90);
            moveForward(30);
            rotateLeft(90);
            //Clear Obstacle, bot in front of block
            distance -= 40;
            Serial.println("Bot ahead of block");
        }
        break;
            
      case RIGHT_ROW:
        Serial.println("Right");
            rotateLeft(90);
            bool_check = check_if_can_avoid();
            if (bool_check){
                Serial.println("Conventional Way");
                //go conventional
                moveForward(10);
                rotateRight(90);
                moveForward(40);
                rotateRight(90);
                moveForward(10);
                rotateLeft(90);
                //Clear Obstacle, bot in front of block
                distance -= 40;
                Serial.println("Bot ahead of block");
            }else{
                //go alternative
                rotateLeft(180);
                moveForward(30);
                rotateLeft(90);
                moveForward(40);
                rotateLeft(90);
                moveForward(30);
                rotateRight(90);
                //Clear Obstacle, bot in front of block
                distance -= 40;
                Serial.println("Bot ahead of block");
            }
        break;
            
      case CENTER_ROW:
        Serial.println("center");
        rotateLeft(90);
        bool_check = check_if_can_avoid();
        if(bool_check){
          moveForward(10);
          rotateRight(90);  
          right_offset++;
        }else{
          rotateLeft(180);
          moveForward(10);
          rotateLeft(90);
          left_offset++;
        }
        break;
            
      case WALL :
        Serial.println("wall");
        moveForward(distance-4);
        break;
    }
    
  }
}

boolean check_if_can_avoid(){
    //check if wall is at least 2 grids ahead
    if (obstaclePosition(sensor1.distance())<2 || 
        obstaclePosition(sensor2.distance())<2 || 
        obstaclePosition(sensor3.distance())<2  )
    {
        //wall here go by alternative
        return false;
    }else{
        //no wall, go by conventional way
        return true;
    }
}

void incLeft() {
  encoderCountLeft++;
}

void incRight() {
  encoderCountRight++;
}

double tuneWithPID() {
//  Serial.println(String(encoderCountLeft) + ", " + String(encoderCountRight) + ", " + String(encoderCountLeft - encoderCountRight));
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
  delay(100);
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
  delay(100);
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
  //Serial.println(String(calibrateSensorValue(sensor1.distance()))+","+String(calibrateSensorValue(sensor2.distance()))+","+String(calibrateSensorValue(sensor3.distance())));
  int sensor1val = 0, sensor2val = 0, sensor3val = 0;
  sensor1val = sensor1.distance();
  sensor2val = sensor2.distance();
  sensor3val = sensor3.distance();
  if (obstaclePosition(sensor1val) == 1) {
    sensorState = LEFT_ROW;
  }else if(obstaclePosition(sensor2val) == 1){
    sensorState = CENTER_ROW;
  }else if(obstaclePosition(sensor3val) == 1){
    sensorState = RIGHT_ROW;
  }else{
    sensorState = NO_OBSTACLE;
  }
//  if (((calibrateSensorValue(sensor1.distance())-calibrateSensorValue(sensor2.distance())) < 5)&&((calibrateSensorValue(sensor2.distance())-calibrateSensorValue(sensor3.distance())) < 5)&&((calibrateSensorValue(sensor3.distance())-calibrateSensorValue(sensor1.distance())) < 5)){
//    if (sensorState != NO_OBSTACLE){
//      sensorState = WALL;
//    }
//  }
  //TODO: Check Right Sensor Values
  //TODO: Check Center Sensor Values
  //    if(caliberateSensorValue(sensor3.distance()) <= 30 && caliberateSensorValue(sensor3.distance())){
  //      sensorState = CENTER_ROW;
  //    }

}

bool isClearSide(){
  if (calibrateSensorValue(sensor1.distance()) <= 20 && calibrateSensorValue(sensor1.distance()) >= 0 ) {
    return false;
  }
  if(calibrateSensorValue(sensor2.distance()) <= 20 && calibrateSensorValue(sensor2.distance()) >= 0){
    return false;
  }
  return true;

}



