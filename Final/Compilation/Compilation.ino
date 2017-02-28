/**
 * ============================== Libraries to be included ==============================
 * include all the libraries that will be used
 */
#include "DualVNH5019MotorShield.h"
#include "PinChangeInt.h"
#include "SharpIR.h"

/**
 * ============================== Define all constant values ==============================
 * define all the constant values that will be used
 */
#define pinEncoderL 3
#define pinEncoderR 5
#define pinSensorL 0  // Left
#define pinSensorFL 1 // Front Left
#define pinSensorFC 2 // Front Center
#define pinSensorFR 3 // Front Right
#define pinSensorRF 4 // Right Front
#define pinSensorRR 5 // Right Rear

#define MODEL_SHORT 1080 // 1080 (Short), 20150 (Long)
#define MODEL_LONG 20150 // 1080 (Short), 20150 (Long)

#define WALL_GAP 10

DualVNH5019MotorShield md;

SharpIR sensorL(pinSensorL, 200, 99, MODEL_LONG);
SharpIR sensorFL(pinSensorFL, 200, 99, MODEL_SHORT);
SharpIR sensorFC(pinSensorFC, 200, 99, MODEL_SHORT);
SharpIR sensorFR(pinSensorFR, 200, 99, MODEL_SHORT);
SharpIR sensorRF(pinSensorRF, 200, 99, MODEL_SHORT);
SharpIR sensorRR(pinSensorRR, 200, 99, MODEL_SHORT);

/**
 * ============================== For mapping sensor values ==============================
 * key in values returned from sensor
 * arrMapping0 is for long-range
 * long-range (start from 20); short-range (start from 10)
 */
double arrMapping0[] = {20.3, 25.36, 33.7, 43.78, 53.8, 65.5, 74.7, 86.6, 98.4, 110.7, 125.57, 139.8};
double arrMapping1[] = {9.94, 21, 32.66, 45.5, 61, 81};
double arrMapping2[] = {9.82, 20.45, 32.55, 44.66, 52.3, 61.4};
double arrMapping3[] = {10.58, 21.66, 32.43, 35.5, 35.6, 40.1};
double arrMapping4[] = {10.51, 22.56, 36.4, 48.3, 60.7, 71};
double arrMapping5[] = {10.21, 21.68, 33.52, 42.5, 50.6, 60, 70, 80};

/**
 * ============================== Initiate global variables ==============================
 * initiate global variables that will be used
 */
volatile long encoderCountLeft, encoderCountRight;
long prevTick;
double integral;

int mode;

void setup() {
  Serial.begin(9600);

  md.init();
  pinMode(pinEncoderL, INPUT);
  pinMode(pinEncoderR, INPUT);

  PCintPort::attachInterrupt(pinEncoderL, incLeft, RISING);
  PCintPort::attachInterrupt(pinEncoderR, incRight, RISING);
}

void loop() {
  int i = 0, j = 1, val = 0;
  char commandBuffer[10];
  char command, ch;
  bool flag = true;

  mode = 0;

  while (1){
    if (Serial.available()) {
      delay(50);
      while (Serial.available()) {
        ch = Serial.read();
        commandBuffer[i] = ch;
        i++;
      }
      break;
    }
  }

  command = commandBuffer[0];

  while (j < i) {
    val *= 10; 
    val = val + (commandBuffer[j] - 48);
    j++;
  }
  
  switch (command) {
    case 'F': case 'f': // forward
      (val == 0) ? forward(10) : forward(val * 10);
      break;
    case 'B': case 'b': // reverse
      (val == 0) ? reverse(10) : reverse(val * 10);
      break;
    case 'L': case 'l': // rotateLeft
      (val == 0) ? rotateLeft(90) : rotateLeft(val);
      break;
    case 'R': case 'r': // rotateRight
      (val == 0) ? rotateRight(90) : rotateRight(val);
      break;
    case 'S': case 's': // readSensors
      readSensors();
      break;
    case 'C': case 'c': // calibrate to right wall
      mode = 1;
      calibrateWithRight();
      break;
    case 'X': case 'x': // calibrate to front wall
      mode = 1;
      calibrateWithFront();
      break;
    default: 
      flag = false;
      Serial.println("E");
  }

  if (flag) {
    Serial.println("D");
  }
}


/**
 * ============================== For Motors ==============================
 */
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

/**
 * ============================== For Sensors ==============================
 */
void readSensors() {
  String output = "";

  output += String(obstaclePosition(calibrateSensorValue(sensorL.distance(), 0)));
  output += String(obstaclePosition(calibrateSensorValue(sensorFL.distance(), 1)));
  output += String(obstaclePosition(calibrateSensorValue(sensorFC.distance(), 2)));
  output += String(obstaclePosition(calibrateSensorValue(sensorFR.distance(), 3)));
  output += String(obstaclePosition(calibrateSensorValue(sensorRF.distance(), 4)));
  output += String(obstaclePosition(calibrateSensorValue(sensorRR.distance(), 5)));

  Serial.println(output);
}

double calibrateSensorValue(double dist, int n){
  double *arr;
  int i, len;

  //int dist = sensor.distance();
  
  switch(n){
    case 0: arr = arrMapping0; len = sizeof(arrMapping0)/sizeof(*arr); break;
    case 1: arr = arrMapping1; len = sizeof(arrMapping1)/sizeof(*arr); break;
    case 2: arr = arrMapping2; len = sizeof(arrMapping2)/sizeof(*arr); break;
    case 3: arr = arrMapping3; len = sizeof(arrMapping3)/sizeof(*arr); break;
    case 4: arr = arrMapping4; len = sizeof(arrMapping4)/sizeof(*arr); break;
    case 5: arr = arrMapping5; len = sizeof(arrMapping5)/sizeof(*arr); break;
    default: return -1;
  }

  for (i = 0; i < len; i++){
    if (dist < arr[i]){
      //Serial.println("true, " + String(i) + ", " + String(dist) + ", " + arr[i]);
      int a = (i == 0)? 0 : arr[i-1];
      int offset = (n == 0)? 1 : 0;

      return modifiedMap(dist, a, arr[i], ((i + offset) * 10), ((i + offset + 1) * 10));
    }
  }
  return -1;
}

int obstaclePosition(int val){
  return ((val + 4)) / 10;
}

double modifiedMap(double x, double in_min, double in_max, double out_min, double out_max) {
  double temp = (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
  temp = (int) (4*temp + .5);
  return (double) temp/4;
}

/**
 * ============================== Calibrate robot ==============================
 */
void calibrateWithRight() {
  double distL = calibrateSensorValue(sensorRF.distance(), 4);
  double distR = calibrateSensorValue(sensorRR.distance(), 5);
  if ((distL + distR) != (2 * WALL_GAP)) {
    mode = 0;
    rotateRight(90);
    calibrateWithFront();
    rotateLeft(90);
    mode = 1;
  }
  else {
    calibrateAngle(sensorRF, 4, sensorRR, 5, 19);
  }
}

void calibrateWithFront() {
  calibrateAngle(sensorFL, 1, sensorFR, 3, 17);
  calibrateDistance(sensorFL, 1);
}
 
void calibrateAngle(SharpIR sensorL, int arrL, SharpIR sensorR, int arrR, int dist) {
  double distL = calibrateSensorValue(sensorL.distance(), arrL);
  double distR = calibrateSensorValue(sensorR.distance(), arrR);
  double diff = abs(distL - distR);
  double mean = diff / 2;

  double angle = 0;

  
  while (mean > 0.25){
    Serial.println("dist: " + String(distL) + ", " + String(distR) + ", " + String(diff) + ", " + String(mean));
    angle = (asin(mean/dist) * (180/3.14159265));
    //angle = (mean > 0.5) ? angle : angle/2;
    Serial.println("angle: " + String(angle));
    if (distL > distR){
      rotateRight(angle);
    }
    else if (distR > distL){   
      rotateLeft(angle);
    }
    
    distL = calibrateSensorValue(sensorL.distance(), arrL);
    distR = calibrateSensorValue(sensorR.distance(), arrR);
    diff = abs(distL - distR);
    mean = diff / 2;
  }
  Serial.println(mean);
}

void calibrateDistance(SharpIR sensor, int arr){
  double dist = calibrateSensorValue(sensor.distance(), arr);

  if (dist < WALL_GAP) {
    reverse(WALL_GAP - dist);
  }
  else if (dist > WALL_GAP) {
    forward(dist - WALL_GAP);
  }
}
