/**
 * ============================== Libraries to be included ==============================
 * include all the libraries that will be used
 */
#include "DualVNH5019MotorShield.h"
#include "PinChangeInt.h"
#include "SharpIR.h"

/**
 * ============================== Define all constant values ==============================
 * define al the constant values that will be used
 */
#define pinEncoderL 3
#define pinEncoderR 5
#define pinSensorR 0  // Right
#define pinSensorFL 1 // Front Left
#define pinSensorFC 2 // Front Center
#define pinSensorFR 3 // Front Right
#define pinSensorRF 4 // Right Front
#define pinSensorRR 5 // Right Rear

#define MODEL_SHORT 1080 // 1080 (Short), 20150 (Long)
#define MODEL_LONG 20150 // 1080 (Short), 20150 (Long)

#define WALL_GAP 10

DualVNH5019MotorShield md;

SharpIR sensorR(pinSensorR, 200, 99, MODEL_LONG);
SharpIR sensorFL(pinSensorFL, 200, 99, MODEL_SHORT);
SharpIR sensorFC(pinSensorFC, 200, 99, MODEL_SHORT);
SharpIR sensorFR(pinSensorFR, 200, 99, MODEL_SHORT);
SharpIR sensorRF(pinSensorLB, 200, 99, MODEL_SHORT);
SharpIR sensorRR(pinSensorLF, 200, 99, MODEL_SHORT);

/**
 * ============================== For mapping sensor values ==============================
 * key in values returned from sensor
 * arrMapping0 is for long-range
 * long-range (start from 20); short-range (start from 10)
 */
int arrMapping0[] = {20, 27, 37, 47, 58, 69, 80, 91, 100, 120, 130, 140, 150};
int arrMapping1[] = {10, 20, 30, 40, 50, 60, 70, 80};
int arrMapping2[] = {10, 20, 30, 40, 50, 60, 70, 80};
int arrMapping3[] = {10, 20, 30, 40, 50, 60, 70, 80};
int arrMapping4[] = {10, 20, 30, 40, 50, 60, 70, 80};
int arrMapping5[] = {10, 20, 30, 40, 50, 60, 70, 80};

/**
 * ============================== Initiate global variables ==============================
 * initiate global variables that will be used
 */
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
  int i = 0, j = 1, val = 0;
  char commandBuffer[10];
  char command, ch;
  bool flag = true;

  while (1){
    if (Serial.available()) {
      delay(10);
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

  val = (val == 0) ? 1 : val;

  switch (command) {
    case 'F': case 'f': // forward
      forward(val * 10); 
      break;
    case 'B': case 'b': // reverse
      reverse(val * 10);
      break;
    case 'L': case 'l': // rotateLeft
      rotateLeft(val);
      break;
    case 'R': case 'r': // rotateRight
      rotateRight(val);
      break;
    case 'S': case 's': // readSensors
      readSensors();
      break;
    case 'C': case 'c': // calibrate to right wall
      calibrateAngle(sensorRF, 4, sensorRR, 5, 10);
      calibrateDistance(sensorRF, 4);
      break;
    case 'X': case 'x': // calibrate to front wall
      calibrateAngle(sensorFL, 1, sensorFR, 3, 10);
      calibrateDistance(sensorFR, 1);
      break;
    default: 
      flag = false;
      Serial.println("Invalid Command!");
  }

  if (flag) {
    Serial.print(command);
    if (val != 0) Serial.print(val);
    Serial.println(" done!");
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

  p = error * kp;
  i = integral * ki;
  d = (prevTick - encoderCountLeft) * kd;
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
    md.setSpeeds(200 - pid, 200 + pid);
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
    md.setSpeeds(-(200 - pid), -(200 + pid));
  }
  md.setBrakes(400, 400);
  delay(100);
}

int rotateRight(double deg) {
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
    md.setSpeeds(200 - pid, -(200 + pid));
  }
  md.setBrakes(400, 400);
  delay(100);
}

int rotateLeft(double deg) {
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
    md.setSpeeds(-(200 - pid), (200 + pid));
  }
  md.setBrakes(400, 400);
  delay(100);
}

/**
 * ============================== For Sensors ==============================
 */
void readSensors() {
  String output = "";

  output += String(obstaclePosition(calibrateSensorValue(sensorR.distance(), 0)));
  output += String(obstaclePosition(calibrateSensorValue(sensorFL.distance(), 1)));
  output += String(obstaclePosition(calibrateSensorValue(sensorFC.distance(), 2)));
  output += String(obstaclePosition(calibrateSensorValue(sensorFR.distance(), 3)));
  output += String(obstaclePosition(calibrateSensorValue(sensorLB.distance(), 4)));
  output += String(obstaclePosition(calibrateSensorValue(sensorLF.distance(), 5)));

  Serial.println(output);
}

int calibrateSensorValue(int dist, int n){
  int *arr;
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
      Serial.println("true, " + String(i) + ", " + String(dist) + ", " + arr[i]);
      int a = (i == 0)? 0 : arr[i-1];
      int offset = (n == 0)? 1 : 0;

      return map(dist, a, arr[i], ((i + offset) * 10), ((i + offset + 1) * 10));
    }
  }
  return -1;
}

int obstaclePosition(int val){
  return ((val + 4)) / 10;
}

/**
 * ============================== Calibrate robot ==============================
 */
void calibrateAngle(SharpIR sensorL, int arrL, SharpIR sensorR, int arrR, int dist) {
  int distL = calibrateSensorValue(sensorL.distance(), arrL);
  int distR = calibrateSensorValue(sensorR.distance(), arrR);
  int diff = abs(distL - distR);
  int mean = diff / 2;

  int angle = 0;

  while (diff > 0.2){
    angle = (asin(mean/dist) * (180/3.14159265));
    
    if (distL > distR){
      rotateRight(angle);
    }
    else if (distR > distL){   
      rotateLeft(angle);
    }
    
    distL = calibrateSensorValue(sensorL.distance(), arrL);
    distR = calibrateSensorValue(sensorR.distance(), arrR);
    diff = abs(distL - distR);
  }
}

void calibrateDistance(SharpIR sensor, int arr){
  int dist = calibrateSensorValue(sensor.distance(), arr);

  if (dist < WALL_GAP) {
    reverse(WALL_GAP - dist);
  }
  else if (dist > WALL_GAP) {
    forward(dist - WALL_GAP);
  }
}
