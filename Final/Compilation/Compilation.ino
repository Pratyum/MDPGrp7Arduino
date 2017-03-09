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

#define SPEED_MOVE 250
#define SPEED_SPIN 250
#define SPEED_CALIBRATE 100
#define MOTOR_MULTIPLIER 0.92

#define STEPS_TO_CALIBRATE 3

DualVNH5019MotorShield md;

SharpIR sensorL(pinSensorL, MODEL_LONG);
SharpIR sensorFL(pinSensorFL, MODEL_SHORT);
SharpIR sensorFC(pinSensorFC, MODEL_SHORT);
SharpIR sensorFR(pinSensorFR, MODEL_SHORT);
SharpIR sensorRF(pinSensorRF, MODEL_SHORT);
SharpIR sensorRR(pinSensorRR, MODEL_SHORT);

/**
 * ============================== For mapping sensor values ==============================
 * key in values returned from sensor
 * arrMapping0 is for long-range
 * long-range (start from 20); short-range (start from 10)
 */
// values on 9 mar
double arrMapping0[] = {17.85, 20.84, 27.43, 47.22, 58.60, 72.41, 72.90, 89.02, 131.06};
double arrMapping1[] = {9.93, 20.89, 32.17, 48.84, 71.14};
double arrMapping2[] = {9.68, 20.50, 30.14, 40.52, 51};
double arrMapping3[] = {10.17, 21.07, 31.84, 46.24, 64.60};
double arrMapping4[] = {9.68, 20.50, 31.73, 43.81, 62.69};
double arrMapping5[] = {9.89, 21.27, 32.61, 40.81, 49.80};
/**
 * ============================== Initiate global variables ==============================
 * initiate global variables that will be used
 */
volatile long encoderCountLeft, encoderCountRight;
long prevTick;
double integral;

int mode, loop_counter;

bool calibrateAutoChecked;

void setup() {
  Serial.begin(9600);

  md.init();
  pinMode(pinEncoderL, INPUT);
  pinMode(pinEncoderR, INPUT);

  PCintPort::attachInterrupt(pinEncoderL, incLeft, RISING);
  PCintPort::attachInterrupt(pinEncoderR, incRight, RISING);
  loop_counter = 0;
  calibrateAutoChecked = true;
}

void loop() {
  int i = 0, j = 1, val = 0;
  char commandBuffer[10];
  char command, ch;
  bool flag = true;

  mode = 0;

  while (1){
    if (Serial.available()) {
      ch = Serial.read();
      commandBuffer[i] = ch;
      i++;
      
      if (ch == '|'){
        break;
      }
    }
  }

  command = commandBuffer[0];

  while (j < (i-1)) {
    val *= 10; 
    val = val + (commandBuffer[j] - 48);
    j++;
  }
  
  switch (command) {
    case 'F': case 'f': // forward
      (val == 0) ? forward(10) : forward(val * 10);
      loop_counter++;
      delay(100);
      break;
    case 'B': case 'b': // reverse
      (val == 0) ? reverse(10) : reverse(val * 10);
      loop_counter++;
      delay(100);
      break;
    case 'L': case 'l': // rotateLeft
      (val == 0) ? rotateLeft(90) : rotateLeft(val);
      loop_counter++;
      delay(100);
      break;
    case 'R': case 'r': // rotateRight
      (val == 0) ? rotateRight(90) : rotateRight(val);
      loop_counter++;
      delay(100);
      break;
    case 'S': case 's': // readSensors
      flag = false;
      readSensors();
      // loop_counter++;  
      break;
    case 'C': case 'c': // calibrate to right wall
      mode = 1;
      calibrateWithRight();
      // loop_counter++;  
      break;
    case 'X': case 'x': // calibrate to front wall (sensor 1, 3)
      mode = 1;
      calibrateWithFront();
      // loop_counter++;  
      break;
    case 'Y': case 'y': // calibrate to front wall (sensor 1, 2)
      mode = 1;
//      calibrateWithFront(1);
      // loop_counter++;  
      break;
    case 'Z': case 'z': // calibrate to front wall (sensor 2, 3)
      mode = 1;
//      calibrateWithFront(2);
      // loop_counter++;  
      break;
    default: 
      flag = false;
      Serial.println("E");
  }
  
  if (!Serial.available()) {
    if (flag) {
      if (mode == 1) {
        Serial.println("D");
      }
      else {
        // TODO: read sensors values and store it so that we dont need to read again for calibration
        readSensors();
      }
      
    }
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
//  Serial.println(String(encoderCountLeft) + ", " + String(encoderCountRight) + ", " + String(encoderCountLeft - encoderCountRight));
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

  targetTick = cm * 29.7; // Caliberated to 30.25 ticks per cm

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

  if (deg <= 90) targetTick = deg * 4.39; //4.523
  else if (deg <= 180 ) targetTick = deg * 4.62;
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

  if (deg <= 90) targetTick = deg * 4.39; //4.424;
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

/**
 * ============================== For Sensors ==============================
 */
void readSensors() {
  double distFL = calibrateSensorValue(sensorFL.distance(), 1);
  double distFC = calibrateSensorValue(sensorFC.distance(), 2);
  double distFR = calibrateSensorValue(sensorFR.distance(), 3);
  double distRF = calibrateSensorValue(sensorRF.distance(), 4);
  double distRR = calibrateSensorValue(sensorRR.distance(), 5);
  double distL = calibrateSensorValue(sensorL.distance(), 0);

  String output = "";

  // target side = right, other side = front
  bool calibrate_front = false;

  mode = 1;

  if (loop_counter >= STEPS_TO_CALIBRATE) {
    // check if target side can calibrate angle
    if ((abs(distFL - distFR) < 5) && ((distFL + distFR) <= (3 * WALL_GAP))) {
      calibrateAngle(sensorFL, 1, sensorFR, 3, 17);
      calibrateDistance(sensorFL, 1);
      calibrate_front = true;
      loop_counter = 0;
    }
    else if ((abs(distFL - distFC) < 5) && ((distFL + distFC) <= (3 * WALL_GAP))) {
      calibrateAngle(sensorFL, 1, sensorFC, 2, 9);
      calibrateDistance(sensorFC, 2);
      calibrate_front = true;
      loop_counter = 0;
    }
    else if ((abs(distFC - distFR) < 5) && ((distFC + distFR) <= (3 * WALL_GAP))) {
      calibrateAngle(sensorFC, 2, sensorFR, 3, 9);
      calibrateDistance(sensorFC, 2);
      calibrate_front = true;
      loop_counter = 0;
    }

    // check for 1 obstacle on other side
    // if yes, calibrate dist on other side
    if (calibrate_front) {
      if (distRF <= (WALL_GAP + 4)) {
        mode = 0;
        rotateRight(90);
        mode = 1;
        calibrateDistance(sensorFL, 1);
        mode = 0;
        rotateLeft(90);
        mode = 1;
      }
      else if (distRR <= (WALL_GAP + 4)) {
        mode = 0;
        rotateRight(90);
        mode = 1;
        calibrateDistance(sensorFR, 3);
        mode = 0;
        rotateLeft(90);
        mode = 1;
      }
    }
    else if ((abs(distRF - distRR) < 5) && ((distRF + distRR) <= (3 * WALL_GAP))) {
      if (distFL <= (WALL_GAP + 4)) {
        calibrateDistance(sensorFL, 1);
      }
      else if (distFC <= (WALL_GAP + 4)) {
        calibrateDistance(sensorFC, 2);
      }
      else if (distFR <= (WALL_GAP + 4)) {
        calibrateDistance(sensorFR, 3);
      }

      mode = 0;
      rotateRight(90);
      mode = 1;
      calibrateAngle(sensorFL, 1, sensorFR, 3, 17);
      calibrateDistance(sensorFL, 1);
      mode = 0;
      rotateLeft(90);
      mode = 1;
      loop_counter = 0;
    }
  }
  else {
    if ((abs(distRF - distRR) < 5) && ((distRF + distRR) <= (3 * WALL_GAP))) {
      if (((distRF + distRR) < ((2 * WALL_GAP) - 1)) || ((distRF + distRR) > ((2 * WALL_GAP) + 1))) {
        mode = 0;
        rotateRight(90);
        mode = 1;
        calibrateAngle(sensorFL, 1, sensorFR, 3, 17);
        calibrateDistance(sensorFL, 1);
        mode = 0;
        rotateLeft(90);
        mode = 1;
      }
      else {
        calibrateAngle(sensorRF, 4, sensorRR, 5, 19);
      }
    }
  }


  output += String(obstaclePosition(distFL, 1));
  output += String(obstaclePosition(distFC, 1));
  output += String(obstaclePosition(distFR, 1));
  output += String(obstaclePosition(distRF, 1));
  output += String(obstaclePosition(distRR, 1));
  output += String(obstaclePosition(distL, 0));
  
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
  return i*10;
}

int obstaclePosition(int val, int shortrange){
  int tmp = 0;
  if (shortrange == 1) {
    tmp = (val + 4) / 10;
    if ((tmp >= 1) && (tmp <= 2)) {
      return tmp;
    }
    else {
      return 0; 
    }    
  }
  else {
    tmp = (val - 6) / 10;
    if ((tmp >= 1) && (tmp <= 3)) {
      return tmp;
    }
    else {
      return 0; 
    }
  }
}

double modifiedMap(double x, double in_min, double in_max, double out_min, double out_max) {
  double temp = (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
  // temp = (int) (4*temp + .5);
  // return (double) temp/4;
  return temp;
}

/**
 * ============================== Calibrate robot ==============================
 */
void calibrateWithRight() {
  double distL = calibrateSensorValue(sensorRF.distance(), 4);
  double distR = calibrateSensorValue(sensorRR.distance(), 5);

  if ((abs(distL - distR) < 5) && ((distL + distR) <= (3 * WALL_GAP))) {
    if (((distL + distR) < ((2 * WALL_GAP) - 1)) || ((distL + distR) > ((2 * WALL_GAP) + 1))) {
      mode = 0;
      rotateRight(90);
      mode = 1;
      calibrateWithFront();
      mode = 0;
      rotateLeft(90);
      mode = 1;
    }
    else {
      calibrateAngle(sensorRF, 4, sensorRR, 5, 19);
      calibrateAutoChecked = true;
      loop_counter = 0;
    }
  }
}

void calibrateWithFront() {
  double distL = calibrateSensorValue(sensorFL.distance(), 1);
  double distC = calibrateSensorValue(sensorFC.distance(), 2);
  double distR = calibrateSensorValue(sensorFR.distance(), 3);

  if ((abs(distL - distR) < 5) && ((distL + distR) <= (3 * WALL_GAP))) {
    calibrateAngle(sensorFL, 1, sensorFR, 3, 17);
    calibrateDistance(sensorFL, 1);
    calibrateAutoChecked = true;
    loop_counter = 0;
  }
  else if ((abs(distL - distC) < 5) && ((distL + distC) <= (3 * WALL_GAP))) {
    calibrateAngle(sensorFL, 1, sensorFC, 2, 9);
    calibrateDistance(sensorFC, 2);
    calibrateAutoChecked = true;
    loop_counter = 0;
  }
  else if ((abs(distC - distR) < 5) && ((distC + distR) <= (3 * WALL_GAP))) {
    calibrateAngle(sensorFC, 2, sensorFR, 3, 9);
    calibrateDistance(sensorFC, 2);
    calibrateAutoChecked = true;
    loop_counter = 0;
  }
}
 
void calibrateAngle(SharpIR sensorL, int arrL, SharpIR sensorR, int arrR, int dist) {
  double distL = calibrateSensorValue(sensorL.distance(), arrL);
  double distR = calibrateSensorValue(sensorR.distance(), arrR);
  double diff = abs(distL - distR);
  double mean = diff / 2;

  double angle = 0;

  
  while (mean > 0.25){
    // Serial.println("dist: " + String(distL) + ", " + String(distR) + ", " + String(diff) + ", " + String(mean));
    angle = (asin(mean/dist) * (180/3.14159265));
    angle = (mean > 0.5) ? angle : angle/2;
    // Serial.println("angle: " + String(angle));
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
  // Serial.println(mean);
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

void calibrateAuto() {
  // auto-calibration code
  if ((loop_counter >= STEPS_TO_CALIBRATE) && (calibrateAutoChecked == false)) {
    calibrateAuto();
  }
  else {
    calibrateWithRight();
  }
  calibrateWithFront();
}

