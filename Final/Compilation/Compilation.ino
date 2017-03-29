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
#define pinGreenLED 11 // green debugging led
#define pinRedLED 13 // red debugging led

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

#define SPEED_MOVE 350
#define SPEED_SPIN 350
#define SPEED_CALIBRATE 150
#define MOTOR_MULTIPLIER 1

// ********** change values here! **********
#define TICKS_PER_CM 25.9
#define TICKS_ROTATE_RIGHT 4.0
#define TICKS_ROTATE_LEFT 4.0
#define STEPS_PER_EXTRA_TICK 195 // smaller left, bigger right

#define RANGE_OF_LEFT_SENSOR 5
#define RANGE_OF_FRONT_SENSOR 2
#define RANGE_OF_RIGHT_SENSOR 3

#define KP 10 // 18 - smaller righter, bigger lefter
#define KI 0.0001// 0.005
#define KD 0.040 // 0.025

#define DELAY_PER_MOVE 90
// ********** end of values to change! **********

#define TICKS_TO_RAMP 100

#define STEPS_TO_CALIBRATE 5
#define STEPS_TO_BEST_CALIBRATE 3

//#define PI 3.14159265

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
// values on 24 mar
double arrMapping0[] = {18.86, 24.20, 32.40, 41.62, 52.23, 63.15, 75.36, 86.6, 98.4};
double arrMapping1[] = {10.06, 20.79, 33.96, 59.98, 70.27};
double arrMapping2[] = {10.08, 20.79, 31.52, 43.43, 50.32};
double arrMapping3[] = {10.35, 23.12, 38.17, 56.42, 76.63};
double arrMapping4[] = {10.35, 20.89, 32.39, 45.41, 59.30};
double arrMapping5[] = {10.00, 20.79, 30.74, 43.43, 53.33};

/**
 * ============================== Initiate global variables ==============================
 * initiate global variables that will be used
 */
volatile long encoderCountLeft, encoderCountRight;
long prevTick;
double integral;

int step_counter, step_best_calibrate;
bool obstacle_left_center, obstacle_left_rear, obstacle_right_rear;
bool opportunity_calibrate_left, opportunity_calibrate_front, opportunity_calibrate_right;

bool mode_fastest_path, mode_fastest_diagonal, mode_calibration, forward_command, sensor_command;

void setup() {
  Serial.begin(9600);
  Serial.setTimeout(50);

  pinMode(pinGreenLED, OUTPUT);
  pinMode(pinRedLED, OUTPUT);
  digitalWrite(pinGreenLED, HIGH);
  digitalWrite(pinRedLED, LOW);
  
  md.init();
  pinMode(pinEncoderL, INPUT);
  pinMode(pinEncoderR, INPUT);

  PCintPort::attachInterrupt(pinEncoderL, incLeft, RISING);
  PCintPort::attachInterrupt(pinEncoderR, incRight, RISING);
  step_counter = 0;
  step_best_calibrate = 0;
  mode_fastest_path = false;
  mode_fastest_diagonal = false;
  mode_calibration = false;
  obstacle_left_center = false;
  obstacle_left_rear = false;
  obstacle_right_rear = false;
}

void loop() {
  int i = 0, j = 1;
  double val;
  String valString = "";
  char commandBuffer[50];
  char command, ch;
  bool flag = true;
  int emergency = 0;

  forward_command = false;
  sensor_command = false;

  opportunity_calibrate_left = false;
  opportunity_calibrate_front = false;
  opportunity_calibrate_right = false;

  while (1){
    if (Serial.available()) {
      ch = Serial.read();
  
      if ((ch == 'a') || (ch == 'A')) {
        continue;
      }

      if ((ch == 'z') || (ch == 'Z') || (ch == 'c') || (ch == 'C') || (ch == 'e') || (ch == 'E')) {
        switch(ch) {
          case 'z': case 'Z':
            opportunity_calibrate_left = true;
            break;
          // case 'x': case 'X':
          //   opportunity_calibrate_front = true;
          //   break;
          case 'c': case 'C':
            opportunity_calibrate_right = true;
            break;
          case 'e': case 'E':
            emergency = 1;
            break;
        }
        delay(10);
        ch = Serial.read();
        continue;
      }
      
      commandBuffer[i] = ch;
      i++;

      if (ch == '|'){
        digitalWrite(pinGreenLED, LOW);
        digitalWrite(pinRedLED, HIGH);
        break;
      }
    }
  }
  
  command = commandBuffer[0];

  while (j < (i-1)) {
    valString += commandBuffer[j];
    j++;
  }
  
  val = valString.toDouble();
  int start_inst_index = 0;
  switch (command) {
    case 'M': case 'm': // mode
      flag = false;
      if (val = 2) {
        mode_fastest_path = true;
        mode_fastest_diagonal = true;
      }
      else if (val == 1) {
        mode_fastest_path = true;
        mode_fastest_diagonal = false;
      }
      else if (val == 0) {
        mode_fastest_path = false;
        mode_fastest_diagonal = false;
      }
      break;
    case 'F': case 'f': // forward
        (val == 0) ? forward(10) : forward(val * 10);
//      if (val == 0){
//        forward(10);
//      }
//      else if (val >= 4){
//        while(val >= 4){
//          forward(40);
//          step_counter++;
//          step_best_calibrate++;
//          autoCalibrate(1);
//          val -= 4;
//        }
//        if (val > 0){9
//          forward(val * 10); 
//        }
//      }
//      else {
//        forward(val * 10);
//      }

      if ((val == 0) || (val == 1)) {
        forward_command = true;
      }
      step_counter++;
      step_best_calibrate++;
      break;
    case 'B': case 'b': // reverse
      (val == 0) ? reverse(10) : reverse(val * 10);
      step_counter++;
      step_best_calibrate++;
      break;
    case 'L': case 'l': // rotateLeft
      (val == 0) ? rotateLeft(90) : rotateLeft(val);
      step_counter++;
      step_best_calibrate++;
      break;
    case 'R': case 'r': // rotateRight
      (val == 0) ? rotateRight(90) : rotateRight(val);
      step_counter++;
      step_best_calibrate++;
      break;
    case 'j': case 'J':
      curveLeft();
      break;
    case 'k': case 'K':
      curveRight();
      break;
    case 'S': case 's': // readSensors
      flag = false;
      sensor_command = true;
      autoCalibrate(1);
      readSensors();
      break;
    // case 'C': case 'c': // calibrate to right wall
    //   flag = false;
    //   step_counter = (calibrateWithRight()) ? 0 : step_counter;
    //   break;
    case 'X': case 'x': 
      flag = false;
      step_counter = (calibrateWithFront()) ? 0 : step_counter;
      break;
    // case 'Z': case 'z':
    //   flag = false; 
    //   step_counter = (calibrateWithLeft()) ? 0 : step_counter;
    //   break;
    // case 'E': case 'e':
    //   flag = false;
    //   (autoCalibrate(1));
      break;
    default: 
      flag = false;
//      Serial.println("E");
  }
  
  if (flag) {
    // check if command was action, calibrate if yes
    if (mode_fastest_path) {
      if ((!mode_fastest_diagonal) && (Serial.available())) {
        autoCalibrate(1);
      }
    }
    else {
      autoCalibrate(emergency);  
    }
    
  }

  if (!mode_fastest_path) {
    if (flag) {
      // TODO: read sensors values and store it so that we dont need to read again for calibration
      readSensors();
    }
  }
  else {
    delay(10);
    if (!Serial.available()) {
      calibrateDistance(sensorFC, 2);
    }
  }
  digitalWrite(pinGreenLED, HIGH);
  digitalWrite(pinRedLED, LOW); 
}


/**
 * ============================== For Motors ==============================
 */
void incLeft() {
  (encoderCountLeft % STEPS_PER_EXTRA_TICK == 0) ? encoderCountLeft += 2:encoderCountLeft++;
}

void incRight() {
  encoderCountRight++;
}

double computePID() {
//  Serial.println(String(encoderCountLeft) + ", " + String(encoderCountRight) + ", " + String(encoderCountLeft - encoderCountRight));
  double kp, ki, kd, p, i, d, error, pid;
  
  kp = KP; // 10 trial and error
  ki = KI; // 0.005
  kd = KD; // 0.025

  error = encoderCountLeft - encoderCountRight;
  integral += error;

  p = kp * error;
  i = ki * integral;
  d = kd * (prevTick - encoderCountLeft);
  pid = p + i + d;

  prevTick = encoderCountLeft;

  return pid;
}

double computeCurvePID(bool isLeft) {
//  Serial.println(String(encoderCountLeft) + ", " + String(encoderCountRight) + ", " + String(encoderCountLeft - encoderCountRight));
  double kp, ki, kd, p, i, d, error, pid;
  
  kp = KP; // 10 trial and error
  ki = KI; // 0.005
  kd = KD; // 0.025
  
  if(isLeft) {
    error = 2.50877193 * encoderCountLeft - encoderCountRight;
  }
  else {
    error = encoderCountLeft - 2.50877193 * encoderCountRight;
  }
  
  integral += error;

  p = kp * error;
  i = ki * integral;
  d = kd * (prevTick - encoderCountLeft);
  pid = p + i + d;

  prevTick = encoderCountLeft;

  return pid;
}


void curveLeft() {
  double pid;
  int targetTick;
  integral = 0;
  encoderCountLeft = encoderCountRight = prevTick = 0;
  double SPEED_RAMP = 0;

  double multiplier = 2.50877193;
  double cm = PI * 5.7;
//  Serial.println(cm);
  targetTick = ((cm * 1.15)-1.5) * TICKS_PER_CM; // Caliberated to 30.25 ticks per cm

  while (encoderCountLeft < min(TICKS_TO_RAMP, targetTick)) {
    SPEED_RAMP = modifiedMap(encoderCountLeft, 0, TICKS_TO_RAMP, 50, SPEED_MOVE);
    pid = computeCurvePID(true);
    md.setSpeeds((SPEED_RAMP / multiplier) - pid, SPEED_RAMP + pid);
  }
  while (encoderCountLeft < targetTick  - TICKS_TO_RAMP) {
    pid = computeCurvePID(true);
    md.setSpeeds((SPEED_MOVE / multiplier) - pid, SPEED_MOVE + pid);
  }
  while (encoderCountLeft < targetTick) {
    SPEED_RAMP = modifiedMap(encoderCountLeft, targetTick - TICKS_TO_RAMP, targetTick, SPEED_MOVE, 50);
    pid = computeCurvePID(true);
    md.setSpeeds((SPEED_RAMP / multiplier) - pid, SPEED_RAMP + pid);
  }

  md.setBrakes(400, 400);
  delay(DELAY_PER_MOVE);
}

void curveRight() {
  double pid;
  int targetTick;
  integral = 0;
  encoderCountLeft = encoderCountRight = prevTick = 0;
  double SPEED_RAMP = 0;

  double multiplier = 2.50877193;
  double cm = PI * 5.7;

  targetTick = ((cm * 1.15)-1.5) * TICKS_PER_CM; // Caliberated to 30.25 ticks per cm

  while (encoderCountRight < min(TICKS_TO_RAMP, targetTick)) {
    SPEED_RAMP = modifiedMap(encoderCountRight, 0, TICKS_TO_RAMP, 50, SPEED_MOVE);
    pid = computeCurvePID(false);
    md.setSpeeds(SPEED_RAMP - pid, (SPEED_RAMP / multiplier) + pid);
  }
  while (encoderCountRight < targetTick ) {
    pid = computeCurvePID(false);
    md.setSpeeds(SPEED_MOVE - pid, (SPEED_MOVE / multiplier) + pid);
  }
  while (encoderCountRight < targetTick) {
    SPEED_RAMP = modifiedMap(encoderCountRight, targetTick - TICKS_TO_RAMP, targetTick, SPEED_MOVE, 50);
    pid = computeCurvePID(false);
    md.setSpeeds(SPEED_RAMP - pid, (SPEED_RAMP / multiplier) + pid);
  }

  md.setBrakes(400, 400);
  delay(DELAY_PER_MOVE);
}

void forward(double cm) {
  double pid;
  int targetTick;
  integral = 0;
  encoderCountLeft = encoderCountRight = prevTick = 0;
  double SPEED_RAMP = 0;

  targetTick = ((cm * 1.16)-1.6) * TICKS_PER_CM; // Caliberated to 30.25 ticks per cm

  if (!mode_calibration) {
    while (encoderCountLeft < min(TICKS_TO_RAMP, targetTick)) {
      SPEED_RAMP = modifiedMap(encoderCountLeft, 0, TICKS_TO_RAMP, 50, SPEED_MOVE);
      pid = computePID();
      md.setSpeeds((SPEED_RAMP * MOTOR_MULTIPLIER) - pid, SPEED_RAMP + pid);
//      Serial.print((SPEED_RAMP * MOTOR_MULTIPLIER));
//      Serial.print("\t");
//      Serial.println((SPEED_RAMP) + pid);
    }
    while (encoderCountLeft < targetTick  - TICKS_TO_RAMP) {
      pid = computePID();
      md.setSpeeds((SPEED_MOVE * MOTOR_MULTIPLIER) - pid, SPEED_MOVE + pid);
//      Serial.print((SPEED_MOVE * MOTOR_MULTIPLIER));
//      Serial.print("\t");
//      Serial.println((SPEED_MOVE) + pid);
    }
    while (encoderCountLeft < targetTick) {
      // if ((calibrateSensorValue(sensorFL.distance(), 1) <= WALL_GAP) || (calibrateSensorValue(sensorFC.distance(), 2) <= WALL_GAP) || (calibrateSensorValue(sensorFR.distance(), 3) <= WALL_GAP)) {
      //   break;
      // }
      SPEED_RAMP = modifiedMap(encoderCountLeft, targetTick - TICKS_TO_RAMP, targetTick, SPEED_MOVE, 50);
      pid = computePID();
      md.setSpeeds((SPEED_RAMP * MOTOR_MULTIPLIER) - pid, SPEED_RAMP + pid);
//      Serial.print((SPEED_RAMP * MOTOR_MULTIPLIER));
//      Serial.print("\t");
//      Serial.println((SPEED_RAMP) + pid);
    }
  }
  else {
    while (encoderCountLeft < targetTick) {
      pid = computePID();
      md.setSpeeds((SPEED_CALIBRATE * MOTOR_MULTIPLIER) - pid, SPEED_CALIBRATE + pid);
    }
  }

  md.setBrakes(400, 400);
  delay(DELAY_PER_MOVE);
}


void reverse(double cm) {
  double pid;
  int targetTick;
  integral = 0;
  encoderCountLeft = encoderCountRight = prevTick = 0;
  double SPEED_RAMP = 0;

  targetTick = ((cm * 1.1)-1) * TICKS_PER_CM; // Caliberated to 30.25 ticks per cm

  if (!mode_calibration) {
    while (encoderCountLeft < min(TICKS_TO_RAMP, targetTick)) {
      SPEED_RAMP = modifiedMap(encoderCountLeft, 0, TICKS_TO_RAMP, 50, SPEED_MOVE);
      pid = computePID();
      md.setSpeeds(-((SPEED_RAMP * MOTOR_MULTIPLIER) - pid), -(SPEED_RAMP + pid));
    }
    while (encoderCountLeft < targetTick - TICKS_TO_RAMP) {
      pid = computePID();
      md.setSpeeds(-((SPEED_MOVE * MOTOR_MULTIPLIER) - pid), -(SPEED_MOVE + pid));
    }
    while (encoderCountLeft < targetTick) {
      SPEED_RAMP = modifiedMap(encoderCountLeft, targetTick - TICKS_TO_RAMP, targetTick, SPEED_MOVE, 50);
      pid = computePID();
      md.setSpeeds(-((SPEED_RAMP * MOTOR_MULTIPLIER) - pid), -(SPEED_RAMP + pid));
    }
  }
  else {
    while (encoderCountLeft < targetTick) {
      pid = computePID();
      md.setSpeeds(-((SPEED_CALIBRATE * MOTOR_MULTIPLIER) - pid), -(SPEED_CALIBRATE + pid));
    }
  }

  md.setBrakes(400, 400);
  delay(DELAY_PER_MOVE);
}

void rotateRight(double deg) {
  double pid;
  int targetTick;
  integral = 0;
  encoderCountLeft = encoderCountRight = prevTick = 0;
  double SPEED_RAMP = 0;

  if (deg <= 90) targetTick = deg * TICKS_ROTATE_RIGHT; //4.523 4.375
  else if (deg <= 180 ) targetTick = deg * 4.62;
  else if (deg <= 360 ) targetTick = deg * 4.675;
  else targetTick = deg * 4.65;

  if (!mode_calibration) {
    while (encoderCountLeft < min(TICKS_TO_RAMP, targetTick)) {
      SPEED_RAMP = modifiedMap(encoderCountLeft, 0, TICKS_TO_RAMP, 50, SPEED_SPIN);
      pid = computePID();
      md.setSpeeds((SPEED_RAMP * MOTOR_MULTIPLIER) - pid, -(SPEED_RAMP + pid));
    }
    while (encoderCountLeft < targetTick - TICKS_TO_RAMP) {
      pid = computePID();
      md.setSpeeds((SPEED_SPIN * MOTOR_MULTIPLIER) - pid, -(SPEED_SPIN + pid));
    }
    while (encoderCountLeft < targetTick) {
      SPEED_RAMP = modifiedMap(encoderCountLeft, targetTick - TICKS_TO_RAMP, targetTick, SPEED_SPIN, 50);
      pid = computePID();
      md.setSpeeds((SPEED_RAMP * MOTOR_MULTIPLIER) - pid, -(SPEED_RAMP + pid));
    }
  }
  else {
    while (encoderCountLeft < targetTick) {
      pid = computePID();
      md.setSpeeds((SPEED_CALIBRATE * MOTOR_MULTIPLIER) - pid, -(SPEED_CALIBRATE + pid));
    }
  }

  md.setBrakes(400, 400);
  delay(DELAY_PER_MOVE);
}

void rotateLeft(double deg) {
  double pid;
  int targetTick;
  integral = 0;
  encoderCountLeft = encoderCountRight = prevTick = 0;
  double SPEED_RAMP = 0;

  if (deg <= 90) targetTick = deg * TICKS_ROTATE_LEFT; //4.424;
  else if (deg <= 180 ) targetTick = deg * 4.51;
  else if (deg <= 360 ) targetTick = deg * 4.51;
  else targetTick = deg * 4.65;

  if (!mode_calibration) {
    while (encoderCountLeft < min(TICKS_TO_RAMP, targetTick)) {
      SPEED_RAMP = modifiedMap(encoderCountLeft, 0, TICKS_TO_RAMP, 50, SPEED_SPIN);
      pid = computePID();
      md.setSpeeds(-((SPEED_RAMP * MOTOR_MULTIPLIER) - pid), (SPEED_RAMP + pid));
    }
    while (encoderCountLeft < targetTick - TICKS_TO_RAMP) {
      pid = computePID();
      md.setSpeeds(-((SPEED_SPIN * MOTOR_MULTIPLIER) - pid), (SPEED_SPIN + pid));
    }
    while (encoderCountLeft < targetTick) {
      SPEED_RAMP = modifiedMap(encoderCountLeft, targetTick - TICKS_TO_RAMP, targetTick, SPEED_SPIN, 50);
      pid = computePID();
      md.setSpeeds(-((SPEED_RAMP * MOTOR_MULTIPLIER) - pid), (SPEED_RAMP + pid));
    }
  }
  else {
    while (encoderCountLeft < targetTick) {
      pid = computePID();
      md.setSpeeds(-((SPEED_CALIBRATE * MOTOR_MULTIPLIER) - pid), (SPEED_CALIBRATE + pid));
    }
  }

  md.setBrakes(400, 400);
  delay(DELAY_PER_MOVE);
}

/**
 * ============================== For Sensors ==============================
 */
void readSensors() {
  int i;
  String output = "";
  
  double distFL = calibrateSensorValue(sensorFL.distance(), 1);
  double distFC = calibrateSensorValue(sensorFC.distance(), 2);
  double distFR = calibrateSensorValue(sensorFR.distance(), 3);
  double distRF = calibrateSensorValue(sensorRF.distance(), 4);
  double distRR = calibrateSensorValue(sensorRR.distance(), 5);
  double distL = calibrateSensorValue(sensorL.distance(), 0);

  // prepare to return obstacle position to algo
  int posFL = obstaclePosition(distFL, 1);
  int posFC = obstaclePosition(distFC, 1);
  int posFR = obstaclePosition(distFR, 1);
  int posRF = obstaclePosition(distRF, 2);
  int posRR = obstaclePosition(distRR, 2);
  int posL = obstaclePosition(distL, 0);

  // check for any values that are not satisfied
  for (i = 0; i < 10 ; i++) {
    if (posFL != -1) {
      break;
    }
    else {
      posFL = obstaclePosition(calibrateSensorValue(sensorFL.distance(), 1), 1);
    }
  }

  for (i = 0; i < 10 ; i++) {
    if (posFC != -1) {
      break;
    }
    else {
      posFC = obstaclePosition(calibrateSensorValue(sensorFC.distance(), 2), 1);
    }
  }


  for (i = 0; i < 10 ; i++) {
    if (posFR != -1) {
      break;
    }
    else {
      posFR = obstaclePosition(calibrateSensorValue(sensorFR.distance(), 3), 1);
    }
  }


  for (i = 0; i < 10 ; i++) {
    if (posRF != -1) {
      break;
    }
    else {
      posRF = obstaclePosition(calibrateSensorValue(sensorRF.distance(), 4), 2);
    }
  }


  for (i = 0; i < 10 ; i++) {
    if (posRR != -1) {
      break;
    }
    else {
      posRR = obstaclePosition(calibrateSensorValue(sensorRR.distance(), 5), 2);
    }
  }


  for (i = 0; i < 10 ; i++) {
    if (posL != -1) {
      break;
    }
    else {
      posL = obstaclePosition(calibrateSensorValue(sensorL.distance(), 0), 0);
    }
  }

  // ensure that there are no "-1" sensor values
  posFL = (posFL == -1) ? 0 : posFL;
  posFC = (posFC == -1) ? 0 : posFC;
  posFR = (posFR == -1) ? 0 : posFR;
  posRF = (posRF == -1) ? 0 : posRF;
  posRR = (posRR == -1) ? 0 : posRR;
  posL = (posL == -1) ? 0 : posL;

  // for checking how many obstacles are there on left
  if (!sensor_command) {
    if (forward_command) {
      obstacle_left_rear = obstacle_left_center;
      obstacle_left_center = (posL == 1) ? true : false;
      obstacle_right_rear = (posRR == 1) ? true : false;
    }
    else {
      obstacle_left_center = false;
      obstacle_left_rear = false;
      obstacle_right_rear = false;
    }
  }

  // concatenate all position into a string and send
  output += String(posFL);
  output += String(posFC);
  output += String(posFR);
  output += String(posRF);
  output += String(posRR);
  output += String(posL);
  
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

      if ((i == 0) && (n == 0)) {
        return modifiedMap(dist, a, arr[i], 0, ((i + offset + 1) * 10));
      }

      return modifiedMap(dist, a, arr[i], ((i + offset) * 10), ((i + offset + 1) * 10));
    }
  }
  
  return i*10;
}

int obstaclePosition(double val, int shortrange){
  /*
    values for shortrange
    0 = left side
    1 = front 
    2 = right side
  */

  int tmp = 0;

  int modulo = ((int) (val + 0.5)) % 10;
  
  if ((modulo != 7) && (modulo != 8) && (modulo != 9) && (modulo != 0) && (modulo != 1) && (modulo != 2) && (modulo != 3)) {
    return -1;
  }
  else if (shortrange == 1) {
    tmp = (val + 4) / 10;
    if ((tmp >= 1) && (tmp <= RANGE_OF_FRONT_SENSOR)) {
      return tmp;
    }
    else {
      return 0; 
    }    
  }
  else if (shortrange == 2) {
    tmp = (val + 4) / 10;
    if ((tmp >= 1) && (tmp <= RANGE_OF_RIGHT_SENSOR)) {
      return tmp;
    }
    else {
      return 0; 
    }    
  }
  else {
    tmp = (val - 6) / 10;
    if ((tmp >= 1) && (tmp <= RANGE_OF_LEFT_SENSOR)) {
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
void autoCalibrate(int force_calibrate) {
  digitalWrite(pinGreenLED, HIGH);
  digitalWrite(pinRedLED, HIGH);

  bool to_calibrate = false;
  double distFL = calibrateSensorValue(sensorFL.distance(), 1);
  double distFC = calibrateSensorValue(sensorFC.distance(), 2);
  double distFR = calibrateSensorValue(sensorFR.distance(), 3);
  double distRF = calibrateSensorValue(sensorRF.distance(), 4);
  double distRR = calibrateSensorValue(sensorRR.distance(), 5);
  double distL = calibrateSensorValue(sensorL.distance(), 0);

 // Serial.println(String(distFL) + ", " + String(distFC) + ", " + String(distFR) + ", " + String(distRF) + ", " + String(distRR) + ", " + String(distL));
  int calibrate_front = 0;
  int calibrate_right = 0;

  if (force_calibrate == 0) {
    bool to_calibrate1,to_calibrate2,to_calibrate3,to_calibrate4,to_calibrate5,to_calibrate6;
    to_calibrate1 = ((distFL <= WALL_GAP + 4) && ((distFL <= (WALL_GAP - 2)) || (distFL >= (WALL_GAP + 2)))) ? true : false;
    to_calibrate2 = ((distFC <= WALL_GAP + 4) && ((distFC <= (WALL_GAP - 2)) || (distFC >= (WALL_GAP + 2)))) ? true : false;
    to_calibrate3 = ((distFR <= WALL_GAP + 4) && ((distFR <= (WALL_GAP - 2)) || (distFR >= (WALL_GAP + 2)))) ? true : false;
    to_calibrate4 = ((distRF <= WALL_GAP + 4) && ((distRF <= (WALL_GAP - 2)) || (distRF >= (WALL_GAP + 2)))) ? true : false;
    to_calibrate5 = ((distRR <= WALL_GAP + 4) && ((distRR <= (WALL_GAP - 2)) || (distRR >= (WALL_GAP + 2)))) ? true : false;
    to_calibrate6 = ((distL <= (2 * WALL_GAP) + 4) && ((distL <= ((2 * WALL_GAP) - 2)) || (distL >= ((2 * WALL_GAP) + 2)))) ? true : false;
    to_calibrate = (to_calibrate1 || to_calibrate2 || to_calibrate3 || to_calibrate4 || to_calibrate5 || to_calibrate6);
    force_calibrate++;
  }
  else if ((force_calibrate == 1) || (force_calibrate == 2)) {
    to_calibrate = true;
  }
  else {
    return;
  }

  force_calibrate++;

  // check for best opportunity to calibrate
  if (step_best_calibrate >= STEPS_TO_BEST_CALIBRATE) {
    if (((abs(distFL - distFR) < 5) && ((distFL + distFR) <= (3 * WALL_GAP))) || ((abs(distFL - distFC) < 5) && ((distFL + distFC) <= (3 * WALL_GAP))) || ((abs(distFC - distFR) < 5) && ((distFC + distFR) <= (3 * WALL_GAP)))) {
      if ((distRF <= (WALL_GAP + 4)) || (distRR <= (WALL_GAP + 4)) || (distL <= ((2 * WALL_GAP) + 4))) {
        step_counter = STEPS_TO_CALIBRATE;
        step_best_calibrate = 0;
        to_calibrate = true;
      }
    }
    else if ((abs(distRF - distRR) < 5) && ((distRF + distRR) <= (3 * WALL_GAP))) {
      if ((distFL <= (WALL_GAP + 4)) || (distFC <= (WALL_GAP + 4)) || (distFR <= (WALL_GAP + 4))) {
        step_counter = STEPS_TO_CALIBRATE;
        step_best_calibrate = 0;
        to_calibrate = true;
      }
    }
  }

  // calibrate if above steps count
  // if (step_counter >= STEPS_TO_CALIBRATE) {
  // Serial.print("To caliberate: ");
  // Serial.print(to_calibrate);
  // Serial.print(", Left: ");
  // Serial.print(opportunity_calibrate_left);
  // Serial.println();
  if (to_calibrate) {
    if (opportunity_calibrate_left) {
      rotateLeft(90);
      calibrateWithFront();
      rotateRight(90);

      distFL = calibrateSensorValue(sensorFL.distance(), 1);

      if ((distFL <= WALL_GAP + 4) && ((distFL <= (WALL_GAP - 2)) || (distFL >= (WALL_GAP + 2)))) {
        calibrateDistance(sensorFL, 1);
      }
      else {
        distFC = calibrateSensorValue(sensorFC.distance(), 2);

        if ((distFC <= WALL_GAP + 4) && ((distFC <= (WALL_GAP - 2)) || (distFC >= (WALL_GAP + 2)))) {
          calibrateDistance(sensorFC, 2);
        }
        else {
          distFR = calibrateSensorValue(sensorFR.distance(), 3);

          if ((distFR <= WALL_GAP + 4) && ((distFR <= (WALL_GAP - 2)) || (distFR >= (WALL_GAP + 2)))) {
            calibrateDistance(sensorFR, 3);
          }
        }
      }
    }
    else {
      if ((abs(distFL - distFR) < 5) && ((distFL + distFR) <= (3 * WALL_GAP))) {
        // check if target side can calibrate angle
        calibrateAngle(sensorFL, 1, sensorFR, 3, 17);
        calibrateDistance(sensorFL, 1);
        calibrate_front = 1;
      }
      else if ((abs(distFL - distFC) < 5) && ((distFL + distFC) <= (3 * WALL_GAP))) {
        calibrateAngle(sensorFL, 1, sensorFC, 2, 9);
        calibrateDistance(sensorFC, 2);
        calibrate_front = 2;
      }
      else if ((abs(distFC - distFR) < 5) && ((distFC + distFR) <= (3 * WALL_GAP))) {
        calibrateAngle(sensorFC, 2, sensorFR, 3, 9);
        calibrateDistance(sensorFC, 2);
        calibrate_front = 3;
      }
      // check for 1 obstacle on other side
      // if yes, calibrate dist on other side
      if (calibrate_front > 0) {
        step_counter = 0;

        distRF = calibrateSensorValue(sensorRF.distance(), 4);
        
        if ((distRF <= (WALL_GAP + 4)) && (distRF >= (WALL_GAP + 2)) && (distRF <= (WALL_GAP - 2))) {
          rotateRight(90);
          calibrateDistance(sensorFL, 1);
          rotateLeft(90);
          calibrate_right = 1;
        }
        else {
          distRR = calibrateSensorValue(sensorRR.distance(), 5);

          if ((distRR <= (WALL_GAP + 4)) && (distRR >= (WALL_GAP + 2)) && (distRR <= (WALL_GAP - 2))) {
            rotateRight(90);
            calibrateDistance(sensorFC, 2);
            rotateLeft(90);
            calibrate_right = 1;
          }
          else {
            distL = calibrateSensorValue(sensorL.distance(), 0);

            if ((distL <= ((WALL_GAP * 2) + 4)) && (distL >= (WALL_GAP + 2)) && (distL <= (WALL_GAP - 2))) {
              rotateLeft(90);
              calibrateDistance(sensorFR, 3);
              rotateRight(90);
              calibrate_right = 1;
            }
          }
        }

        // calibrate the angle after turning back
        if (calibrate_right == 1) {
          switch (calibrate_front) {
            case 1: calibrateAngle(sensorFL, 1, sensorFR, 3, 17); break;
            case 2: calibrateAngle(sensorFL, 1, sensorFC, 2, 9); break;
            case 3: calibrateAngle(sensorFC, 2, sensorFR, 3, 9); break;
            default: break;
          }
        }
      }
      else if (((abs(distRF - distRR) < 5) && ((distRF + distRR) <= (3 * WALL_GAP))) || opportunity_calibrate_right || (forward_command && obstacle_right_rear && ((distRF <= (WALL_GAP + 4)) || (distRR <= (WALL_GAP + 4))))) {
        // check for right wall and calibrate
        step_counter = 0;

        // check to see if there is enough clearance from wall
        if (distFL <= (WALL_GAP + 4)) {
          calibrateDistance(sensorFL, 1);
        }
        else if (distFC <= (WALL_GAP + 4)) {
          calibrateDistance(sensorFC, 2);
        }
        else if (distFR <= (WALL_GAP + 4)) {
          calibrateDistance(sensorFR, 3);
        }

        if ((abs(distRF - distRR) < 5) && ((distRF + distRR) <= (3 * WALL_GAP))) {
          if (((distRF + distRR) < ((2 * WALL_GAP) - 4)) || ((distRF + distRR) > ((2 * WALL_GAP) + 4))) {
            // calibrate to right
            rotateRight(90);
            calibrateWithFront();
            rotateLeft(90);
          }
          else {
            calibrateAngle(sensorRF, 4, sensorRR, 5, 9);
          }
        }
        else {
          rotateRight(90);
          calibrateWithFront();
          rotateLeft(90);
        }
        

        // check for front obstacles to calibrate with
        distFL = calibrateSensorValue(sensorFL.distance(), 1);
        if (distFL <= (WALL_GAP + 4)) {
          calibrateDistance(sensorFL, 1);
          calibrate_front = 1;
        }
        else {
          distFC = calibrateSensorValue(sensorFC.distance(), 2);
          if (distFC <= (WALL_GAP + 4)) {
            calibrateDistance(sensorFC, 2);
            calibrate_front = 2;
          }
          else {
            distFR = calibrateSensorValue(sensorFR.distance(), 3);
            if (distFR <= (WALL_GAP + 4)) {
              calibrateDistance(sensorFR, 3);
              calibrate_front = 3;
            }
          }
        }
      }
      else {
        // rotateLeft(90);
        // calibrateWithFront();
        // rotateRight(90);

        // distFL = calibrateSensorValue(sensorFL.distance(), 1);

        // if ((distFL <= WALL_GAP + 4) && ((distFL <= (WALL_GAP - 2)) || (distFL >= (WALL_GAP + 2)))) {
        //   calibrateDistance(sensorFL, 1);
        // }
        // else {
        //   distFC = calibrateSensorValue(sensorFC.distance(), 2);

        //   if ((distFC <= WALL_GAP + 4) && ((distFC <= (WALL_GAP - 2)) || (distFC >= (WALL_GAP + 2)))) {
        //     calibrateDistance(sensorFC, 2);
        //   }
        //   else {
        //     distFR = calibrateSensorValue(sensorFR.distance(), 3);

        //     if ((distFR <= WALL_GAP + 4) && ((distFR <= (WALL_GAP - 2)) || (distFR >= (WALL_GAP + 2)))) {
        //       calibrateDistance(sensorFR, 3);
        //     }
        //   }
        // }
      }
    }
  }
  else if ((abs(distRF - distRR) < 5) && ((distRF + distRR) <= (3 * WALL_GAP))) {
    calibrateAngle(sensorRF, 4, sensorRR, 5, 9);
    
    if (((distRF + distRR) < ((2 * WALL_GAP) - 4)) || ((distRF + distRR) > ((2 * WALL_GAP) + 4))) {
      rotateRight(90);
      calibrateDistance(sensorFL, 1);
      rotateLeft(90);
      calibrateAngle(sensorRF, 4, sensorRR, 5, 9);
    }
  }
  else if (distL <= ((WALL_GAP * 2) + 4)) {
    if ((distL <= ((WALL_GAP * 2) - 2)) || (distL >= ((WALL_GAP * 2) + 2))) {
      step_counter = STEPS_TO_CALIBRATE;

      rotateLeft(90);
      calibrateWithFront();
      rotateRight(90);

      distFL = calibrateSensorValue(sensorFL.distance(), 1);

      if ((distFL <= WALL_GAP + 4) && ((distFL <= (WALL_GAP - 2)) || (distFL >= (WALL_GAP + 2)))) {
        calibrateDistance(sensorFL, 1);
      }
      else {
        distFC = calibrateSensorValue(sensorFC.distance(), 2);

        if ((distFC <= WALL_GAP + 4) && ((distFC <= (WALL_GAP - 2)) || (distFC >= (WALL_GAP + 2)))) {
          calibrateDistance(sensorFC, 2);
        }
        else {
          distFR = calibrateSensorValue(sensorFR.distance(), 3);

          if ((distFR <= WALL_GAP + 4) && ((distFR <= (WALL_GAP - 2)) || (distFR >= (WALL_GAP + 2)))) {
            calibrateDistance(sensorFR, 3);
          }
        }
      }
    }
  }

  distFL = calibrateSensorValue(sensorFL.distance(), 1);
  distFC = calibrateSensorValue(sensorFC.distance(), 2);
  distFR = calibrateSensorValue(sensorFR.distance(), 3);
  distRF = calibrateSensorValue(sensorRF.distance(), 4);
  distRR = calibrateSensorValue(sensorRR.distance(), 5);
  distL = calibrateSensorValue(sensorL.distance(), 0);

  // if too close to one obstacle (calibrate to that one obstacle)
  if (distFL <= WALL_GAP - 2) {
    if (!calibrateWithFront()) {
      calibrateDistance(sensorFL, 1);
    }
  }

  if (distFC <= WALL_GAP - 2) {
    if (!calibrateWithFront()) {
      calibrateDistance(sensorFC, 2);
    }
  }

  if (distFR <= WALL_GAP - 2) {
    if (!calibrateWithFront()) {
      calibrateDistance(sensorFR, 3);
    }
  }

  if (distRF <= WALL_GAP - 2) {
    rotateRight(90);
    if (!calibrateWithFront()) {
      calibrateDistance(sensorFL, 1);
    }
    rotateLeft(90);
  }

  if (distRR <= WALL_GAP - 2) {
    rotateRight(90);
    if (!calibrateWithFront()) {
      calibrateDistance(sensorFC, 2);
    }
    rotateLeft(90);
  }

  if (distL <= ((WALL_GAP * 2) - 2)) {
    rotateLeft(90);
    if (!calibrateWithFront()) {
      calibrateDistance(sensorFR, 3);
    }
    rotateRight(90);
  }
}

bool calibrateWithLeft() {
  bool output = false;

  rotateLeft(90);
  output = calibrateWithFront();
  rotateRight(90);

  return output; // return whether calibrated angle
}

bool calibrateWithFront() {
  bool output = false;

  double distL = calibrateSensorValue(sensorFL.distance(), 1);
  double distC = calibrateSensorValue(sensorFC.distance(), 2);
  double distR = calibrateSensorValue(sensorFR.distance(), 3);

  if ((abs(distL - distR) < 5) && ((distL + distR) <= (3 * WALL_GAP))) {
    calibrateAngle(sensorFL, 1, sensorFR, 3, 17);
    calibrateDistance(sensorFL, 1);
    output = true;
  }
  else if ((abs(distL - distC) < 5) && ((distL + distC) <= (3 * WALL_GAP))) {
    calibrateAngle(sensorFL, 1, sensorFC, 2, 9);
    calibrateDistance(sensorFC, 2);
    output = true;
  }
  else if ((abs(distC - distR) < 5) && ((distC + distR) <= (3 * WALL_GAP))) {
    calibrateAngle(sensorFC, 2, sensorFR, 3, 9);
    calibrateDistance(sensorFC, 2);
    output = true;
  }

  // if (distFL <= (WALL_GAP + 4)) {
  //   calibrateDistance(sensorFL, 1);
  // }
  // else if (distFC <= (WALL_GAP + 4)) {
  //   calibrateDistance(sensorFC, 2);
  // }
  // else if (distFR <= (WALL_GAP + 4)) {
  //   calibrateDistance(sensorFR, 3);
  // }

  return output; // return whether calibrated angle
}

bool calibrateWithRight() {
  bool output = false;

  double distL = calibrateSensorValue(sensorRF.distance(), 4);
  double distR = calibrateSensorValue(sensorRR.distance(), 5);

  if ((abs(distL - distR) < 5) && ((distL + distR) <= (3 * WALL_GAP))) {
    if (((distL + distR) < ((2 * WALL_GAP) - 1)) || ((distL + distR) > ((2 * WALL_GAP) + 1))) {
      rotateRight(90);
      calibrateWithFront();
      rotateLeft(90);
    }
    else {
      calibrateAngle(sensorRF, 4, sensorRR, 5, 9);
    }
    output = true;
  }
  else {
    rotateRight(90);
    output = calibrateWithFront();
    rotateLeft(90);
  }

  // TODO: implement check and calibrate with front

  return output; // return whether calibrated angle
}

void calibrateAngle(SharpIR sensorL, int arrL, SharpIR sensorR, int arrR, int dist) {
  Serial.println("Hello");
  mode_calibration = true;
  double distL = calibrateSensorValue(sensorL.distance(), arrL);
  double distR = calibrateSensorValue(sensorR.distance(), arrR);
  double diff = abs(distL - distR);
  int i = 0;

  double angle = 0;

//  angle = (atan(diff/dist) * (180/3.14159265));
  // angle = (diff > 0.5) ? angle : angle/2;

//  while (angle > 30) {
//    if (distL > distR){
//      rotateRight(10);
//    }
//    else if (distR > distL){   
//      rotateLeft(10);
//    }
//
//    distL = calibrateSensorValue(sensorL.distance(), arrL);
//    distR = calibrateSensorValue(sensorR.distance(), arrR);
//    diff = abs(distL - distR);
//
//    angle = (atan(diff/dist) * (180/3.14159265));
//    // angle = (diff > 0.5) ? angle : angle/2;
//  }
//
//  if (distL > distR){
//    rotateRight(angle);
//  }
//  else if (distR > distL){   
//    rotateLeft(angle);
//  }

  for (i = 0; i < 9; i++){
    if (diff < 0.25) {
      break;
    }
    // Serial.println("dist: " + String(distL) + ", " + String(distR) + ", " + String(diff) + ", " + String(diff));
    angle = (atan(diff/dist) * (180/3.14159265));
//    angle = (diff > 0.5) ? angle : angle/2;
    angle = angle/2;
    // Serial.println("angle: " + String(angle));
    if (distL > distR){
      rotateRight(angle);
    }
    else if (distR > distL){   
      rotateLeft(angle);
    }
    delay(20);
    distL = calibrateSensorValue(sensorL.distance(), arrL);
    distR = calibrateSensorValue(sensorR.distance(), arrR);
    diff = abs(distL - distR);
  }
  // Serial.println(diff);
  mode_calibration = false;
}

void calibrateDistance(SharpIR sensor, int arr){
  mode_calibration = true;
  double dist = calibrateSensorValue(sensor.distance(), arr);

  if (dist < WALL_GAP) {
    reverse(WALL_GAP - dist);
  }
  else if (dist > WALL_GAP) {
    forward(dist - WALL_GAP);
  }
  mode_calibration = false;
}

