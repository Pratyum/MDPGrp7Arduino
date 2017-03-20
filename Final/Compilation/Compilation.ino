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

#define SPEED_MOVE 250
#define SPEED_SPIN 250  
#define SPEED_CALIBRATE 75
#define MOTOR_MULTIPLIER 1

#define TICKS_PER_CM 27.2
#define TICKS_TO_RAMP 100


#define STEPS_TO_CALIBRATE 5
#define STEPS_TO_BEST_CALIBRATE 3

#define DELAY_PER_MOVE 90

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
// values on 17 mar part 2
double arrMapping0[] = {18.63, 23.71, 31.86, 41.45, 50.80, 64.23, 72.41, 89.02, 131.06};
double arrMapping1[] = {10.16, 21.17, 34.91, 55.04, 71.14};
double arrMapping2[] = {9.82, 20.50, 32.61, 44.60, 51};
double arrMapping3[] = {10.54, 23.36, 38.32, 55.04, 64.60};
double arrMapping4[] = {10.25, 21.33, 33.25, 46.59, 62.69};
double arrMapping5[] = {9.86, 20.99, 33.25, 45.41, 61.24};

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

bool mode_fastest_path, mode_calibration, forward_command, sensor_command;

void setup() {
  Serial.begin(9600);

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

      if ((ch == 'z') || (ch == 'Z') || (ch == 'x') || (ch == 'X') || (ch == 'c') || (ch == 'C')) {
        switch(ch) {
          case 'z': case 'Z':
            opportunity_calibrate_left = true;
            break;
          case 'x': case 'X':
            opportunity_calibrate_front = true;
            break;
          case 'c': case 'C':
            opportunity_calibrate_right = true;
            break;
        }
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

  switch (command) {
    case 'M': case 'm': // mode
      flag = false;
      if (val == 1) {
        mode_fastest_path = true;
      }
      else if (val == 0) {
        mode_fastest_path = false;
      }
      break;
    case 'F': case 'f': // forward
      (val == 0) ? forward(10) : forward(val * 10);
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
    case 'S': case 's': // readSensors
      flag = false;
      sensor_command = true;
      autoCalibrate(0);
      readSensors();
      break;
    case 'C': case 'c': // calibrate to right wall
      flag = false;
      step_counter = (calibrateWithRight()) ? 0 : step_counter;
      break;
    case 'X': case 'x': 
      flag = false;
      step_counter = (calibrateWithFront()) ? 0 : step_counter;
      break;
    case 'Z': case 'z':
      flag = false; 
      step_counter = (calibrateWithLeft()) ? 0 : step_counter;
      break;
    case 'E': case 'e':
      flag = false;
      (autoCalibrate(1));
      break;
    default: 
      flag = false;
      Serial.println("E");
  }
  
  if (flag) {
    // check if command was action, calibrate if yes
    autoCalibrate(0);
  }

  if (!Serial.available()) {
    if (flag) {
      // TODO: read sensors values and store it so that we dont need to read again for calibration
      readSensors();
    }
    digitalWrite(pinGreenLED, HIGH);
    digitalWrite(pinRedLED, LOW);
  }
  
  if (mode_fastest_path) {
//    autoCalibrate();
    if (!Serial.available()) {
      calibrateDistance(sensorFC, 2);
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
  
  kp = 18; // 10 trial and error
  ki = 0.005; // 0.005
  kd = 0.025; // 0.025

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
  double SPEED_RAMP = 0;

  targetTick = ((cm * 1.1)-1) * TICKS_PER_CM; // Caliberated to 30.25 ticks per cm

  if (!mode_calibration) {
    // check if forward is valid
//    double distFL = calibrateSensorValue(sensorFL.distance(), 1);
//    double distFC = calibrateSensorValue(sensorFC.distance(), 2);
//    double distFR = calibrateSensorValue(sensorFR.distance(), 3);
//  
//    if (distFL <= 14) {
//      mode = 1;
//      calibrateDistance(sensorFL, 1);
//      mode = 0;
//      return false;
//    }
//    else if (distFC <= 14) {
//      mode = 1;
//      calibrateDistance(sensorFC, 2);
//      mode = 0;
//      return false;
//    }
//    else if (distFR <= 14) {
//      mode = 1;
//      calibrateDistance(sensorFR, 3);
//      mode = 0;
//      return false;
//    }

    while (encoderCountLeft < min(TICKS_TO_RAMP, targetTick)) {
      SPEED_RAMP = modifiedMap(encoderCountLeft, 0, TICKS_TO_RAMP, 50, SPEED_MOVE);
      pid = computePID();
      md.setSpeeds((SPEED_RAMP * MOTOR_MULTIPLIER) - pid, (SPEED_RAMP) + pid);
    }
    while (encoderCountLeft < targetTick  - TICKS_TO_RAMP) {
      pid = computePID();
      md.setSpeeds((SPEED_MOVE * MOTOR_MULTIPLIER) - pid, SPEED_MOVE + pid);
    }
    while (encoderCountLeft < targetTick) {
      SPEED_RAMP = modifiedMap(encoderCountLeft, targetTick - TICKS_TO_RAMP, targetTick, SPEED_MOVE, 50);
      pid = computePID();
      md.setSpeeds((SPEED_RAMP * MOTOR_MULTIPLIER) - pid, (SPEED_RAMP) + pid);
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

  targetTick = ((cm * 1.07)-0.7) * TICKS_PER_CM; // Caliberated to 30.25 ticks per cm

  if (!mode_calibration) {
    while (encoderCountLeft < min(TICKS_TO_RAMP, targetTick)) {
      SPEED_RAMP = modifiedMap(encoderCountLeft, 0, TICKS_TO_RAMP, 50, SPEED_MOVE);
      pid = computePID();
      md.setSpeeds(-((SPEED_RAMP * MOTOR_MULTIPLIER) - pid), -((SPEED_RAMP) + pid));
    }
    while (encoderCountLeft < targetTick - TICKS_TO_RAMP) {
      pid = computePID();
      md.setSpeeds(-((SPEED_MOVE * MOTOR_MULTIPLIER) - pid), -(SPEED_MOVE + pid));
    }
    while (encoderCountLeft < targetTick) {
      SPEED_RAMP = modifiedMap(encoderCountLeft, targetTick - TICKS_TO_RAMP, targetTick, SPEED_MOVE, 50);
      pid = computePID();
      md.setSpeeds(-((SPEED_RAMP * MOTOR_MULTIPLIER) - pid), -((SPEED_RAMP) + pid));
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

  if (deg <= 90) targetTick = deg * 4.24; //4.523 4.375
  else if (deg <= 180 ) targetTick = deg * 4.62;
  else if (deg <= 360 ) targetTick = deg * 4.675;
  else targetTick = deg * 4.65;

  if (!mode_calibration) {
    while (encoderCountLeft < min(TICKS_TO_RAMP, targetTick)) {
      SPEED_RAMP = modifiedMap(encoderCountLeft, 0, TICKS_TO_RAMP, 50, SPEED_SPIN);
      pid = computePID();
      md.setSpeeds((SPEED_RAMP * MOTOR_MULTIPLIER) - pid, -((SPEED_RAMP) + pid));
    }
    while (encoderCountLeft < targetTick - TICKS_TO_RAMP) {
      pid = computePID();
      md.setSpeeds((SPEED_SPIN * MOTOR_MULTIPLIER) - pid, -(SPEED_SPIN + pid));
    }
    while (encoderCountLeft < targetTick) {
      SPEED_RAMP = modifiedMap(encoderCountLeft, targetTick - TICKS_TO_RAMP, targetTick, SPEED_SPIN, 50);
      pid = computePID();
      md.setSpeeds((SPEED_RAMP * MOTOR_MULTIPLIER) - pid, -((SPEED_RAMP) + pid));
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

  if (deg <= 90) targetTick = deg * 4.27; //4.424;
  else if (deg <= 180 ) targetTick = deg * 4.51;
  else if (deg <= 360 ) targetTick = deg * 4.51;
  else targetTick = deg * 4.65;

  if (!mode_calibration) {
    while (encoderCountLeft < min(TICKS_TO_RAMP, targetTick)) {
      SPEED_RAMP = modifiedMap(encoderCountLeft, 0, TICKS_TO_RAMP, 50, SPEED_SPIN);
      pid = computePID();
      md.setSpeeds(-((SPEED_RAMP * MOTOR_MULTIPLIER) - pid), ((SPEED_RAMP) + pid));
    }
    while (encoderCountLeft < targetTick - TICKS_TO_RAMP) {
      pid = computePID();
      md.setSpeeds(-((SPEED_SPIN * MOTOR_MULTIPLIER) - pid), (SPEED_SPIN + pid));
    }
    while (encoderCountLeft < targetTick) {
      SPEED_RAMP = modifiedMap(encoderCountLeft, targetTick - TICKS_TO_RAMP, targetTick, SPEED_SPIN, 50);
      pid = computePID();
      md.setSpeeds(-((SPEED_RAMP * MOTOR_MULTIPLIER) - pid), ((SPEED_RAMP) + pid));
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
  int posRF = obstaclePosition(distRF, 1);
  int posRR = obstaclePosition(distRR, 1);
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
      posRF = obstaclePosition(calibrateSensorValue(sensorRF.distance(), 4), 1);
    }
  }


  for (i = 0; i < 10 ; i++) {
    if (posRR != -1) {
      break;
    }
    else {
      posRR = obstaclePosition(calibrateSensorValue(sensorRR.distance(), 5), 1);
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

      return modifiedMap(dist, a, arr[i], ((i + offset) * 10), ((i + offset + 1) * 10));
    }
  }
  return i*10;
}

int obstaclePosition(double val, int shortrange){
  int tmp = 0;

  int modulo = ((int) (val + 0.5)) % 10;
  
  if ((modulo != 7) && (modulo != 8) && (modulo != 9) && (modulo != 0) && (modulo != 1) && (modulo != 2) && (modulo != 3)) {
    return -1;
  }
  else if (shortrange == 1) {
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
    if ((tmp >= 1) && (tmp <= 4)) {
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

//  Serial.println(String(distFL) + ", " + String(distFC) + ", " + String(distFR) + ", " + String(distRF) + ", " + String(distRR) + ", " + String(distL));
  int calibrate_front = 0;
  int calibrate_right = 0;

  if (force_calibrate == 0) {
    to_calibrate = ((distFL <= WALL_GAP + 4) && ((distFL <= (WALL_GAP - 3)) || (distFL >= (WALL_GAP + 3)))) ? true : false;
    to_calibrate = ((distFC <= WALL_GAP + 4) && ((distFC <= (WALL_GAP - 3)) || (distFC >= (WALL_GAP + 3)))) ? true : false;
    to_calibrate = ((distFR <= WALL_GAP + 4) && ((distFR <= (WALL_GAP - 3)) || (distFR >= (WALL_GAP + 3)))) ? true : false;
    to_calibrate = ((distRF <= WALL_GAP + 4) && ((distRF <= (WALL_GAP - 3)) || (distRF >= (WALL_GAP + 3)))) ? true : false;
    to_calibrate = ((distRR <= WALL_GAP + 4) && ((distRR <= (WALL_GAP - 3)) || (distRR >= (WALL_GAP + 3)))) ? true : false;
    to_calibrate = ((distL <= (2 * WALL_GAP) + 4) && ((distL <= ((2 * WALL_GAP) - 3)) || (distL >= ((2 * WALL_GAP) + 3)))) ? true : false;
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
  if (to_calibrate) {
    // check if target side can calibrate angle
    if ((abs(distFL - distFR) < 5) && ((distFL + distFR) <= (3 * WALL_GAP))) {
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
    else if (((abs(distRF - distRR) < 5) && ((distRF + distRR) <= (3 * WALL_GAP))) || opportunity_calibrate_right) {
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
        if (((distRF + distRR) < ((2 * WALL_GAP) - 6)) || ((distRF + distRR) > ((2 * WALL_GAP) + 6))) {
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
    else if (forward_command || opportunity_calibrate_left) {
      if (opportunity_calibrate_left) {
        step_counter = 0;

        rotateLeft(90);
        autoCalibrate(force_calibrate);
        rotateRight(90);
      }
      else if (distL <= ((WALL_GAP * 2) - 3)) {
        if (obstacle_left_center || obstacle_left_rear) {
          step_counter = 0;

          rotateLeft(90);
          autoCalibrate(force_calibrate);
          rotateRight(90);
        }
      }
      else if (obstacle_left_center && obstacle_left_rear) {
        step_counter = 0;

        rotateLeft(90);
        autoCalibrate(force_calibrate);
        rotateRight(90);
      }
    }
    else {
      rotateLeft(90);
      autoCalibrate(force_calibrate);
      rotateRight(90);
    }
  }
  else {
    if ((abs(distRF - distRR) < 5) && ((distRF + distRR) <= (3 * WALL_GAP))) {
      calibrateAngle(sensorRF, 4, sensorRR, 5, 9);
      
      if (((distRF + distRR) < ((2 * WALL_GAP) - 6)) || ((distRF + distRR) > ((2 * WALL_GAP) + 6))) {
        rotateRight(90);
        calibrateDistance(sensorFL, 1);
        rotateLeft(90);
        calibrateAngle(sensorRF, 4, sensorRR, 5, 9);
      }
    }
    else if (distL <= ((WALL_GAP * 2) - 2)) {
      step_counter = STEPS_TO_CALIBRATE;

      rotateLeft(90);
      autoCalibrate(force_calibrate);
      rotateRight(90);
    }
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
    angle = (diff > 0.5) ? angle : angle/2;
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

