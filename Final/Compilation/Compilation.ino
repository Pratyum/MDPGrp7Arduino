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

#define SPEED_MOVE 300
#define SPEED_SPIN 300
#define SPEED_CALIBRATE 100
#define MOTOR_MULTIPLIER 0.90

#define STEPS_TO_CALIBRATE 5

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
// values on 10 mar
double arrMapping0[] = {18.25, 23.24, 30.72, 41.27, 50.80, 61.66, 72.41, 89.02, 131.06};
double arrMapping1[] = {9.63, 20.40, 32.39, 47.02, 71.14};
double arrMapping2[] = {9.00, 19.87, 30.54, 41.15, 51};
double arrMapping3[] = {9.55, 20.00, 30.64, 42.90, 64.60};
double arrMapping4[] = {9.33, 20.40, 30.94, 37.72, 62.69};
double arrMapping5[] = {9.97, 20.40, 30.14, 34.45, 49.80};

/**
 * ============================== Initiate global variables ==============================
 * initiate global variables that will be used
 */
volatile long encoderCountLeft, encoderCountRight;
long prevTick;
double integral;

int mode, step_counter;

bool calibrateAutoChecked;

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
  calibrateAutoChecked = true;
}

void loop() {
  int i = 0, j = 1;
  double val;
  String valString = "";
  char commandBuffer[20];
  char command, ch;
  bool flag = true;

  mode = 0;

  while (1){
    if (Serial.available()) {
      ch = Serial.read();
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
    case 'F': case 'f': // forward
      flag = (val == 0) ? forward(10) : forward(val * 10);
      if (!flag) {
        Serial.println("E");
      }
      step_counter++;
      delay(100);
      break;
    case 'B': case 'b': // reverse
      (val == 0) ? reverse(10) : reverse(val * 10);
      step_counter++;
      delay(100);
      break;
    case 'L': case 'l': // rotateLeft
      (val == 0) ? rotateLeft(90) : rotateLeft(val);
      step_counter++;
      delay(100);
      break;
    case 'R': case 'r': // rotateRight
      (val == 0) ? rotateRight(90) : rotateRight(val);
      step_counter++;
      delay(100);
      break;
    case 'S': case 's': // readSensors
      flag = false;
      readSensors();
      // step_counter++;  
      break;
    case 'C': case 'c': // calibrate to right wall
      mode = 1;
      calibrateWithRight();
      // step_counter++;  
      break;
    case 'X': case 'x': case 'Y': case 'y': case 'Z': case 'z': // calibrate to front wall (sensor 1, 3)
      mode = 1;
      calibrateWithFront();
      // step_counter++;  
      break;
//    case 'Y': case 'y': // calibrate to front wall (sensor 1, 2)
//      mode = 1;
////      calibrateWithFront(1);
//      // step_counter++;  
//      break;
//    case 'Z': case 'z': // calibrate to front wall (sensor 2, 3)
//      mode = 1;
////      calibrateWithFront(2);
//      // step_counter++;  
//      break;
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
    digitalWrite(pinGreenLED, HIGH);
    digitalWrite(pinRedLED, LOW);
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

bool forward(double cm) {
  double pid;
  int targetTick;
  integral = 0;
  encoderCountLeft = encoderCountRight = prevTick = 0;
  
  targetTick = cm * 28.5; // Caliberated to 30.25 ticks per cm

  // check if forward is valid
  double distFL = calibrateSensorValue(sensorFL.distance(), 1);
  double distFC = calibrateSensorValue(sensorFC.distance(), 2);
  double distFR = calibrateSensorValue(sensorFR.distance(), 3);

  if (distFL <= 14) {
    calibrateDistance(sensorFL, 1);
    return false;
  }
  else if (distFC <= 14) {
    calibrateDistance(sensorFC, 2);
    return false;
  }
  else if (distFR <= 14) {
    calibrateDistance(sensorFR, 3);
    return false;
  }

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
  delay(50);

  return true;
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
  delay(50);
}

void rotateRight(double deg) {
  double pid;
  int targetTick;
  integral = 0;
  encoderCountLeft = encoderCountRight = prevTick = 0;

  if (deg <= 90) targetTick = deg * 4.375; //4.523
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
  delay(50);
}

void rotateLeft(double deg) {
  double pid;
  int targetTick;
  integral = 0;
  encoderCountLeft = encoderCountRight = prevTick = 0;

  if (deg <= 90) targetTick = deg * 4.424; //4.424;
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
  delay(50);
}

/**
 * ============================== For Sensors ==============================
 */
void readSensors() {
  digitalWrite(pinGreenLED, HIGH);
  digitalWrite(pinRedLED, HIGH);
  double distFL = calibrateSensorValue(sensorFL.distance(), 1);
  double distFC = calibrateSensorValue(sensorFC.distance(), 2);
  double distFR = calibrateSensorValue(sensorFR.distance(), 3);
  double distRF = calibrateSensorValue(sensorRF.distance(), 4);
  double distRR = calibrateSensorValue(sensorRR.distance(), 5);
  double distL = calibrateSensorValue(sensorL.distance(), 0);

  String output = "";

  // target side = right, other side = front
  int calibrate_front = 0;
  int calibrate_right = 0;

  int i;

  mode = 1;

  // check for best opportunity to calibrate
  if (((abs(distFL - distFR) < 5) && ((distFL + distFR) <= (3 * WALL_GAP))) || ((abs(distFL - distFC) < 5) && ((distFL + distFC) <= (3 * WALL_GAP))) || ((abs(distFC - distFR) < 5) && ((distFC + distFR) <= (3 * WALL_GAP)))) {
    if ((distRF <= (WALL_GAP + 4)) || (distRR <= (WALL_GAP + 4))) {
      step_counter = STEPS_TO_CALIBRATE;
    }
  }
  else if ((abs(distRF - distRR) < 5) && ((distRF + distRR) <= (3 * WALL_GAP))) {
    if ((distFL <= (WALL_GAP + 4)) || (distFC <= (WALL_GAP + 4)) || (distFR <= (WALL_GAP + 4))) {
      step_counter = STEPS_TO_CALIBRATE;
    }
  }

  // calibrate if above steps count
  if (step_counter >= STEPS_TO_CALIBRATE) {
    // check if target side can calibrate angle
    if ((abs(distFL - distFR) < 5) && ((distFL + distFR) <= (3 * WALL_GAP))) {
      calibrateAngle(sensorFL, 1, sensorFR, 3, 17);
      calibrateDistance(sensorFL, 1);
      calibrate_front = 1;
      step_counter = 0;
    }
    else if ((abs(distFL - distFC) < 5) && ((distFL + distFC) <= (3 * WALL_GAP))) {
      calibrateAngle(sensorFL, 1, sensorFC, 2, 9);
      calibrateDistance(sensorFC, 2);
      calibrate_front = 2;
      step_counter = 0;
    }
    else if ((abs(distFC - distFR) < 5) && ((distFC + distFR) <= (3 * WALL_GAP))) {
      calibrateAngle(sensorFC, 2, sensorFR, 3, 9);
      calibrateDistance(sensorFC, 2);
      calibrate_front = 3;
      step_counter = 0;
    }

    // check for 1 obstacle on other side
    // if yes, calibrate dist on other side
    if (calibrate_front > 0) {
      distRF = calibrateSensorValue(sensorRF.distance(), 4);
      
      if (distRF <= (WALL_GAP + 4)) {
        mode = 0;
        rotateRight(90);
        mode = 1;
        calibrateDistance(sensorFL, 1);
        mode = 0;
        rotateLeft(90);
        mode = 1;
        calibrate_right = 1;
      }
      else  {
        distRR = calibrateSensorValue(sensorRR.distance(), 5);

        if (distRR <= (WALL_GAP + 4)) {
          mode = 0;
          rotateRight(90);
          mode = 1;
          calibrateDistance(sensorFR, 3);
          mode = 0;
          rotateLeft(90);
          mode = 1;
          calibrate_right = 1;
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

        // update the sensor values if calibrated
        distRF = calibrateSensorValue(sensorRF.distance(), 4);
        distRR = calibrateSensorValue(sensorRR.distance(), 5);
      }
      distFL = calibrateSensorValue(sensorFL.distance(), 1);
      distFC = calibrateSensorValue(sensorFC.distance(), 2);
      distFR = calibrateSensorValue(sensorFR.distance(), 3);
      distL = calibrateSensorValue(sensorL.distance(), 0);
    }
    else  {
      // check for right wall and calibrate
      if ((abs(distRF - distRR) < 5) && ((distRF + distRR) <= (3 * WALL_GAP))) {
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

        // calibrate to right
        mode = 0;
        rotateRight(90);
        mode = 1;
        calibrateAngle(sensorFL, 1, sensorFR, 3, 17);
        calibrateDistance(sensorFL, 1);
        mode = 0;
        rotateLeft(90);
        mode = 1;

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
        

        // update the sensor values if calibrated
        if (calibrate_front > 0) {
          distFL = calibrateSensorValue(sensorFL.distance(), 1);
          distFC = calibrateSensorValue(sensorFC.distance(), 2);
          distFR = calibrateSensorValue(sensorFR.distance(), 3);
        }

        distRF = calibrateSensorValue(sensorRF.distance(), 4);
        distRR = calibrateSensorValue(sensorRR.distance(), 5);
        distL = calibrateSensorValue(sensorL.distance(), 0);
      }
    }
  }
  else {
    if ((abs(distRF - distRR) < 5) && ((distRF + distRR) <= (3 * WALL_GAP))) {
      calibrateAngle(sensorRF, 4, sensorRR, 5, 19);
      
      if (((distRF + distRR) < ((2 * WALL_GAP) - 8)) || ((distRF + distRR) > ((2 * WALL_GAP) + 8))) {
        mode = 0;
        rotateRight(90);
        mode = 1;
        calibrateDistance(sensorFL, 1);
        mode = 0;
        rotateLeft(90);
        mode = 1;
        calibrateAngle(sensorRF, 4, sensorRR, 5, 19);

        distFL = calibrateSensorValue(sensorFL.distance(), 1);
        distFC = calibrateSensorValue(sensorFC.distance(), 2);
        distFR = calibrateSensorValue(sensorFR.distance(), 3);
        distRF = calibrateSensorValue(sensorRF.distance(), 4);
        distRR = calibrateSensorValue(sensorRR.distance(), 5);
        distL = calibrateSensorValue(sensorL.distance(), 0);
      }
    }
  }

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

int obstaclePosition(int val, int shortrange){
  int tmp = 0;

  int modulo = ((int) (val + 0.5)) % 10;
  
  if ((modulo != 7) && (val % 10 != 8) && (val % 10 != 9) && (val % 10 != 0) && (val % 10 != 1) && (val % 10 != 2) && (val % 10 != 3)) {
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
      step_counter = 0;
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
    step_counter = 0;
  }
  else if ((abs(distL - distC) < 5) && ((distL + distC) <= (3 * WALL_GAP))) {
    calibrateAngle(sensorFL, 1, sensorFC, 2, 9);
    calibrateDistance(sensorFC, 2);
    calibrateAutoChecked = true;
    step_counter = 0;
  }
  else if ((abs(distC - distR) < 5) && ((distC + distR) <= (3 * WALL_GAP))) {
    calibrateAngle(sensorFC, 2, sensorFR, 3, 9);
    calibrateDistance(sensorFC, 2);
    calibrateAutoChecked = true;
    step_counter = 0;
  }
}
 
void calibrateAngle(SharpIR sensorL, int arrL, SharpIR sensorR, int arrR, int dist) {
  double distL = calibrateSensorValue(sensorL.distance(), arrL);
  double distR = calibrateSensorValue(sensorR.distance(), arrR);
  double diff = abs(distL - distR);
  double mean = diff / 2;

  double angle = 0;

  angle = (asin(mean/dist) * (180/3.14159265));
  // angle = (mean > 0.5) ? angle : angle/2;

  while (angle > 30) {
    if (distL > distR){
      rotateRight(10);
    }
    else if (distR > distL){   
      rotateLeft(10);
    }

    distL = calibrateSensorValue(sensorL.distance(), arrL);
    distR = calibrateSensorValue(sensorR.distance(), arrR);
    diff = abs(distL - distR);
    mean = diff / 2;

    angle = (asin(mean/dist) * (180/3.14159265));
    // angle = (mean > 0.5) ? angle : angle/2;
  }

  if (distL > distR){
    rotateRight(angle);
  }
  else if (distR > distL){   
    rotateLeft(angle);
  }
    
  // while (mean > 0.25){
  //   // Serial.println("dist: " + String(distL) + ", " + String(distR) + ", " + String(diff) + ", " + String(mean));
  //   angle = (asin(mean/dist) * (180/3.14159265));
  //   angle = (mean > 0.5) ? angle : angle/2;
  //   // Serial.println("angle: " + String(angle));
  //   if (distL > distR){
  //     rotateRight(angle);
  //   }
  //   else if (distR > distL){   
  //     rotateLeft(angle);
  //   }

  //   distL = calibrateSensorValue(sensorL.distance(), arrL);
  //   distR = calibrateSensorValue(sensorR.distance(), arrR);
  //   diff = abs(distL - distR);
  //   mean = diff / 2;
  // }
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

