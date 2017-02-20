#include "DualVNH5019MotorShield.h"
#include "PinChangeInt.h"
#include "SharpIR.h"

DualVNH5019MotorShield md;

#define pinSensor1 1
#define pinSensor2 2
#define pinEncoderL 3
#define pinEncoderR 5
#define pinSwitch 8
#define MODEL 1080 // 1080 (Short), 20150 (Long)

SharpIR sensor1(pinSensor1, 200, 99, MODEL);
SharpIR sensor2(pinSensor2, 200, 99, MODEL);

unsigned long duration1, duration2;
double encoderCountLeft = 0, encoderCountRight = 0;

double target_Tick = 0, last_tick = 0; 
double error = 0, integral = 0;
unsigned long lastTime=0;
double lastInput = 0;
double ITerm =0;

double pid = 0;
bool inObstacleAvoid = false;


void setup(){
  Serial.begin(9600);
  
  md.init();
  pinMode(pinEncoderL, INPUT);
  pinMode(pinEncoderR, INPUT);
  pinMode(pinSwitch, INPUT);
//  myPID.SetSampleTime(5);
  PCintPort::attachInterrupt(pinEncoderL, incLeft, RISING);
  PCintPort::attachInterrupt(pinEncoderR, incRight, RISING);
  
  //md.setSpeeds(100, 100);
}

void loop(){
  moveForward(100);
  delay(1000000);
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

  kp=15; // trial and error 
  ki=0;
  kd=0;


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
  
  while (encoderCountLeft < target_Tick ){
    if(!checkForObstcle()){
    pid = tuneWithPID();
    Serial.println(pid);
    md.setSpeeds(200 - pid, 200 + pid);
    }else{
      int last_target_tick = target_Tick;
      moveForward(25);
      //TODO : Check left and Right for the sensors to move left or Right
      rotateLeft(90);
      //TODO : Check which sensor to move away from the block
      moveForward(20);
      rotateRight(90);
      moveForward(10);
      rotateRight(90);
      moveForward(20);
      rotateLeft(90);
      inObstacleAvoid = false;
      target_Tick = last_target_tick - (10*30.25);
    }
  }
  
   md.setBrakes(400, 400);
}

void moveBackward(double cmDis) {
  pid = 0;
  encoderCountLeft = 0, encoderCountRight = 0;   
  error = 0, integral = 0, last_tick = 0;

  target_Tick = cmDis * 30.25; // Caliberated to 30.25 ticks per cm 
  
  while (encoderCountLeft < target_Tick ){
//    pid = tuneWithPID();
//   Serial.println(encoderCountLeft);
//   Serial.println(encoderCountRight);
//   if( myPID.Compute()){
//    Serial.println("PID WORKS");
//   }
//    pid = tuneWithPID2();
    pid = tuneWithPID();
    Serial.println(pid);
    md.setSpeeds(-(200 - pid), -(200 + pid));
  }
  
   md.setBrakes(400, 400);
}

int rotateRight(double angle) {
  pid = 0;
  encoderCountRight = 0, encoderCountLeft = 0; 
  error = 0, integral = 0;

  if (angle <= 90) target_Tick = angle * 4.47;
  else if (angle <=180 ) target_Tick = angle * 4.62;
  else if (angle <=360 ) target_Tick = angle * 4.675;
  else target_Tick = angle * 4.65;

  while (encoderCountLeft < target_Tick ) {
    pid = tuneWithPID2();
    md.setSpeeds(200 - pid, -(200 + pid));
  }
  md.setBrakes(400,400);
}

int rotateLeft(double angle) {
  pid = 0;
  encoderCountRight = 0, encoderCountLeft = 0; 
  error = 0, integral = 0;

  if (angle <= 90) target_Tick = angle * 4.47;
  else if (angle <=180 ) target_Tick = angle * 4.62;
  else if (angle <=360 ) target_Tick = angle * 4.675;
  else target_Tick = angle * 4.65;

  while (encoderCountLeft < target_Tick ) {
    pid = tuneWithPID2();
    md.setSpeeds(-(200 - pid), (200 + pid));
  }
  md.setBrakes(400,400);
}


double tuneWithPID2(){
   double ki=0,kp=15,kd=0;
   unsigned long now = millis();
   unsigned long timeChange = (now - lastTime);
      /*Compute all the working error variables*/
      double input = encoderCountRight;
      double error = encoderCountLeft - input;
      ITerm+= (ki * error);
      double dInput = (input - lastInput);
      /*Remember some variables for next time*/
      lastInput = encoderCountRight;
      lastTime = now;
 
      /*Compute PID Output*/
      double output = kp * error + ITerm- kd * dInput;
      return output;
      
}

bool checkForObstcle(){
  //TODO : Add the third sensor
  if(inObstacleAvoid){
    return false;
  }
  if(calibrateSensorValue(sensor1.distance())<=30 || calibrateSensorValue(sensor2.distance())<=30 ) {
    inObstacleAvoid = true;
    return true;
  }
  return false;
}

int calibrateSensorValue(int val){
  /**
   * 20cm distance has the most accurate reading
   * with each increment of 10cm, there will be 2cm additional increment in readings
   */
  return (val + 4)/1.2;
}

int obstaclePosition(int val){
  /**
   * range of values are from 
   * (1.2n - 4) - (4 + 0.1n) to (1.2n - 4) + (4 + 0.1n)
   */
  int i;
  for (i = 1; i <= 10; i++){
    if (val <= (13 * i)) {
      return i;
    }
  }
  return 0;
}

