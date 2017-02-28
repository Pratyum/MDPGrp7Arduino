#include "SharpIR.h"

#define pinSensor0 0
#define pinSensor1 1
#define pinSensor2 2
#define pinSensor3 3 
#define pinSensor4 4
#define pinSensor5 5

#define MODEL_SHORT 1080 // 1080 (Short), 20150 (Long)
#define MODEL_LONG 20150 // 1080 (Short), 20150 (Long)

SharpIR sensor0(pinSensor0, 200, 99, MODEL_LONG);
SharpIR sensor1(pinSensor1, 200, 99, MODEL_SHORT);
SharpIR sensor2(pinSensor2, 200, 99, MODEL_SHORT);
SharpIR sensor3(pinSensor3, 200, 99, MODEL_SHORT);
SharpIR sensor4(pinSensor4, 200, 99, MODEL_SHORT);
SharpIR sensor5(pinSensor5, 200, 99, MODEL_SHORT);

/**
 * ===== For mapping sensor values =====
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

void setup() {
  Serial.begin(9600);
}

void loop() {

  double pepe1 = millis();  // takes the time before the loop on the library begins
  double dist0 = sensor0.distance();
  double dist1 = sensor1.distance();
  double dist2 = sensor2.distance();
  double dist3 = sensor3.distance();
  double dist4 = sensor4.distance();
  double dist5 = sensor5.distance();
  double pepe2 = millis() - pepe1;  // the following gives you the time taken to get the measurement
  
  Serial.print(dist0);
  Serial.print(", ");
//  Serial.println("Mapped value is: " + String(calibrateSensorValue(dist0, 0)));
//  Serial.print(obstaclePosition(dist0));
//  Serial.print(", ");
  Serial.print(dist1);
  Serial.print(", ");
//  Serial.print(obstaclePosition(dist1));
//  Serial.print(", ");
  Serial.print(dist2);
  Serial.print(", ");
//  Serial.print(obstaclePosition(dist2));
//  Serial.print(", ");
  Serial.print(dist3);
  Serial.print(", ");
//  Serial.print(obstaclePosition(dist3));
//  Serial.print(", ");
  Serial.print(dist4);
  Serial.print(", ");
//  Serial.print(obstaclePosition(dist4));
//  Serial.print(", ");
  Serial.print(dist5);
//  Serial.print(", ");
//  Serial.print(obstaclePosition(dist5));
  Serial.println();
  Serial.print(calibrateSensorValue(dist0, 0));
  Serial.print(", ");
  Serial.print(calibrateSensorValue(dist1, 1));
  Serial.print(", ");
  Serial.print(calibrateSensorValue(dist2, 2));
  Serial.print(", ");
  Serial.print(calibrateSensorValue(dist3, 3));
  Serial.print(", ");
  Serial.print(calibrateSensorValue(dist4, 4));
  Serial.print(", ");
  Serial.print(calibrateSensorValue(dist5, 5));
  Serial.print(", ");
  Serial.println();
  Serial.print("Time taken (ms): ");
  Serial.println(pepe2);

  delay(1000);
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
//      Serial.println("true, " + String(i) + ", " + String(dist) + ", " + arr[i]);
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

double modifiedMap(double x, double in_min, double in_max, double out_min, double out_max)
{
 double temp = (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
 temp = (int) (4*temp + .5);
 return (double) temp/4;
}

