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
//SharpIR sensor1(pinSensor1, 200, 99, MODEL_SHORT);
//SharpIR sensor2(pinSensor2, 200, 99, MODEL_SHORT);
//SharpIR sensor3(pinSensor3, 200, 99, MODEL_SHORT);
//SharpIR sensor4(pinSensor4, 200, 99, MODEL_SHORT);
//SharpIR sensor5(pinSensor5, 200, 99, MODEL_SHORT);

/**
 * ===== For mapping sensor values =====
 * key in values returned from sensor
 * arrMapping0 is for long-range
 * long-range (start from 20); short-range (start from 10)
 */
int arrMapping0[] = {20, 27, 37, 47, 58, 66, 78, 91, 100, 120, 130, 140, 150};
int arrMapping1[] = {10, 20, 30, 40, 50, 60, 70, 80};
int arrMapping2[] = {10, 20, 30, 40, 50, 60, 70, 80};
int arrMapping3[] = {10, 20, 30, 40, 50, 60, 70, 80};
int arrMapping4[] = {10, 20, 30, 40, 50, 60, 70, 80};
int arrMapping5[] = {10, 20, 30, 40, 50, 60, 70, 80};

void setup() {
  Serial.begin(9600);
}

void loop() {
  delay(200);

  double pepe1 = millis();  // takes the time before the loop on the library begins
  int dist0 = sensor0.distance();
//  int dist1 = sensor1.distance();
//  int dist2 = sensor2.distance();
//  int dist3 = sensor3.distance();
//  int dist4 = sensor4.distance();
//  int dist5 = sensor5.distance();
  
  Serial.println(dist0);
//  Serial.print(", ");
  Serial.println("Mapped value is: " + String(calibrateSensorValue(dist0, 0)));
//  Serial.print(obstaclePosition(dist0));
//  Serial.print(", ");
//  Serial.print(dist1);
//  Serial.print(", ");
//  Serial.print(obstaclePosition(dist1));
//  Serial.print(", ");
//  Serial.print(dist2);
//  Serial.print(", ");
//  Serial.print(obstaclePosition(dist2));
//  Serial.print(", ");
//  Serial.print(dist3);
//  Serial.print(", ");
//  Serial.print(obstaclePosition(dist3));
//  Serial.print(", ");
//  Serial.print(dist4);
//  Serial.print(", ");
//  Serial.print(obstaclePosition(dist4));
//  Serial.print(", ");
//  Serial.print(dist5);
//  Serial.print(", ");
//  Serial.print(obstaclePosition(dist5));
  Serial.println();

  double pepe2 = millis() - pepe1;  // the following gives you the time taken to get the measurement
  
//  Serial.print("Time taken (ms): ");
//  Serial.println(pepe2);
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

