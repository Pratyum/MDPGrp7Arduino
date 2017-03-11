#include "SharpIR.h"
#include "RunningMedian.h"

#define pinSensor0 0
#define pinSensor1 1
#define pinSensor2 2
#define pinSensor3 3 
#define pinSensor4 4
#define pinSensor5 5

#define MODEL_SHORT 1080 // 1080 (Short), 20150 (Long)
#define MODEL_LONG 20150 // 1080 (Short), 20150 (Long)

SharpIR sensor0(pinSensor0, MODEL_LONG);
SharpIR sensor1(pinSensor1, MODEL_SHORT);
SharpIR sensor2(pinSensor2, MODEL_SHORT);
SharpIR sensor3(pinSensor3, MODEL_SHORT);
SharpIR sensor4(pinSensor4, MODEL_SHORT);
SharpIR sensor5(pinSensor5, MODEL_SHORT);

/**
 * ===== For mapping sensor values =====
 * key in values returned from sensor
 * arrMapping0 is for long-range
 * long-range (start from 20); short-range (start from 10)
 */
// values on 28 feb
//double arrMapping0[] = {20.3, 25.36, 33.7, 43.78, 53.8, 65.5, 74.7, 86.6, 98.4, 110.7, 125.57, 139.8};
//double arrMapping1[] = {9.94, 21, 32.66, 45.5, 61, 81};
//double arrMapping2[] = {9.82, 20.45, 32.55, 44.66, 52.3, 61.4};
//double arrMapping3[] = {10.58, 21.66, 32.43, 35.5, 35.6, 40.1};
//double arrMapping4[] = {10.51, 22.56, 36.4, 48.3, 60.7, 71};
//double arrMapping5[] = {10.21, 21.68, 33.52, 42.5, 50.6, 60, 70, 80};

// values on 1 mar
//double arrMapping0[] = {20.3, 25.36, 33.7, 43.78, 53.8, 65.5, 74.7, 86.6, 98.4, 110.7, 125.57, 139.8};
//double arrMapping1[] = {9.39, 21.45, 35.7, 53.5, 61, 81};
//double arrMapping2[] = {9.25, 19.90, 31.5, 44.66, 52.3, 61.4};
//double arrMapping3[] = {9.38, 19.60, 29.2, 35.5, 35.6, 40.1};
//double arrMapping4[] = {10.51, 22.56, 36.4, 48.3, 60.7, 71};
//double arrMapping5[] = {10.21, 21.68, 33.52, 42.5, 50.6, 60, 70, 80};

// values on 2 mar
//double arrMapping0[] = {18.74, 23.90, 32.29, 42.15, 52.99, 63.31, 72.90, 86.22, 100.06, 112.23};
//double arrMapping1[] = {10.14, 23.00, 38.02, 54.9, 69.27};
//double arrMapping2[] = {9.57, 20.14, 31.72, 43.05, 54.46};
//double arrMapping3[] = {10.03, 20.40, 30.74, 33.61};
//double arrMapping4[] = {9.91, 20.99, 32.72, 47.02, 60.33};
//double arrMapping5[] = {9.89, 22.22, 35.16, 46.42, 50.75};

// values on 8 mar
//double arrMapping0[] = {17.85, 20.84, 27.43, 47.22, 58.60, 72.41, 72.90, 89.02, 131.06};
//double arrMapping1[] = {8.87, 19.09, 30.54, 47.02, 74.50};
//double arrMapping2[] = {8.72, 19.10, 29.95, 39.8, 47.8};
//double arrMapping3[] = {9.19, 19.18, 29.76, 41.67,59.9};
//double arrMapping4[] = {10.18, 21.07, 33.49, 48.84, 69.92};
//double arrMapping5[] = {9.81, 21.48, 33.04, 43.81, 55.04};

// values on 9 mar
//double arrMapping0[] = {17.85, 20.84, 27.43, 47.22, 58.60, 72.41, 72.90, 89.02, 131.06};
//double arrMapping1[] = {10.78, 21.59, 33.96, 48.84, 71.14};
//double arrMapping2[] = {10.40, 20.89, 31.55, 40.52, 51};
//double arrMapping3[] = {11.01, 21.59, 33.02, 46.24, 64.60};
//double arrMapping4[] = {9.68, 20.50, 31.73, 43.81, 62.69};
//double arrMapping5[] = {9.89, 21.27, 32.61, 40.81, 49.80};

// values on 10 mar
double arrMapping0[] = {18.25, 23.24, 30.72, 41.27, 50.80, 61.66, 72.41, 89.02, 131.06};
double arrMapping1[] = {9.63, 20.40, 32.39, 47.02, 71.14};
double arrMapping2[] = {9.00, 19.87, 30.54, 41.15, 51};
double arrMapping3[] = {9.55, 20.00, 30.64, 42.90, 64.60};
double arrMapping4[] = {9.33, 20.40, 30.94, 37.72, 62.69};
double arrMapping5[] = {9.97, 20.40, 30.14, 34.45, 49.80};

void setup() {
  Serial.begin(9600);
}

void loop() {
  test();
//  fastRead(4, 50);
  delay(1000);
}

void test() {

  double pepe1 = millis();  // takes the time before the loop on the library begins
  int sample_size = 10;
  double dist0 = medianDistance(sensor0, sample_size);
  double dist1 = medianDistance(sensor1, sample_size);
  double dist2 = medianDistance(sensor2, sample_size);
  double dist3 = medianDistance(sensor3, sample_size);
  double dist4 = medianDistance(sensor4, sample_size);
  double dist5 = medianDistance(sensor5, sample_size);
  double pepe2 = millis() - pepe1;  // the following gives you the time taken to get the measurement

   double time0 = millis();
   double distance0 = sensor0.distance();
   double distance1 = sensor1.distance();
   double distance2 = sensor2.distance();
   double distance3 = sensor3.distance();
   double distance4 = sensor4.distance();
   double distance5 = sensor5.distance();
   double end_time0 = millis() - time0;

  
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
  Serial.print("Time taken with median (ms): ");
  Serial.println(pepe2);
  Serial.print("Time taken without median (ms): ");
  Serial.println(end_time0);
  Serial.println();

  //delay(1000);
}

double medianDistance(SharpIR sensor , int sample_size) {
  RunningMedian sample = RunningMedian(sample_size);
  for (int i = 0; i < sample_size; i ++) {
    sample.add(sensor.distance()); 
  }
  return sample.getMedian();
}

double calibrateSensorValue(double dist, int n){
  double *arr;
  int i, len;
  
  
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
      int a = (i == 0)? 0 : arr[i-1];
      int offset = (n == 0)? 1 : 0;

      return modifiedMap(dist, a, arr[i], ((i + offset) * 10), ((i + offset + 1) * 10));
    }
  }
  
  return i*10;
}

int obstaclePosition(int val){
  return ((val + 4)) / 10;
}

double modifiedMap(double x, double in_min, double in_max, double out_min, double out_max) {
 double temp = (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
// temp = (int) (4*temp + .5);
// return (double) temp/4;
return temp;
}
