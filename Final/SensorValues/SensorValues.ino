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
  
  Serial.print(dist0);
//  Serial.print(", ");
  long value1 = map(dist0,20,27,20,30);
  long value2 = map(dist0,27,37,30,40);
  long value3 = map(dist0,37,47,40,50);
  long value4 = map(dist0,47,57,50,60);
  long value5 = map(dist0,57,69,60,70);
  long value6 = map(dist0,69,79,70,80);
  long value7 = map(dist0,79,81,70,80);
  long value8 = map(dist0,81,91,80,90);
  Serial.println("Values are : " + String(value1)+","+ String(value2)+","+ String(value3)+","+String(value4)+","+String(value5)+","+String(value6)+","+String(value7)+","+String(value8));
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

