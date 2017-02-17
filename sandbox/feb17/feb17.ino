#include "SharpIR.h"

#define pinSensor1 1
#define pinSensor2 2
#define pinSensor3 3
#define pinSensor4 4
#define pinSensor5 5
#define MODEL 1080 // 1080 (Short), 20150 (Long)

//SharpIR sensor1(pinSensor1, 100, 95, MODEL);
SharpIR sensor2(pinSensor2, 200, 99, MODEL);
SharpIR sensor3(pinSensor3, 200, 99, MODEL);
SharpIR sensor4(pinSensor4, 200, 99, MODEL);
//SharpIR sensor5(pinSensor5, 100, 95, MODEL);

void setup() {
  Serial.begin(9600);
}

void loop() {
  delay(300);   

  double pepe1=millis();  // takes the time before the loop on the library begins
//  Serial.print(sensor1.distance());
//  Serial.print(", ");
  Serial.print(sensor2.distance());
  Serial.print(", ");
  Serial.print(sensor3.distance());
  Serial.print(", ");
  Serial.print(sensor4.distance());
//  Serial.print(", ");
//  Serial.print(sensor5.distance());
  Serial.println();

  double pepe2=millis()-pepe1;  // the following gives you the time taken to get the measurement
  
  //Serial.print("Time taken (ms): ");
  //Serial.println(pepe2);
}

int calibrateSensorValue(int n){
  double val_10, val_20, val_30, val_40;

  return 0;
  
}

