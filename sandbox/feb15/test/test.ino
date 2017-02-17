#include "DualVNH5019MotorShield.h"
#include "PinChangeInt.h"

DualVNH5019MotorShield md;

#define pinEncoderL 3
#define pinEncoderR 5

unsigned long duration1, duration2;
unsigned int count = 0;

void setup(){
  Serial.begin(9600);
  
  md.init();
  pinMode(pinEncoderL, INPUT);
  pinMode(pinEncoderR, INPUT);
  
  md.setSpeeds(250, 250);
}

void loop() {
  if (count == 500) {
    md.setSpeeds(300, 300);
  }

  if (count == 1000) {
    md.setBrakes(300, 300);
  }
  
  duration1 = pulseIn(pinEncoderL, HIGH);
  duration2 = pulseIn(pinEncoderR, HIGH);

  //Serial.println("duration1: " + String(duration1) + " duration2: " + String(duration2));
  //Serial.println("rpm1: " + String(60000000/(2 * duration1 * 562.25)) + " rmp2: " + String (60000000/(2 * duration2 * 562.25)));

  Serial.println(String(count*5) + ", " + String(60000000/(2 * duration1 * 562.25)) + ", " + String (60000000/(2 * duration2 * 562.25)));

  count++;

  delay(5);
}

