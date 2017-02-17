/* read a rotary encoder with interrupts
   Encoder hooked up with common to GROUND,
   encoder0PinA to pin 2, encoder0PinB to pin 4 (or pin 3 see below)
   it doesn't matter which encoder pin you use for A or B  

   uses Arduino pull-ups on A & B channel outputs
   turning on the pull-ups saves having to hook up resistors 
   to the A & B channel outputs 

*/ 

#include "DualVNH5019MotorShield.h"

DualVNH5019MotorShield md;

#define encoder0PinA  2
#define encoder0PinB  3
#define encoder1PinA  4
#define encoder1PinB  5

volatile unsigned long encoder0Pos = 0;
volatile unsigned long encoder1Pos = 0;

unsigned long lastValue0 = 0;
unsigned long lastValue1 = 0;

void setup() { 

  //interrupts();
  pinMode(encoder0PinA, INPUT); 
  //digitalWrite(encoder0PinA, HIGH);       // turn on pull-up resistor
  pinMode(encoder0PinB, INPUT); 
  //digitalWrite(encoder0PinB, HIGH);       // turn on pull-up resistor

  pinMode(encoder1PinA, INPUT); 
//  digitalWrite(encoder1PinA, HIGH);       // turn on pull-up resistor
  pinMode(encoder1PinB, INPUT); 
//  digitalWrite(encoder1PinB, HIGH);       // turn on pull-up resistor
  
  Serial.begin (115200);
  Serial.println("start");                // a personal quirk
  md.init();
  
  md.setM1Speed(50);
  md.setM2Speed(50);

  attachInterrupt(digitalPinToInterrupt(encoder0PinB), doEncoder0, RISING);
  attachInterrupt(digitalPinToInterrupt(encoder1PinB), doEncoder1, RISING);
}

void loop(){
  long tmp0, tmp1;
  
  Serial.println("Encoder0: ");
  tmp0 = encoder0Pos;
  Serial.println (tmp0 - lastValue0, DEC);
  lastValue0 = tmp0; 
  
  Serial.println("Encoder1: ");
  tmp1 = encoder1Pos;
  Serial.println (tmp1 - lastValue1, DEC);
  lastValue1 = tmp1;

  delay(1000);
  // end loop()
}

void stopIfFault()
{
  if (md.getM1Fault())
  {
    Serial.println("M1 fault");
    while(1);
  }
  if (md.getM2Fault())
  {
    Serial.println("M2 fault");
    while(1);
  }
}

void doEncoder0() {
  encoder0Pos++;
}

void doEncoder1() {
  encoder1Pos++;
}
