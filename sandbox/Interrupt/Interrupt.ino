#define encoder0PinA  3

volatile unsigned long encoder0Pos = 0;

int ledPin = 13;

void setup() { 
  pinMode(encoder0PinA, INPUT); 
  //digitalWrite(encoder0PinA, HIGH);       // turn on pull-up resistor

  //attachInterrupt(digitalPinToInterrupt(encoder0PinA), doEncoder, RISING);  // encoder pin on interrupt 0 - pin 2

  
  //attachInterrupt(1, doEncoderB, CHANGE);
  
  Serial.begin (9600);
  Serial.println("start");                // a personal quirk

  attachInterrupt(digitalPinToInterrupt(encoder0PinA), doEncoder, RISING);

  pinMode(ledPin, OUTPUT);
} 

void loop(){
  for (int i = 0; i < 10; i++) {
    digitalWrite(ledPin, HIGH);
    delay(100);
    digitalWrite(ledPin, LOW);
    delay(100);
  }
  
  Serial.println("Encoder0: ");
  Serial.println (encoder0Pos, DEC);
//  Serial.println("Encoder1: ");
//  Serial.println (encoder1Pos, DEC);

  delay(1000);
  // end loop()
}

void doEncoder() {
  encoder0Pos++;
  Serial.println ("doEncoder()");
}
