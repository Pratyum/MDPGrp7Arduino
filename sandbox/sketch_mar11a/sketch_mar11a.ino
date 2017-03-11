#define pin 7

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);

  pinMode(pin, OUTPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  digitalWrite(pin, HIGH);
  delay(1000);
  digitalWrite(pin, LOW);
  delay(1000);
}
