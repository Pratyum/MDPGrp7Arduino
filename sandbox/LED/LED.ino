/*
 * Jeffrey's First Program
 */
int switchPin = 8;
int ledPin = 11;
boolean lastButton = LOW;
boolean currentButton = LOW;
int ledLevel = 0;

void setup() {
  // put your setup code here, to run once:
  // initialize pins as outputs
  pinMode(switchPin, INPUT);
  pinMode(ledPin, OUTPUT);
}

boolean debounce(boolean last) {
  boolean current = digitalRead(switchPin);
  if (last != current) {
    delay(5);
    current = digitalRead(switchPin);
  }
  return current;
}

void loop() {
  // put your main code here, to run repeatedly:
  currentButton = debounce(lastButton);
  if (lastButton == LOW && currentButton == HIGH) {
    ledLevel = ledLevel + 50;
  }
  lastButton = currentButton;

  if (ledLevel > 255) ledLevel = 0;
  analogWrite(ledPin, ledLevel);
}
