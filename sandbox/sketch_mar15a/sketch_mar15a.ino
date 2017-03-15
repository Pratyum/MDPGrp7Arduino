int n;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  n = 0;
}

void loop() {
  // put your main code here, to run repeatedly:
  
  inc(n);

  Serial.println(n);

  delay(100);
}

void inc(int &a) {
  a++;
}

