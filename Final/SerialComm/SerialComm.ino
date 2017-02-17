void setup() {
  Serial.begin(9600);
}

void loop()
{
  String tmp = "";
  while(Serial.available() == 0);
  Serial.setTimeout(10);
  tmp = Serial.readString();

  Serial.println(tmp);
}
