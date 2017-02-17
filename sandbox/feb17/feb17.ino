
String tmp = "";

void setup() {
  Serial.begin(9600);
}

void loop()
{
  while(Serial.available() == 0);
  Serial.setTimeout(10);
  tmp = Serial.readString();

  Serial.println(tmp);

//  if (tmp == "ABC") {
//    Serial.println("ABC");
//  }
//  else {
//    Serial.println("Others");
//  }
}
