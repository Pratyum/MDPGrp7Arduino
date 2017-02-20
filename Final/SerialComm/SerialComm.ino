void setup() {
  Serial.begin(9600);
}

void loop()
{
  /**
   * return string that is received
   */
  //  String tmp = "";
  //  while(Serial.available() == 0);
  //  Serial.setTimeout(10);
  //  tmp = Serial.readString();
  //
  //  Serial.println(tmp);
  
  /**
   * return char with ascii + 1
   */
  while (Serial.available() == 0);
  Serial.println(char(Serial.read()+1));
}
