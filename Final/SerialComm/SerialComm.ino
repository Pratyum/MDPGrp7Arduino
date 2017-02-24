void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  Serial.begin(9600);
}

void loop()
{
  /**
   * return string that is received
   */
  /*
  String tmp = "";
  while(Serial.available() == 0);
  Serial.setTimeout(10);
  tmp = Serial.readString();

  Serial.println(tmp);
  //*/

  
  /**
   * return char with ascii + 1
   */
  //*
  if (Serial.available() != 0){ 
    Serial.println(char(Serial.read()+1));
//    blinkLED();
  }
  //*/
}

void blinkLED(){
  digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)
  delay(1000);                       // wait for a second
  digitalWrite(LED_BUILTIN, LOW);    // turn the LED off by making the voltage LOW
  delay(1000);                       // wait for a second
}

