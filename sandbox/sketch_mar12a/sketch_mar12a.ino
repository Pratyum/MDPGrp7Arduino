void setup() {
  Serial.begin(9600);
}

void loop() {
  int i = 0, j = 1;
  float value;
  String valueString = "";
  char commandBuffer[10];
  char command, ch;
    
  while (1){
    if (Serial.available()) {
      ch = Serial.read();
      commandBuffer[i] = ch;
      i++;
      
      if (ch == '|'){
        break;
      }
    }
  }
  
  command = commandBuffer[0];

  while (j < (i-1)) {
    valueString += commandBuffer[j];
    j++;
  }

  value = valueString.toDouble();

  value = (value == 0) ? 1 : value;

  Serial.println("valueString: " + valueString);
  Serial.println("value: " + String(value));
  Serial.println("value * 10: " + String(value * 10));
  Serial.println();
}
