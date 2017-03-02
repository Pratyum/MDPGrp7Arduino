void setup() {
  Serial.begin(9600);

  randomSeed(analogRead(0));
}

void loop() {
  int i = 0, j = 1, val = 0;
  char commandBuffer[10];
  char command, ch;
  bool flag = true;

  while (1){
    if (Serial.available()) {
      delay(10);
      while (Serial.available()) {
        ch = Serial.read();
        commandBuffer[i] = ch;
        i++;
      }
      break;
    }
  }

  command = commandBuffer[0];

  while (j < i) {
    val *= 10; 
    val = val + (commandBuffer[j] - 48);
    j++;
  }

  switch (command) {
    case 'F': case 'f': // forward
      forward(val * 10); 
      break;
    case 'B': case 'b': // reverse
      reverse(val * 10);
      break;
    case 'L': case 'l': // rotateLeft
      rotateLeft(val);
      break;
    case 'R': case 'r': // rotateRight
      rotateRight(val);
      break;
    case 'S': case 's': // readSensors
      readSensors();
      break;
    case 'C': case 'c': // calibrate
//      calibrate(val);
      break;
    default: 
      flag = false;
      Serial.println("E");
  }

  if (flag) {
    Serial.println("D");
  }
}

void forward(double cm) {

  delay(100);
}


void reverse(double cm) {

  delay(100);
}

int rotateRight(double deg) {

  delay(100);
}

int rotateLeft(double deg) {

  delay(100);
}

/**
 * ============================== For Sensors ==============================
 */
void readSensors() {
  String output = "";

  output += String(random(3));
  output += String(random(3));
  output += String(random(3));
  output += String(random(3));
  output += String(random(3));
  output += String(random(5));

  Serial.println(output);
}
