void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
}
// values on 10 mar
double arrMapping0[] = {18.25, 23.24, 30.72, 41.27, 50.80, 61.66, 72.41, 89.02, 131.06};
double arrMapping1[] = {9.63, 20.40, 32.39, 47.02, 71.14};
double arrMapping2[] = {9.00, 19.87, 30.54, 41.15, 51};
double arrMapping3[] = {9.55, 20.00, 30.64, 42.90, 64.60};
double arrMapping4[] = {9.33, 20.40, 30.94, 37.72, 62.69};
double arrMapping5[] = {9.97, 20.40, 30.14, 34.45, 49.80};

void loop() {
  // put your main code here, to run repeatedly:
  double now = millis();
  Serial.println(calibrateSensorValue(10, 1));
Serial.print(millis() - now);
  Serial.println(" ms");
  delay(1000);
}

double calibrateSensorValue(double dist, int n){
  
  double *arr;
  int i, len;

  //int dist = sensor.distance();
  
  switch(n){
    case 0: arr = arrMapping0; len = sizeof(arrMapping0)/sizeof(*arr); break;
    case 1: arr = arrMapping1; len = sizeof(arrMapping1)/sizeof(*arr); break;
    case 2: arr = arrMapping2; len = sizeof(arrMapping2)/sizeof(*arr); break;
    case 3: arr = arrMapping3; len = sizeof(arrMapping3)/sizeof(*arr); break;
    case 4: arr = arrMapping4; len = sizeof(arrMapping4)/sizeof(*arr); break;
    case 5: arr = arrMapping5; len = sizeof(arrMapping5)/sizeof(*arr); break;
    default: return -1;
  }

  for (i = 0; i < len; i++){
    if (dist < arr[i]){
      //Serial.println("true, " + String(i) + ", " + String(dist) + ", " + arr[i]);
      int a = (i == 0)? 0 : arr[i-1];
      int offset = (n == 0)? 1 : 0;

      return modifiedMap(dist, a, arr[i], ((i + offset) * 10), ((i + offset + 1) * 10));
    }
  }
  
  return i*10;
}

double modifiedMap(double x, double in_min, double in_max, double out_min, double out_max) {
  double temp = (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
  // temp = (int) (4*temp + .5);
  // return (double) temp/4;
  return temp;
}
