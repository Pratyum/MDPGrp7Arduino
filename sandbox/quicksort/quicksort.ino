void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
}

void loop() {
  double arr[10] = {4.5, 3.6, 2.9, 6.1, 2, 8.7, 1.4, 7.8, 9.3, 5.0};
  quickSort(arr, 0, 9);
  for (int i = 0; i < 10; i++) {
    Serial.print(arr[i]);
    Serial.print(", ");
  }
  
  delay(100000);
}

void quickSort(double arr[], int left, int right) {
     int i = left, j = right;
     double tmp;
     double pivot = arr[(left + right) / 2];

     /* partition */
     while (i <= j) {
           while (arr[i] < pivot)
                 i++;
           while (arr[j] > pivot)
                 j--;
           if (i <= j) {
                 tmp = arr[i];
                 arr[i] = arr[j];
                 arr[j] = tmp;
                 i++;
                 j--;
           }
     };

     /* recursion */
     if (left < j)
           quickSort(arr, left, j);
     if (i < right)
           quickSort(arr, i, right);
}
