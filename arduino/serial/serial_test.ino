char dataString[50] = {0};
int a = 0;

void setup() {
  Serial.begin(9600);
}

void loop() {
  a++;
  sprintf(dataString, "%02X",a); // convert value to hexa
  Serial.println(dataString); // send data
  delay(1000);
}
  
