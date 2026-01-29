// Define Pins
const int leftTrig = 2, leftEcho = 3;
const int centerTrig = 4, centerEcho = 5;
const int rightTrig = 6, rightEcho = 7;

void setup() {
  Serial.begin(9600); // Start communication with Pi
  
  pinMode(leftTrig, OUTPUT); pinMode(leftEcho, INPUT);
  pinMode(centerTrig, OUTPUT); pinMode(centerEcho, INPUT);
  pinMode(rightTrig, OUTPUT); pinMode(rightEcho, INPUT);
}

int getDistance(int trig, int echo) {
  digitalWrite(trig, LOW); delayMicroseconds(2);
  digitalWrite(trig, HIGH); delayMicroseconds(10);
  digitalWrite(trig, LOW);
  
  long duration = pulseIn(echo, HIGH, 30000); // Timeout after 30ms (~5m)
  if (duration == 0) return 999; // No echo? Return "Far away"
  return duration * 0.034 / 2;
}

void loop() {
  int d1 = getDistance(leftTrig, leftEcho);
  delay(15); // Prevent interference between sensors (Ghost echoes)
  int d2 = getDistance(centerTrig, centerEcho);
  delay(15);
  int d3 = getDistance(rightTrig, rightEcho);
  
  // Send to Pi as CSV: "Left,Center,Right"
  Serial.print(d1);
  Serial.print(",");
  Serial.print(d2);
  Serial.print(",");
  Serial.println(d3);
  
  delay(50); // Send updates 20 times a second
}
