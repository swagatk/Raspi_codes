// Ultrasonic Sensors (As you had them before)
const int leftTrig = 2, leftEcho = 3;
const int centerTrig = 4, centerEcho = 5;
const int rightTrig = 6, rightEcho = 7;

// Motor Pins (New connections)
const int ENA = 10; // PWM Speed Left
const int IN1 = 8;
const int IN2 = 9;
const int ENB = 11; // PWM Speed Right
const int IN3 = 12;
const int IN4 = 13;

// Variables
int motorSpeed = 150; // Default speed (0-255)
char command = 'S';   // Current command (S=Stop)
unsigned long lastCommandTime = 0; // Safety timeout

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  
  // Sensor Setup
  pinMode(leftTrig, OUTPUT); pinMode(leftEcho, INPUT);
  pinMode(centerTrig, OUTPUT); pinMode(centerEcho, INPUT);
  pinMode(rightTrig, OUTPUT); pinMode(rightEcho, INPUT);
  
  // Motor Setup
  pinMode(ENA, OUTPUT); pinMode(ENB, OUTPUT);
  pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);
  
  stopMotors();

}

// --- HELPER FUNCTIONS ---
void setMotors(int left, int right) {
  // Left Motor
  if (left > 0) { digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW); }
  else if (left < 0) { digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH); }
  else { digitalWrite(IN1, LOW); digitalWrite(IN2, LOW); }
  analogWrite(ENA, abs(left));

  // Right Motor
  if (right > 0) { digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW); }
  else if (right < 0) { digitalWrite(IN3, LOW); digitalWrite(IN4, HIGH); }
  else { digitalWrite(IN3, LOW); digitalWrite(IN4, LOW); }
  analogWrite(ENB, abs(right));
}

void stopMotors() {
  setMotors(0, 0);
}

int getDistance(int trig, int echo) {
  digitalWrite(trig, LOW); delayMicroseconds(2);
  digitalWrite(trig, HIGH); delayMicroseconds(10);
  digitalWrite(trig, LOW);
  long duration = pulseIn(echo, HIGH, 20000); // 20ms timeout
  if (duration == 0) return 999;
  return duration * 0.034 / 2;
}

void loop() {
  // put your main code here, to run repeatedly:
  // 1. READ COMMANDS FROM PI
  if (Serial.available() > 0) {
    command = Serial.read();
    lastCommandTime = millis(); // Reset safety timer
    
    // Interpret Command
    if (command == 'F') setMotors(motorSpeed, -motorSpeed);        // Forward
    else if (command == 'B') setMotors(-motorSpeed, motorSpeed); // Backward
    else if (command == 'L') setMotors(motorSpeed, motorSpeed);  // Left Spin
    else if (command == 'R') setMotors(-motorSpeed, -motorSpeed);  // Right Spin
    else if (command == 'S') stopMotors();                        // Stop
    // Speed control (1 = Slow, 2 = Medium, 3 = Fast)
    else if (command == '1') motorSpeed = 100;
    else if (command == '2') motorSpeed = 180;
    else if (command == '3') motorSpeed = 255;
  }
  
  // 2. SAFETY STOP (Watchdog)
  // If Pi disconnects or crashes, stop motors after 1 second of silence
  if (millis() - lastCommandTime > 1000) {
    stopMotors();
  }

  // 3. READ SENSORS & SEND TO PI (Every 100ms)
  static unsigned long lastSensorTime = 0;
  if (millis() - lastSensorTime > 500) {
    lastSensorTime = millis();
    int d1 = getDistance(leftTrig, leftEcho);
    int d2 = getDistance(centerTrig, centerEcho);
    int d3 = getDistance(rightTrig, rightEcho);
    
    // Format: "D,Left,Center,Right"
    Serial.print("D,");
    Serial.print(d1); Serial.print(",");
    Serial.print(d2); Serial.print(",");
    Serial.println(d3);
  }

}
