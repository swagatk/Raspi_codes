#include <math.h>
#include <Servo.h>

// --- PIN DEFINITIONS ---

// 1. Ultrasonic Sensors (Moved to Analog Pins)
const int leftTrig = A0, leftEcho = A1;
const int centerTrig = A2, centerEcho = A3;
const int rightTrig = A4, rightEcho = A5;

// 2. Motor Pins (ENA moved to 3 to survive Timer 1 conflict)
const int ENA = 3;  // PWM Speed Left
const int IN1 = 8;
const int IN2 = 9;
const int ENB = 11; // PWM Speed Rights
const int IN3 = 12;
const int IN4 = 13;

// 3. Servo Motors (Using freed Digital Pins)
const int PIN_SHOULDER_L = 2;
const int PIN_SHOULDER_R = 4;
const int PIN_ELBOW_L = 5;
const int PIN_ELBOW_R = 6;
const int PIN_WRIST = 7;

// --- OBJECTS & VARIABLES ---
Servo shoulderLeft;
Servo shoulderRight;
Servo elbowLeft;
Servo elbowRight;
Servo wrist;

int motorSpeed = 255; // Default speed (0-255)
float right_scale = 1.0; //scaledown right wheel
float left_scale = 1.0; //scaledown left wheel
char command = 'S';   // Current command (S=Stop)
unsigned long lastCommandTime = 0; // Safety timeout

// Track current servo positions for smooth movement
int currentShoulderL = 180;  // DOWN position for Left
int currentShoulderR = 130;  // Home position for Right shoulder
int currentElbowL = 90;      // UP position for Elbow Left
int currentElbowR = 90;     // UP position for Elbow Right
const int SERVO_DELAY = 15;  // ms between steps (higher = slower)

void setup() {
  Serial.begin(115200);
  Serial.setTimeout(50);  // Reduce parseInt() timeout from 1000ms to 50ms
  
  // Sensor Setup
  pinMode(leftTrig, OUTPUT); pinMode(leftEcho, INPUT);
  pinMode(centerTrig, OUTPUT); pinMode(centerEcho, INPUT);
  pinMode(rightTrig, OUTPUT); pinMode(rightEcho, INPUT);
  
  // Motor Setup
  pinMode(ENA, OUTPUT); pinMode(ENB, OUTPUT);
  pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);
  
  // Set initial neutral/stop positions BEFORE attaching.
  // Using writeMicroseconds(1500) gives a much more precise "center" (stop pulse at 1.5ms) 
  shoulderLeft.writeMicroseconds(1500);
  shoulderRight.writeMicroseconds(1500);
  elbowLeft.writeMicroseconds(1500);
  elbowRight.writeMicroseconds(1500);
  wrist.writeMicroseconds(1500);

  // Servo Setup
  /* 
  COMMENTED OUT AT BOOT.
  We will NOT attach the servos here. By not attaching them, the Arduino sends 0 signals.
  The servos will stay completely dead and relaxed upon power-up.
  */
  
  stopMotors();
}

// --- HELPER FUNCTIONS FOR ARM ---
// Smooth shoulder movement - only the right shoulder is installed
void moveShoulderSmooth(int targetR, int delayMs = SERVO_DELAY) {
  if (!shoulderRight.attached()) shoulderRight.attach(PIN_SHOULDER_R);
  
  // Move in 1-degree increments
  while (currentShoulderR != targetR) {
    if (currentShoulderR < targetR) currentShoulderR++;
    else if (currentShoulderR > targetR) currentShoulderR--;
    
    shoulderRight.write(currentShoulderR);
    delay(delayMs);
  }
}

// Direct shoulder movement for the installed right shoulder servo
void moveShoulder(int angle) {
  if (!shoulderRight.attached()) shoulderRight.attach(PIN_SHOULDER_R);
  currentShoulderR = angle;
  shoulderRight.write(currentShoulderR);
}

void moveElbow(int angle) {
  if (!elbowLeft.attached()) elbowLeft.attach(PIN_ELBOW_L);
  if (!elbowRight.attached()) elbowRight.attach(PIN_ELBOW_R);
  currentElbowL = angle;
  currentElbowR = 180 - angle;
  elbowLeft.write(angle);
  elbowRight.write(180 - angle);
}

// Smooth elbow movement - moves both servos gradually to target positions
void moveElbowSmooth(int targetL, int targetR) {
  if (!elbowLeft.attached()) elbowLeft.attach(PIN_ELBOW_L);
  if (!elbowRight.attached()) elbowRight.attach(PIN_ELBOW_R);
  
  // Move in 1-degree increments
  while (currentElbowL != targetL || currentElbowR != targetR) {
    if (currentElbowL < targetL) currentElbowL++;
    else if (currentElbowL > targetL) currentElbowL--;
    
    if (currentElbowR < targetR) currentElbowR++;
    else if (currentElbowR > targetR) currentElbowR--;
    
    elbowLeft.write(currentElbowL);
    elbowRight.write(currentElbowR);
    delay(SERVO_DELAY);
  }
}

// Combined shoulder + elbow smooth movement (all 4 servos simultaneously)
void moveArmSmooth(int targetSL, int targetSR, int targetEL, int targetER, int delayMs = SERVO_DELAY) {
  if (!shoulderRight.attached()) shoulderRight.attach(PIN_SHOULDER_R);
  if (!elbowLeft.attached()) elbowLeft.attach(PIN_ELBOW_L);
  if (!elbowRight.attached()) elbowRight.attach(PIN_ELBOW_R);
  
  int startShoulderR = currentShoulderR;
  int startElbowL = currentElbowL;
  int startElbowR = currentElbowR;
  int shoulderSteps = abs(targetSR - startShoulderR);
  int elbowLeftSteps = abs(targetEL - startElbowL);
  int elbowRightSteps = abs(targetER - startElbowR);
  int totalSteps = shoulderSteps;
  if (elbowLeftSteps > totalSteps) totalSteps = elbowLeftSteps;
  if (elbowRightSteps > totalSteps) totalSteps = elbowRightSteps;

  currentShoulderL = targetSL;

  if (totalSteps == 0) {
    return;
  }

  // Interpolate all arm joints across the same timeline so they finish together.
  // Use floating-point progress to ensure perfectly smooth, simultaneous tracking
  for (int step = 1; step <= totalSteps; step++) {
    float progress = (float)step / (float)totalSteps;
    currentShoulderR = startShoulderR + ((targetSR - startShoulderR) * progress);
    currentElbowL = startElbowL + ((targetEL - startElbowL) * progress);
    currentElbowR = startElbowR + ((targetER - startElbowR) * progress);

    shoulderRight.write(currentShoulderR);
    elbowLeft.write(currentElbowL);
    elbowRight.write(currentElbowR);
    delay(delayMs);
  }
}

// Arm-down: keep the elbow in the safe 90->0 / 90->180 corridor and let it lead the shoulder.
void moveArmDownLevel(int targetSL, int targetSR, int targetEL, int targetER, int delayMs = SERVO_DELAY) {
  if (!shoulderRight.attached()) shoulderRight.attach(PIN_SHOULDER_R);
  if (!elbowLeft.attached()) elbowLeft.attach(PIN_ELBOW_L);
  if (!elbowRight.attached()) elbowRight.attach(PIN_ELBOW_R);

  int startShoulderR = currentShoulderR;
  int startElbowL = currentElbowL;
  int startElbowR = currentElbowR;
  
  // Total steps based on the largest movement to ensure smooth interpolation
  int shoulderSteps = abs(targetSR - startShoulderR);
  int elbowLeftSteps = abs(targetEL - startElbowL);
  int elbowRightSteps = abs(targetER - startElbowR);
  
  int totalSteps = shoulderSteps;
  if (elbowLeftSteps > totalSteps) totalSteps = elbowLeftSteps;
  if (elbowRightSteps > totalSteps) totalSteps = elbowRightSteps;

  currentShoulderL = targetSL;

  if (totalSteps == 0) {
    return;
  }

  // Pure linear interpolation for all joints exactly synchronized to the same timeline.
  // We clamp the elbow values tightly so they can never exceed their intended start/end bounds.
  // Left elbow should go 90 -> 0. We constrain it to never go above 90.
  // Right elbow should go 90 -> 180. We constrain it to never go below 90.
  for (int step = 1; step <= totalSteps; step++) {
    float progress = (float)step / (float)totalSteps;
    
    currentShoulderR = startShoulderR + (int)round((targetSR - startShoulderR) * progress);
    currentElbowL = startElbowL + (int)round((targetEL - startElbowL) * progress);
    currentElbowR = startElbowR + (int)round((targetER - startElbowR) * progress);

    // Hard mechanical safety clamp for 'a' (Arm Down) trajectory
    if (currentElbowL > 90) currentElbowL = 90;
    if (currentElbowL < 0) currentElbowL = 0;
    
    if (currentElbowR < 90) currentElbowR = 90;
    if (currentElbowR > 180) currentElbowR = 180;

    shoulderRight.write(currentShoulderR);
    elbowLeft.write(currentElbowL);
    elbowRight.write(currentElbowR);
    delay(delayMs);
  }
}


// --- HELPER FUNCTIONS FOR DRIVE ---
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
  // 1. READ COMMANDS FROM PI
  if (Serial.available() > 0) {
    command = Serial.read();
    lastCommandTime = millis(); // Reset safety timer
    
    // Interpret Drive Command
    if (command == 'F') setMotors(-int(left_scale * motorSpeed), int(right_scale * motorSpeed));      
    else if (command == 'B') setMotors(int(left_scale * motorSpeed), -int(right_scale * motorSpeed)); 
    else if (command == 'L') setMotors(int(left_scale * motorSpeed), int(right_scale * motorSpeed));  
    else if (command == 'R') setMotors(-int(left_scale * motorSpeed), -int(right_scale * motorSpeed));
    else if (command == 'S') stopMotors();                        
    
    // Interpret Speed Command
    else if (command == '1') motorSpeed = 100; // Increased from 100 for more turning torque
    else if (command == '2') motorSpeed = 130;
    else if (command == '3') motorSpeed = 255;
    
    // Interpret Arm Commands
    else if (command == 'U') {  // Shoulder UP - smooth movement
      moveShoulderSmooth(0, 30);  // Slower lift for better stability
    }
    else if (command == 'D') {  // Shoulder DOWN - smooth movement
      moveShoulderSmooth(130);  // Right: 0->130
    }
    else if (command == 'E') {  // Elbow UP - smooth movement
      moveElbowSmooth(0, 180);  // Left: 90->0, Right: 90->180
    }
    else if (command == 'e') {  // Elbow DOWN - smooth movement
      moveElbowSmooth(90, 90);  // Left: 0->90, Right: 180->90
    }
    else if (command == 'f') {  // Elbow DROP - smooth movement
      moveElbowSmooth(120, 60); // Left: 90->120, Right: 90->60
    }
    else if (command == 'g') {  // ARM DROP pose - smooth movement
      // Pass a custom delay of 45ms (3x slower than default 15ms) for a softer landing
      moveArmSmooth(currentShoulderL, 30, 120, 60, 45); 
      // Ensure the arm has settled completely to avoid mechanical overlap
      delay(200); 
      
      // The code waits above until movement gets to the target before executing the next line 
      // ensuring wrist opens ONLY after ARM has fully reached the drop position
      if (!wrist.attached()) wrist.attach(PIN_WRIST);
      wrist.write(0); // Then open the wrist gate
    }
    else if (command == 'A') {  // ARM UP: Shoulder UP + Elbow DOWN (horizontal to ground)
      moveArmSmooth(currentShoulderL, 0, 90, 90, 20);  // Right shoulder up, Elbow down
    }
    else if (command == 'a') {  // ARM DOWN: Shoulder DOWN + Elbow UP (return to start)
      moveArmSmooth(currentShoulderL, 130, 0, 180, 20);  // Use identical smooth tracking algorithm as 'g'
    }
    else if (command == 'J') {  // Custom Slow Staggered Down
      moveShoulderSmooth(60, 30);  // Step 1: Shoulder goes 0->60
      //moveArmSmooth(currentShoulderL, 60, 0, 180, 30); // Step 2: Elbow goes 90->0/180
      moveElbowSmooth(0, 180); // Ensure elbow finishes its movement before the shoulder continues down
      moveShoulderSmooth(130, 30); // Step 3: Shoulder continues 60->130
    }
    else if (command == 'H') {  // ARM VERTICAL: Shoulder UP + Elbow UP
      moveArmSmooth(currentShoulderL, 0, 0, 180);  // Right shoulder up, Elbow up
    }
    else if (command == 'O') {
      if (!wrist.attached()) wrist.attach(PIN_WRIST);
      wrist.write(0);
    }    // Gate OPEN
    else if (command == 'C') {
      if (!wrist.attached()) wrist.attach(PIN_WRIST);
      wrist.write(90);
    }    // Gate CLOSED
    
    // Interpret Individual Servo Angle Commands: 'v' <servoIndex> <angle>
    // e.g., "v 0 90"
    // Using parseInt() consumes the stream, which blocks loop and messes up millis() timing.
    else if (command == 'v') {
      int servoIdx = Serial.parseInt();
      int angle = Serial.parseInt();
      if (servoIdx == 0) { if (!shoulderLeft.attached()) shoulderLeft.attach(PIN_SHOULDER_L); shoulderLeft.write(angle); currentShoulderL = angle; }
      else if (servoIdx == 1) { if (!shoulderRight.attached()) shoulderRight.attach(PIN_SHOULDER_R); shoulderRight.write(angle); currentShoulderR = angle; }
      else if (servoIdx == 2) { if (!elbowLeft.attached()) elbowLeft.attach(PIN_ELBOW_L); elbowLeft.write(angle); currentElbowL = angle; }
      else if (servoIdx == 3) { if (!elbowRight.attached()) elbowRight.attach(PIN_ELBOW_R); elbowRight.write(angle); currentElbowR = angle; }
      else if (servoIdx == 4) { if (!wrist.attached()) wrist.attach(PIN_WRIST); wrist.write(angle); }
      
      // Update command time to prevent auto-kill right after parsing
      lastCommandTime = millis();
    }
    
    // Interpret Stop command specifically for Servos (New Option)
    else if (command == 'x') {
       // Detach to explicitly disable PWM signal and force stop
       shoulderLeft.detach();
       shoulderRight.detach();
       elbowLeft.detach();
       elbowRight.detach();
       wrist.detach();
    }
    // Update the master timer any time *any* valid character is received
    // Even if the payload parsing took a second, start the 2000ms clock from now
    lastCommandTime = millis();
  }
  
  // 2. SAFETY STOP (Watchdog)
  // If no new movement command is received for 3000 milliseconds, stop DC motors
  // Servos keep holding position - use 'x' command to explicitly detach them
  if (millis() - lastCommandTime > 3000) {
    stopMotors();  // Only stop DC motors, servos stay attached to hold position
  }

  // 3. READ SENSORS & SEND TO PI (Every 100ms)
  // DISABLED FOR SERVO TESTING - uncomment when using with full robot control
  
  static unsigned long lastSensorTime = 0;
  if (millis() - lastSensorTime > 100) {
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