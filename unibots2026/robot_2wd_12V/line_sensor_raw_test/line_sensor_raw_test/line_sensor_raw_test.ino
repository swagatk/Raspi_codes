// line_sensor_raw_test.ino
// Standalone test for a line-following sensor on Arduino digital pin 2.
// Prints raw digital state so you can confirm wiring and sensor behavior.

const int LINE_SENSOR_PIN = 2;      // Sensor DO -> D2
const unsigned long READ_MS = 100;  // Print rate (10 Hz)

unsigned long lastReadMs = 0;

void setup() {
  Serial.begin(115200);
  pinMode(LINE_SENSOR_PIN, INPUT);

  Serial.println("Line Sensor Raw Test");
  Serial.println("Format: LINE_RAW,<0_or_1>,<LOW_or_HIGH>");
}

void loop() {
  unsigned long now = millis();
  if (now - lastReadMs >= READ_MS) {
    lastReadMs = now;

    int raw = digitalRead(LINE_SENSOR_PIN);  // 0 or 1

    Serial.print("LINE_RAW,");
    Serial.print(raw);
    Serial.print(",");
    if (raw == LOW) {
      Serial.println("LOW");
    } else {
      Serial.println("HIGH");
    }
  }
}
