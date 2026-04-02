#define TRIGGER_PIN 29

void setup() {
  Serial.begin(115200);
  Serial7.begin(115200);
  pinMode(TRIGGER_PIN, OUTPUT);
  digitalWrite(TRIGGER_PIN, HIGH);
  delay(500);  // Longer stabilize on boot
}

void loop() {
  // Flush any stale bytes before triggering
  while (Serial7.available()) Serial7.read();

  triggerSensor();
  debugRawBytes();

  delay(25);
}

void triggerSensor() {
  digitalWrite(TRIGGER_PIN, LOW);
  delayMicroseconds(500);
  digitalWrite(TRIGGER_PIN, HIGH);
  delay(10);  // Wait slightly longer than T2
}

void debugRawBytes() {
  uint8_t buffer[16];
  int index = 0;

  unsigned long timeout = millis() + 30;  // Generous 30ms window
  while (millis() < timeout && index < 16) {
    if (Serial7.available()) {
      buffer[index++] = Serial7.read();
    }
  }

  Serial.print("Bytes received: ");
  Serial.println(index);

  if (index == 0) {
    Serial.println("  >> Nothing received — check wiring TX/RX swap, voltage, baud rate");
    return;
  }

  for (int i = 0; i < index; i++) {
    Serial.print("  [");
    Serial.print(i);
    Serial.print("] = 0x");
    if (buffer[i] < 0x10) Serial.print("0");
    Serial.println(buffer[i], HEX);
  }

  // Try to find 0xFF frame header anywhere in buffer
  for (int i = 0; i <= index - 4; i++) {
    if (buffer[i] == 0xFF) {
      Serial.print("  >> Frame found at index ");
      Serial.println(i);

      uint8_t calcSum = (buffer[i] + buffer[i+1] + buffer[i+2]) & 0xFF;
      Serial.print("  >> Checksum calc: 0x");
      Serial.print(calcSum, HEX);
      Serial.print("  got: 0x");
      Serial.println(buffer[i+3], HEX);

      if (calcSum == buffer[i+3]) {
        int dist = buffer[i+1] * 256 + buffer[i+2];
        Serial.print("  >> Distance: ");
        Serial.print(dist);
        Serial.println(" mm  VALID");
      } else {
        Serial.println("  >> Checksum MISMATCH");
      }
    }
  }
}