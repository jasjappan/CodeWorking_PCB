#define TRIGGER_PIN 29  // TX7 — drives sensor's RX

void setup() {
  Serial.begin(115200);      // USB debug
  Serial7.begin(115200);     // UART7 for sensor

  pinMode(TRIGGER_PIN, OUTPUT);
  digitalWrite(TRIGGER_PIN, HIGH);  // Idle HIGH
}

void loop() {
  triggerSensor();
  
  int distance = readSensor();
  
  if (distance >= 0) {
    Serial.print("Distance: ");
    Serial.print(distance);
    Serial.println(" mm");
  } else {
    Serial.println("Read error (bad checksum or timeout)");
  }

  delay(25);  // ≥25ms between triggers
}

void triggerSensor() {
  digitalWrite(TRIGGER_PIN, LOW);
  delayMicroseconds(500);
  digitalWrite(TRIGGER_PIN, HIGH);
}

int readSensor() {
  uint8_t buffer[4];
  int index = 0;

  unsigned long timeout = millis() + 20;  // 20ms timeout

  // Wait for frame head 0xFF
  while (millis() < timeout) {
    if (Serial7.available()) {
      uint8_t b = Serial7.read();
      if (b == 0xFF) {
        buffer[0] = b;
        index = 1;
        break;
      }
    }
  }

  if (index == 0) return -1;  // Timeout, no frame head found

  // Read remaining 3 bytes (Data_H, Data_L, Checksum)
  timeout = millis() + 10;
  while (index < 4 && millis() < timeout) {
    if (Serial7.available()) {
      buffer[index++] = Serial7.read();
    }
  }

  if (index < 4) return -1;  // Incomplete frame

  // Validate checksum
  uint8_t checksum = (buffer[0] + buffer[1] + buffer[2]) & 0xFF;
  if (checksum != buffer[3]) {
    Serial.print("Checksum fail — expected: 0x");
    Serial.print(checksum, HEX);
    Serial.print("  got: 0x");
    Serial.println(buffer[3], HEX);
    return -1;
  }

  // Calculate distance
  int distance = buffer[1] * 256 + buffer[2];
  return distance;
}