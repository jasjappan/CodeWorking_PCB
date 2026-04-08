#include <Wire.h>
#include "Updated_MS5837.hpp"

MS5837 sensor;
float fluidDensity = 997;

void setup() {
  Serial.begin(115200);
  Serial.println("Starting Bar30 Sensor...");

  Wire2.begin();  // SDA=pin25, SCL=pin24

  while (!sensor.init(Wire2)) {  // ← pass Wire2 directly
    Serial.println("Waiting for sensor... Check wiring!");
    delay(1000);
  }

  sensor.setModel(MS5837::MS5837_30BA);
  sensor.setFluidDensity(fluidDensity);
  Serial.println("Sensor Initialized!");
}

void loop() {
  sensor.read();
  Serial.print("Pressure: ");
  Serial.print(sensor.pressure());
  Serial.print(" mbar  |  ");
  Serial.print("Depth: ");
  Serial.print(sensor.depth());
  Serial.print(" m  |  ");
  Serial.print("Temp: ");
  Serial.print(sensor.temperature());
  Serial.println(" C");
  delay(500);
}
