#define encoderPin1 10
#define encoderPin2 11

#define PWM_PIN 8
#define DIR_PIN 9

#define CPR 2500
#define COUNTS_PER_REV (CPR * 4)

volatile long encoderValue = 0;
volatile int lastEncoded = 0;

// User सेट
int pwmValue = 10;   // 👈 change this (0–255)

float currentRPM = 0;

// Timing
unsigned long lastTime = 0;
long lastCount = 0;

void setup() {
  Serial.begin(115200);

  pinMode(encoderPin1, INPUT);   // external pullups
  pinMode(encoderPin2, INPUT);

  pinMode(PWM_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);

  digitalWrite(DIR_PIN, LOW);

  analogWrite(PWM_PIN, pwmValue);

  attachInterrupt(digitalPinToInterrupt(encoderPin1), updateEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderPin2), updateEncoder, CHANGE);
}

void loop() {

  unsigned long currentTime = millis();

  if (currentTime - lastTime >= 100) {

    // atomic read
    noInterrupts();
    long count = encoderValue;
    interrupts();

    long delta = count - lastCount;

    // RPM calculation
    currentRPM = (delta * 600.0) / COUNTS_PER_REV;

    lastCount = count;
    lastTime = currentTime;

    // Log
    Serial.print("PWM: ");
    Serial.print(pwmValue);
    Serial.print(" | RPM: ");
    Serial.print(currentRPM);
    Serial.print(" | Counts: ");
    Serial.println(count);
  }
}

void updateEncoder() {
  int MSB = digitalRead(encoderPin1);
  int LSB = digitalRead(encoderPin2);

  int encoded = (MSB << 1) | LSB;
  int sum = (lastEncoded << 2) | encoded;

  if (sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011)
    encoderValue++;
  if (sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000)
    encoderValue--;

  lastEncoded = encoded;
}