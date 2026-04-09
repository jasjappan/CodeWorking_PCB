#include<Wire.h>
#include<Updated_MS5837.hpp>

#define EncoderPin1 10
#define EncoderPin2 11
volatile long encoderValue = 0;
volatile long lastEncoded = 0;

#define PWM_PIN_A 6
#define DIR_PIN_A 7
#define PWM_PIN_B 8
#define DIR_PIN_B 9

MS5837 barSensor;
float fluidDensity = 997;

#define TriggerPin 29 //TX pin for ranging 1 
uint8_t buf[16];
int range_rx_index = 0;
bool trigDone = false;
bool readDone = false;
long unsigned int T1_delay;
int cmd_i;

//Sensor data to Raspberry Pi ( writtten in IntervalInterrupts in Seconds instead of Hz )
// int barTP_tx = 20Hz;
// int rangingTP_tx = 50Hz;
// int imuTP_tx = 400Hz;

//Timer for sensor functions ( read from sensors )
int barTimer = 0;
int rangingTimer = 0;
int imuSensor = 0;

struct
{
  int temp;
  int depth;
  int pressure;
}barPacket;

struct
{
  int long distance;
}rangingPacket;


// struct
// {

// }imuPacket;

struct
{
  long encoder_copy;
}encoderPacket;


//Sampling time period for each Sensor
long unsigned int barTP_rx =  330;//30Hz;
long unsigned int rangingTP_rx = 330; //30Hz;
// int imuTP_rx =  //200Hz;       //NEED TO GET CODE FOR IMU

//Teensy to Raspberry Pi
IntervalTimer barTimer_tx;
IntervalTimer rangingTimer_tx;
IntervalTimer encoderTimer_tx;
IntervalTimer Pi_rx;

int receivedCMD[10];   //value type and array length needs to be changed to fit the incoming cmd(dont know what it is currently)

long unsigned int timestamp;
long unsigned int time_period = 0;


void setup()
{
  //UART Lines
  Serial.begin(115200);   //for Tx/Rx with Raspberry Pi
  Serial7.begin(115200);  //for receiving data from Ranging sensor (at given frequency)
  pinMode(TriggerPin, OUTPUT);    //Done to allow the Trigger Pin to send the low pulse for starting data sending
  digitalWrite(TriggerPin, HIGH);
  delay(500);
  trigDone = false;
  T1_delay = 0;

  //I2C Lines
  Wire2.begin();          //for receiving data from Pressure sensor
  while(!barSensor.init(Wire2))
  {
    delay(1000);
  }
  barSensor.setModel(MS5837::MS5837_30BA);
  barSensor.setFluidDensity(fluidDensity);

  //Interrupt Sensor(Encoder as can't miss a single pulse)
  pinMode(EncoderPin1, INPUT);
  pinMode(EncoderPin2, INPUT);
  attachInterrupt(digitalPinToInterrupt(EncoderPin1), updateEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(EncoderPin2), updateEncoder, CHANGE);

  pinMode(PWM_PIN_A, OUTPUT);
  pinMode(DIR_PIN_A, OUTPUT);
  pinMode(PWM_PIN_B, OUTPUT);
  pinMode(DIR_PIN_B, OUTPUT);

  //Interrupts Priority   !!change priority according to the rule => Highest for encoder receiving; then after that; higher priority for less frequently occuring events i.e. those with bigger time period
  // encoderTimer_tx.priority(1);
  // // Pi_rx.priority(2);
  // barTimer_tx.priority(2);
  // rangingTimer_tx.priority(3);
  // barTimer_tx.priority(4);

  //Interrupts Begin
  barTimer_tx.begin(barSend, 50000); //20Hz
  rangingTimer_tx.begin(rangingSend, 66000);  //15Hz
  encoderTimer_tx.begin(encoderSend, 70000);   //70Khz
  Pi_rx.begin(piReceive, 50000);              //20Hz

}

//use timer interrupts for all sending data parts
//use an interrupt for receiving data from Raspberry Pi
//Priority Order(Highest to Lowest):
//Encoder pulse Interrupt (updateEncoder)
//slower sampling frequency sensors
//....
//Receiving data from Raspberry Pi
//....
//higher sampling frequency sensors
//main loop

void loop()
{
  timestamp = micros();
  // Serial.println(timestamp);
  // Serial.println(millis());
  // Serial.println(micros());
  // Serial.println("HELLO");
  if ( millis() - barTimer >= barTP_rx )
  {
    barTimer = millis();
    barPacket.depth = (int)barSensor.depth();
    barPacket.temp = (int)barSensor.temperature();
    barPacket.pressure = (int)barSensor.pressure();
    // barSensorRead();
  }

  if ( (millis() - rangingTimer >= rangingTP_rx) && (!trigDone) )
  {
    rangingTimer = millis();
    digitalWrite(TriggerPin, LOW);
    delayMicroseconds(500);
    digitalWrite(TriggerPin, HIGH);
    T1_delay = millis();
    trigDone = true;
    range_rx_index = 0;
    // triggerSensor();
    // debugRawBytes();
  }

  
  if ( trigDone && ((millis() - T1_delay) >= 10) )
  {
    if ( range_rx_index < 16 )      //replaced while with if
    {
      if ( Serial7.available() )
      {
        buf[range_rx_index++] = Serial7.read();
      }
    }
    else
      readDone = true;
  }

  if ( readDone )
  {
    if ( buf[0] == 0xFF )
    {
      int calcsum = (buf[0] + buf[1] + buf[2]) & 0xFF;
      if (calcsum == buf[3])
      {
        rangingPacket.distance = buf[1] * 256 + buf[2];
        trigDone = false;
        readDone = false;
      }
      else
      {
        //CHECKSUM ERROR
        trigDone = false;
        readDone = false;
      }
    }
    //HEADER NOT MATCHED
    trigDone = false;
    readDone = false;
  }

  // if (Serial.available())
  // {
  //   receivedCMD[cmd_i++] = Serial.read();
  //   //perform actions using the received byte
  // }
  // else
  //   cmd_i = 0;
  if ( micros() - timestamp >= time_period )
    time_period = micros() - timestamp;
  Serial.println(time_period);
  // Serial.println(millis() - timestamp);
  // Serial.println(millis());
  // Serial.println(micros());
}

void barSend ()
{
  uint8_t buf[] = {barPacket.depth, barPacket.pressure, barPacket.temp};    //THIS WILL CAUSE PROBLEMS LATER, PLEASE FIX IT LATER ACCORDINGF TO RX DATA FROM RASPBERRY PI
  // Serial.write(buf, sizeof(buf));
  // Serial.println("bar packet");
  // Serial.println(barPacket.temp);
  // Serial.println();
}

void rangingSend ()
{
  // Serial.write(rangingPacket.distance);
  // Serial.println("ranging packet");
  // Serial.println(rangingPacket.distance);
  // Serial.println();
}

void encoderSend ()
{
  noInterrupts();
  encoderPacket.encoder_copy = encoderValue;
  interrupts();
  // Serial.write(encoderPacket.encoder_copy);
  // Serial.println("Encoder packet");
  // Serial.print(encoderPacket.encoder_copy);
  // Serial.println();
}

void piReceive ()
{
  if (Serial.available())
  {
    char incomingByte = Serial.read();
    if (incomingByte == 'e')
    {
      Serial.println(encoderPacket.encoder_copy);
    }
    else if (incomingByte == 'b')
    {
      Serial.println(barPacket.depth);
    }
    else if (incomingByte == 'r')
    {
      Serial.println(rangingPacket.distance);
    }
    else if (incomingByte == 'm')
    {
      char motor = Serial.read();
      int pwm = 0;
      int byte;
      // while ( (byte = Serial.read()) != ' ' )
      for (int i = 0, base = 1; (byte = Serial.read()) != ' '; i++)
      {
        if ( i == 0)
          base = 1;
        else
          base *= 10;
        pwm += (byte - '0') * base;
        // Serial.println(pwm);
      }
      int dir = Serial.read() - '0';
      if ( motor == 'A' )
      {
        digitalWrite(DIR_PIN_A, dir);
        analogWrite(PWM_PIN_A, pwm);
        Serial.println(dir);
        Serial.println(pwm);
      }
      else if (motor == 'B')
      {
        digitalWrite(DIR_PIN_B, dir);
        analogWrite(PWM_PIN_B, pwm);
        Serial.println(dir);
        Serial.println(pwm);
      }
      else
      {
        Serial.print("Incorrect command: ");
        Serial.println(motor);
      }
    }
    else if (incomingByte != '\n' && incomingByte != '\r')
    {
      Serial.print("Incorrect command send: ");
      Serial.println(incomingByte);
    }
  }
}


// //BAR SENSOR READ
// void barSensorRead ()
// {
//   barPacket.depth = barSensor.depth;
//   barPacket.temp = barSensor.temp;
//   barPacket.pressure = barSensor.pressure;
// }

// //RANGING SENSOR READ
// void triggerSensor() {
//   digitalWrite(TRIGGER_PIN, LOW);
//   delayMicroseconds(500);
//   digitalWrite(TRIGGER_PIN, HIGH);
//   delay(10);  // Wait slightly longer than T2
// }

// void debugRawBytes() {
//   uint8_t buffer[16];
//   int range_rx_index = 0;

//   unsigned long timeout = millis() + 30;  // Generous 30ms window
//   while (millis() < timeout && range_rx_index < 16) {
//     if (Serial7.available()) {
//       buffer[range_rx_index++] = Serial7.read();
//     }
//   }

//   Serial.print("Bytes received: ");
//   Serial.println(range_rx_index);

//   if (range_rx_index == 0) {
//     Serial.println("  >> Nothing received — check wiring TX/RX swap, voltage, baud rate");
//     return;
//   }

//   for (int i = 0; i < range_rx_index; i++) {
//     Serial.print("  [");
//     Serial.print(i);
//     Serial.print("] = 0x");
//     if (buffer[i] < 0x10) Serial.print("0");
//     Serial.println(buffer[i], HEX);
//   }

//   // Try to find 0xFF frame header anywhere in buffer
//   for (int i = 0; i <= range_rx_index - 4; i++) {
//     if (buffer[i] == 0xFF) {
//       Serial.print("  >> Frame found at range_rx_index ");
//       Serial.println(i);

//       uint8_t calcSum = (buffer[i] + buffer[i+1] + buffer[i+2]) & 0xFF;
//       Serial.print("  >> Checksum calc: 0x");
//       Serial.print(calcSum, HEX);
//       Serial.print("  got: 0x");
//       Serial.println(buffer[i+3], HEX);

//       if (calcSum == buffer[i+3]) {
//         int dist = buffer[i+1] * 256 + buffer[i+2];
//         Serial.print("  >> Distance: ");
//         Serial.print(dist);
//         Serial.println(" mm  VALID");
//       } else {
//         Serial.println("  >> Checksum MISMATCH");
//       }
//     }
//   }
// }


//ENCODER SENSOR READING
void updateEncoder()
{
  int MSB = digitalRead(EncoderPin1);
  int LSB = digitalRead(EncoderPin2);

  int encoded = (MSB << 1) | LSB;
  int sum = (lastEncoded << 2) | encoded;

  if (sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011)
    encoderValue++;
  if (sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000)
    encoderValue--;

  lastEncoded = encoded;
}