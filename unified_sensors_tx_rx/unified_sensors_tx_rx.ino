#include<Wire.h>
#include<Updated_MS5837.hpp>
#include <Adafruit_BNO08x.h>

//Circular Buffer
//--------------------------------
#define SIZE_OF_BUFFER 50

int buf_length = 0;
int readIndex = 0;
int writeIndex = 0;

char cb[SIZE_OF_BUFFER];
//---------------------------------


//CMD Buffer
//---------------------------------
char cmd_buf[SIZE_OF_BUFFER];
char state[12] = "IDLE";
int cmd_buf_ind = 0;
//---------------------------------



//2500RPM-ABZ Rotary Encoder
//---------------------------------
#define EncoderPin1_A 10
#define EncoderPin2_A 11
volatile long encoderValue_A = 0;
volatile long lastEncoded_A = 0;

#define EncoderPin1_B 33
#define EncoderPin2_B 36
volatile long encoderValue_B = 0;
volatile long lastEncoded_B = 0;
//----------------------


//md220A Motor Driver
//----------------------
#define PWM_PIN_A 6
#define DIR_PIN_A 7
#define PWM_PIN_B 8
#define DIR_PIN_B 9
//-----------------------


//Bar30 Pressure Sensor
//------------------------
MS5837 barSensor;
float fluidDensity = 997;
//------------------------


//LYP-08 Ranging Sensor
//--------------------------
#define TriggerPin 29 //TX pin for ranging 1 
uint8_t buf[16];
int range_rx_index = 0;
bool trigDone = false;
bool readDone = false;
long unsigned int T1_delay;
int cmd_i;
//-----------------------------


//BNO08x IMU
//--------------------------------
#define BNO08X_RESET -1
Adafruit_BNO08x  bno08x(BNO08X_RESET);
sh2_SensorValue_t imu_value;
//---------------------------------



//Sensor data to Raspberry Pi ( writtten in IntervalInterrupts in Seconds instead of Hz )
// int barTP_tx = 20Hz;
// int rangingTP_tx = 50Hz;
// int imuTP_tx = 400Hz;

//Timer for sensor functions ( read from sensors )
//---------------------------
int barTimer = 0;
int rangingTimer = 0;
int imuTimer = 0;
//---------------------------


//Packets
//---------------------------
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


struct
{
  float real;
  float i;
  float j;
  float k;
}imuPacket;

struct
{
  long encoder_copy;
}encoderPacket_A, encoderPacket_B;
//-----------------------------

struct sensorBig        //Stores all sensors' packet -- NEW
{
  int imu1[3];
  int imu2[3];
  int bar;
  int rang1;
  int rang2;
  int enc1;
  int enc2;
}sensorPacket;

//Sampling time period for each Sensor
//-----------------------------
long unsigned int barTP_rx =  33;//30Hz;
long unsigned int rangingTP_rx = 33; //30Hz;
long unsigned int imuTP_rx = 5;
//------------------------------


//Timer Interrupts declared
//----------------------------
IntervalTimer barTimer_tx;
IntervalTimer rangingTimer_tx;
IntervalTimer encoderTimer_tx_A, encoderTimer_tx_B;
IntervalTimer Pi_rx;
IntervalTimer cmd_rx;     //To store incoming bytes in the circ buffer for command reading    --NEW
//-----------------------------


int receivedCMD[10];   //value type and array length needs to be changed to fit the incoming cmd(dont know what it is currently)


//Time Period of one void loop() iteration
//------------------------------
long unsigned int timestamp;
long unsigned int time_period = 0;
//------------------------------


void setup()
{
  //UART Lines
  Serial.begin(115200);                               //for Tx/Rx with Raspberry Pi
  Serial.println("Setup started");

  Serial7.begin(115200);                              //RANGING SENSOR
  pinMode(TriggerPin, OUTPUT);                        //Used to send low pulse for starting communication
  digitalWrite(TriggerPin, HIGH);
  delay(500);
  trigDone = false;
  T1_delay = 0;                                       //To wait after sending low pulse to complete receiving full data

  //I2C Lines
  Wire2.begin();                                      //PRESSURE SENSOR
  while(!barSensor.init(Wire2))
  {
    delay(1000);
  }
  barSensor.setModel(MS5837::MS5837_30BA);
  barSensor.setFluidDensity(fluidDensity);

  //Interrupt Sensor(Encoder as can't miss a single pulse)
  // pinMode(EncoderPin1_A, INPUT);                        //ENCODER A
  // pinMode(EncoderPin2_A, INPUT);
  // attachInterrupt(digitalPinToInterrupt(EncoderPin1_A), updateEncoder_A, CHANGE);
  // attachInterrupt(digitalPinToInterrupt(EncoderPin2_A), updateEncoder_A, CHANGE);

  pinMode(EncoderPin1_B, INPUT);                        //ENCODER B
  pinMode(EncoderPin2_B, INPUT);
  attachInterrupt(digitalPinToInterrupt(EncoderPin1_B), updateEncoder_B, CHANGE);
  attachInterrupt(digitalPinToInterrupt(EncoderPin2_B), updateEncoder_B, CHANGE);


  pinMode(PWM_PIN_A, OUTPUT);                         //MOTOR DRIVER
  pinMode(DIR_PIN_A, OUTPUT);
  pinMode(PWM_PIN_B, OUTPUT);
  pinMode(DIR_PIN_B, OUTPUT);


  Wire1.begin();                                      //IMU_A
  // Try to initialize!
  while (!bno08x.begin_I2C(0x4A, &Wire1)) {
    Serial.println("Failed to find BNO08x chip");     //Need to reboot Teensy if it fails
    // while (1) { delay(10); }      
    delay(10);          
  }
  Serial.println("BNO08x Found!");

  for (int n = 0; n < bno08x.prodIds.numEntries; n++) {
    Serial.print("Part ");
    Serial.print(bno08x.prodIds.entry[n].swPartNumber);
    Serial.print(": Version :");
    Serial.print(bno08x.prodIds.entry[n].swVersionMajor);
    Serial.print(".");
    Serial.print(bno08x.prodIds.entry[n].swVersionMinor);
    Serial.print(".");
    Serial.print(bno08x.prodIds.entry[n].swVersionPatch);
    Serial.print(" Build ");
    Serial.println(bno08x.prodIds.entry[n].swBuildNumber);
  }

  setReports();                                     //Important: Used to set what we will get from IMU

  Serial.println("Reading events");
  delay(100);

  //Interrupts Priority   !!change priority according to the rule => Highest for encoder receiving; then after that; higher priority for less frequently occuring events i.e. those with bigger time period
  // encoderTimer_tx_A.priority(1);
  // // Pi_rx.priority(2);
  // barTimer_tx.priority(2);
  // rangingTimer_tx.priority(3);
  // barTimer_tx.priority(4);

  //Interrupts Begin
  barTimer_tx.begin(barSend, 50000); //20Hz         //INTERRUPTS setting timers

  rangingTimer_tx.begin(rangingSend, 66000);  //15Hz

  // encoderTimer_tx_A.begin(encoderSend_A, 70000);   //70Khz
  encoderTimer_tx_B.begin(encoderSend_B, 70000);   //70Khz
  
  // Pi_rx.begin(piReceive, 50000);              //20Hz

  cmd_rx.begin(rx_parser, 25000);   //40Hz
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

// Here is where you define the sensor outputs you want to receive
void setReports(void) {
  Serial.println("Setting desired reports");
  if (! bno08x.enableReport(SH2_GAME_ROTATION_VECTOR)) {
    Serial.println("Could not enable game vector");
  }
  else
    Serial.println("game vector enabled");
}


///////////////////////////////////////////////////////////////////////////////////////////

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



  if ( millis() - imuTimer >= imuTP_rx )
  {
    imuTimer = millis();
    bno08x.getSensorEvent(&imu_value);
    // Serial.println("updating imu value");
    // Serial.println(imu_value.sensorId);
    switch (imu_value.sensorId)
    {
      case SH2_GAME_ROTATION_VECTOR:
        imuPacket.real = imu_value.un.gameRotationVector.real;
        imuPacket.i = imu_value.un.gameRotationVector.i;
        imuPacket.j = imu_value.un.gameRotationVector.j;
        imuPacket.k = imu_value.un.gameRotationVector.k;
        // Serial.println("Setting IMU packet");
        // Serial.println(imu_value.un.gameRotationVector.real);
        // Serial.println(imu_value.un.gameRotationVector.i);
        // Serial.println(imu_value.un.gameRotationVector.j);
        // Serial.println(imu_value.un.gameRotationVector.k);
        break;
    }
  }


  //Main Packet Building
  
  // sendMainPacket_tx();


  // if (Serial.available())
  // {
  //   receivedCMD[cmd_i++] = Serial.read();
  //   //perform actions using the received byte
  // }
  // else
  //   cmd_i = 0;

  rx_parser();              //to go through stored incoming bytes in circular buffer cb

  if ( micros() - timestamp >= time_period )
    time_period = micros() - timestamp;
  // Serial.println(time_period);
  // Serial.println(millis() - timestamp);
  // Serial.println(millis());
  // Serial.println(micros());
}

//Circular Buffer Basic Operations
//-------------------------------------------------------
char cb_write ( char c )
{
  if (buf_length == SIZE_OF_BUFFER)
  {
    return '\0';  //for false
  }
  cb[writeIndex++] = c;
  buf_length++;
  if (writeIndex == SIZE_OF_BUFFER)
  {
    writeIndex = 0;
  }
  return c;     //if not equals \0 then true
}

char cb_read (void)
{
  if ( buf_length == 0)
  {
    return '\0';  //for false
  }
  char c = cb[readIndex++];
  buf_length--;
  if ( readIndex == SIZE_OF_BUFFER)
  {
    readIndex = 0;
  }
  return c;     //if not equals \0 then true
}
//------------------------------------------------------------


void rx_parser(void)  //to get read from cb and execute a command if any, using a state machine
{
  char c;
  if (strcmp(state, "IDLE"))
  {
    if (cb_read() == '<')
    {
      // state = "CHECK_ST";
      strcpy(state, "CHECK_ST");
    }
    else
    {
      Serial.println("Error during parsing");
      // state = "IDLE";
      strcpy(state, "IDLE");
    }
  }
  if (strcmp(state, "CHECK_ST"))
  {
    if (cb_read() == 'S' && cb_read() == 'T' && cb_read() == '>')
    {
      // state = "START_PARS";
      strcpy(state, "START_PARS");
    }
    else
    {
      Serial.println("Parse error");
      // state = "IDLE";
      strcpy(state, "IDLE");
    }
  }
  if (strcmp(state, "START_PARS"))
  {
    if ((c = cb_read()) == '<')
    {
      // state = "CHECK_EN";
      strcpy(state, "CHECK_EN");
      // buf_write('\0')
      cmd_buf[cmd_buf_ind++] = '\0';
    }
    else
    {
      // buf_write( c );
      cmd_buf[cmd_buf_ind++] = c;
    }
  }
  if (strcmp(state, "CHECK_EN"))
  {
    if (cb_read() == 'E' && cb_read() == 'N' && cb_read() == '>')
    {
      // state = "IDLE";
      strcpy(state, "IDLE");
      // cmd_complete = true;      //tells void loop() that a full cmd is available to execute
      cmd_buf_ind = 0;
      cmd_exec(cmd_buf);
    }
    else
    {
      Serial.println("Parse error");
      // state = "IDLE";
      strcpy(state, "IDLE");
    }
  }
}


void cmd_exec (char buf[])
{
  char cmd[5];
  int i, j;
  int val[12] = {0};
  for ( i = 0 , j = 0 ; j < 4; i++)
  {
    cmd[j] = buf[i];
  }
  cmd[j] = '\0';

  // if ( cmd == "CTRL" )
  if ( strcmp(cmd, "CTRl") )
  {
    
    // int val[12] = {0};
    for ( unsigned int j = 0; j < sizeof(val)/sizeof(val[0]); j++ )
    {
      while ( buf[i++] != ',' && buf[i] != '\0' )
      {
        val[j] *= 10;
        val[j] = buf[i] - '0';
      }
    }

    analogWrite(PWM_PIN_A, val[0]);
    digitalWrite(DIR_PIN_A, val[1]);
    analogWrite(PWM_PIN_A, val[2]);
    digitalWrite(DIR_PIN_A, val[3]);
  }
}



void barSend ()
{
  uint8_t buf[] = {barPacket.depth, barPacket.pressure, barPacket.temp};    //THIS WILL CAUSE PROBLEMS LATER, PLEASE FIX IT LATER ACCORDING TO RX DATA FROM RASPBERRY PI
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

void encoderSend_A ()
{
  noInterrupts();
  encoderPacket_A.encoder_copy = encoderValue_A;
  interrupts();
  // Serial.write(encoderPacket_A.encoder_copy);
  // Serial.println("Encoder packet");
  // Serial.print(encoderPacket_A.encoder_copy);
  // Serial.println();
}

void encoderSend_B ()
{
  noInterrupts();
  encoderPacket_B.encoder_copy = encoderValue_B;
  interrupts();
  // Serial.write(encoderPacket_A.encoder_copy);
  // Serial.println("Encoder packet");
  // Serial.print(encoderPacket_A.encoder_copy);
  // Serial.println();
}

// void piReceive ()
// {
//   if (Serial.available())
//   {
//     char incomingByte = Serial.read();
//     if (incomingByte == 'e')
//     {
//       Serial.println(encoderPacket_B.encoder_copy);
//     }
//     else if (incomingByte == 'b')
//     {
//       Serial.println(barPacket.depth);
//     }
//     else if (incomingByte == 'r')
//     {
//       Serial.println(rangingPacket.distance);
//     }
//     else if (incomingByte == 'm')
//     {
//       char motor = Serial.read();
//       int pwm = 0;
//       int byte;
//       // while ( (byte = Serial.read()) != ' ' )
//       for (int i = 0, base = 1; (byte = Serial.read()) != ' '; i++)
//       {
//         if ( i == 0)
//           base = 1;
//         else
//           base *= 10;
//         pwm += (byte - '0') * base;
//         // Serial.println(pwm);
//       }
//       int dir = Serial.read() - '0';
//       if ( motor == 'A' )
//       {
//         digitalWrite(DIR_PIN_A, dir);
//         analogWrite(PWM_PIN_A, pwm);
//         Serial.println(dir);
//         Serial.println(pwm);
//       }
//       else if (motor == 'B')
//       {
//         digitalWrite(DIR_PIN_B, dir);
//         analogWrite(PWM_PIN_B, pwm);
//         Serial.println(dir);
//         Serial.println(pwm);
//       }
//       else
//       {
//         Serial.print("Incorrect command: ");
//         Serial.println(motor);
//       }
//     }
//     else if ( incomingByte == 'i' )
//     {
//       Serial.println(imuPacket.real);
//       Serial.println(imuPacket.i);
//       Serial.println(imuPacket.j);
//       Serial.println(imuPacket.k);
//     }
//     else if (incomingByte != '\n' && incomingByte != '\r')
//     {
//       Serial.print("Incorrect command send: ");
//       Serial.println(incomingByte);
//     }
//   }
// }



// uses cmd_complete to see if we have a full cmd to read
// starts reading the buffer
// uses keywords like IMU1 to know which member of sensorPacket to target and | to increase index of the array
// 



// void piReceive ()
// {
//   if (cmd_complete)
//   {
//     char incomingByte = Serial.read();
//     if (incomingByte == 'e')
//     {
//       Serial.println(encoderPacket_B.encoder_copy);
//     }
//     else if (incomingByte == 'b')
//     {
//       Serial.println(barPacket.depth);
//     }
//     else if (incomingByte == 'r')
//     {
//       Serial.println(rangingPacket.distance);
//     }
//     else if (incomingByte == 'm')
//     {
//       char motor = Serial.read();
//       int pwm = 0;
//       int byte;
//       // while ( (byte = Serial.read()) != ' ' )
//       for (int i = 0, base = 1; (byte = Serial.read()) != ' '; i++)
//       {
//         if ( i == 0)
//           base = 1;
//         else
//           base *= 10;
//         pwm += (byte - '0') * base;
//         // Serial.println(pwm);
//       }
//       int dir = Serial.read() - '0';
//       if ( motor == 'A' )
//       {
//         digitalWrite(DIR_PIN_A, dir);
//         analogWrite(PWM_PIN_A, pwm);
//         Serial.println(dir);
//         Serial.println(pwm);
//       }
//       else if (motor == 'B')
//       {
//         digitalWrite(DIR_PIN_B, dir);
//         analogWrite(PWM_PIN_B, pwm);
//         Serial.println(dir);
//         Serial.println(pwm);
//       }
//       else
//       {
//         Serial.print("Incorrect command: ");
//         Serial.println(motor);
//       }
//     }
//     else if ( incomingByte == 'i' )
//     {
//       Serial.println(imuPacket.real);
//       Serial.println(imuPacket.i);
//       Serial.println(imuPacket.j);
//       Serial.println(imuPacket.k);
//     }
//     else if (incomingByte != '\n' && incomingByte != '\r')
//     {
//       Serial.print("Incorrect command send: ");
//       Serial.println(incomingByte);
//     }
//   }
// }






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
void updateEncoder_A()
{
  int MSB = digitalRead(EncoderPin1_A);
  int LSB = digitalRead(EncoderPin2_A);

  int encoded = (MSB << 1) | LSB;
  int sum = (lastEncoded_A << 2) | encoded;

  if (sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011)
    encoderValue_A++;
  if (sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000)
    encoderValue_A--;

  lastEncoded_A = encoded;
}

void updateEncoder_B()
{
  int MSB = digitalRead(EncoderPin1_B);
  int LSB = digitalRead(EncoderPin2_B);

  int encoded = (MSB << 1) | LSB;
  int sum = (lastEncoded_B << 2) | encoded;

  if (sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011)
    encoderValue_B++;
  if (sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000)
    encoderValue_B--;

  lastEncoded_B = encoded;
}