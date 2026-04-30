#include<Wire.h>
#include<Updated_MS5837.hpp>
#include <Adafruit_BNO08x.h>
#include <EEPROM.h>
#include <SD.h>
#include <SPI.h>
#include <TinyGPS++.h>
#include <math.h>

//Circular Buffer
//--------------------------------
#define SIZE_OF_BUFFER 150    //atleast greater than rx buffer (~64 Bytes)

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
#define TriggerPin_A 35   //29 //TX pin for ranging 1 
uint8_t buf_A[16];
int range_rx_index_A = 0;
bool trigDone_A = false;
bool readDone_A = false;
long unsigned int T1_delay_A;
int cmd_i_A;

#define TriggerPin_B 29   //29 //TX pin for ranging 1 
uint8_t buf_B[16];
int range_rx_index_B = 0;
bool trigDone_B = false;
bool readDone_B = false;
long unsigned int T1_delay_B;
int cmd_i_B;
//-----------------------------


//BNO08x IMU
//--------------------------------
#define BNO08X_RESET -1
Adafruit_BNO08x bno08x_A(BNO08X_RESET);
Adafruit_BNO08x bno08x_B(BNO08X_RESET);

sh2_SensorValue_t imu_value_A, imu_value_B;
//---------------------------------



//micro m10 gps
//-------------------------------
// Choose a hardware serial port (Serial1, Serial2, etc.)
#define GPS_SERIAL Serial1
// #define DEG_TO_RAD 0.017453292519943295

TinyGPSPlus gps;
//--------------------------------



//Sensor data to Raspberry Pi ( writtten in IntervalInterrupts in Seconds instead of Hz )
// int barTP_tx = 20Hz;
// int rangingTP_tx = 50Hz;
// int imuTP_tx = 400Hz;

//Timer for sensor functions ( read from sensors )
//---------------------------
unsigned int main_timer = 0;

int barTimer = 0;
int rangingTimer_A = 0;
int rangingTimer_B = 0;
int imuTimer = 0;
int gpsTimer = 0;

unsigned int wait_for_ST_bytes = 0;
unsigned int wait_for_EN_bytes = 0;
//---------------------------


//Packets
//---------------------------
struct
{
  float temp;
  float depth;
  int pressure;
  bool initFlag;
}barPacket;

struct
{
  int long distance;
  bool initFlag;
}rangingPacket_A, rangingPacket_B;


struct
{
  float game_real;
  float game_i;
  float game_j;
  float game_k;
  float magnetic_x;
  float magnetic_y;
  float magnetic_z;
  bool status;
}imuPacket_A, imuPacket_B;

struct
{
  long encoder_copy;
  bool initFlag;
}encoderPacket_A, encoderPacket_B;

struct
{
  double easting;
  double northing;
  bool status;
}gpsPacket;


struct sensorBig        //Stores all sensors' packet
{
  float imu1[7];        //game vector[4] then magnetic field[3]
  float imu2[7];
  double gps[2];
  int bar;
  int rang1;
  int rang2;
  int enc1;
  int enc2;
}unifiedSensor;


// EEPROM
//------------------------------
int rebootAdr = 0;    //used to see if this is the first boot up or not
int adrSelEnc = 1;    //used to see which encoder to select
int adrEncA = 3;      //used to get encoderValueA if this is not the first boot
int adrEncB = 7;      //used to get encoderValueB if this is not the first boot
byte selectEncA;
byte firstBoot;
//------------------------------


// SD Card
//--------------------------------
File logFile;
const int chipSelect = BUILTIN_SDCARD;

//--------------------------------

//Persisting Data after software reboot
//--------------------------------------------------------
// struct PersistData
// {
//   uint32_t magic;
//   uint8_t imu_flag;
//   long encoder_A;
//   long encoder_B;
// };

// __attribute__((section(".noinit"))) PersistData persist;

// #define PERSIST_MAGIC 0x11000011



//---------------------------------------------------------

//-----------------------------



//Sampling time period for each Sensor
//-----------------------------
long unsigned int barTP_rx =  330;//30Hz;
long unsigned int rangingTP_rx_A = 33; //30Hz;
long unsigned int rangingTP_rx_B = 33;
long unsigned int imuTP_rx = 5;
long unsigned int gpsTP_rx = 330;
//------------------------------


//Timer Interrupts declared
//----------------------------
// IntervalTimer barTimer_tx;
// IntervalTimer rangingTimer_tx;
IntervalTimer encoderTimer_tx_A;
IntervalTimer encoderTimer_tx_B;
// IntervalTimer Pi_rx;
IntervalTimer rx_storer;     //To store incoming bytes in the circ buffer for command reading
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

  Serial.println("Inside void setup");

  Serial8.begin(115200);                              //RANGING SENSOR_A
  pinMode(TriggerPin_A, OUTPUT);                        //Used to send low pulse for starting communication
  digitalWrite(TriggerPin_A, HIGH);
  delay(500);
  trigDone_A = false;
  T1_delay_A = 0;                                       //To wait after sending low pulse to complete receiving full data

  Serial7.begin(115200);                              //RANGING SENSOR_B
  pinMode(TriggerPin_B, OUTPUT);                        //Used to send low pulse for starting communication
  digitalWrite(TriggerPin_B, HIGH);
  delay(500);
  trigDone_B = false;
  T1_delay_B = 0;                                       //To wait after sending low pulse to complete receiving full data

  GPS_SERIAL.begin(115200);                     // M10 GPS - Serial1

  //I2C Lines
  unsigned int now = millis();
  Wire2.begin();                                      //PRESSURE SENSOR
  while(!(barPacket.initFlag = barSensor.init(Wire2)) && ( millis() - now < 1000 ))   //wait for 1 seconds
  {
    delay(10);
    Serial.println("Bar Failing..");
  }
  barSensor.setModel(MS5837::MS5837_30BA);
  barSensor.setFluidDensity(fluidDensity);
  Serial.println("Bar success");

  // if ( persist.magic != PERSIST_MAGIC )     //for first boot
  // {
  //   // Serial.println("")
  //   persist.magic = PERSIST_MAGIC;
  //   persist.imu_flag = 1;
  //   persist.encoder_A = 0;
  //   persist.encoder_B = 0;
  //   Serial.println("First boot");
  // }
  // else
  // {
  //   Serial.println("Function rebooted");
  // }

  // Serial.println("Persist part done");

  EEPROM.get(rebootAdr, firstBoot);
  if ( firstBoot )      //if firstBoot is set
  {
    encoderValue_A = 0;
    encoderValue_B = 0;
    selectEncA = 1;
  }
  else                  //if firstBoot is not set
  {
    long encA_copy, encB_copy;
    EEPROM.get(adrSelEnc, selectEncA);
    EEPROM.get(adrEncA, encA_copy);
    EEPROM.get(adrEncB, encB_copy);
    EEPROM.put(rebootAdr, 0);
    encoderValue_A = encA_copy;
    encoderValue_B = encB_copy;
    Serial.println(selectEncA);
  }

  if ( !SD.begin(chipSelect) )
  {
    Serial.println("SD Card initialization failed");
  }

  logFile = SD.open("logFile.txt", FILE_WRITE);
  logFile.println("New cycle");

  // encoderValue_A = persist.encoder_A;   //taken from last boot
  // encoderValue_B = persist.encoder_B;

  // Interrupt Sensor(Encoder as can't miss a single pulse)
  pinMode(EncoderPin1_A, INPUT);                        //ENCODER A
  pinMode(EncoderPin2_A, INPUT);
  attachInterrupt(digitalPinToInterrupt(EncoderPin1_A), updateEncoder_A, CHANGE);
  attachInterrupt(digitalPinToInterrupt(EncoderPin2_A), updateEncoder_A, CHANGE);

  pinMode(EncoderPin1_B, INPUT);                        //ENCODER B
  pinMode(EncoderPin2_B, INPUT);
  attachInterrupt(digitalPinToInterrupt(EncoderPin1_B), updateEncoder_B, CHANGE);
  attachInterrupt(digitalPinToInterrupt(EncoderPin2_B), updateEncoder_B, CHANGE);


  pinMode(PWM_PIN_A, OUTPUT);                         //MOTOR DRIVER
  pinMode(DIR_PIN_A, OUTPUT);
  pinMode(PWM_PIN_B, OUTPUT);
  pinMode(DIR_PIN_B, OUTPUT);

  Serial.println("Encoder part done");

  Wire1.begin();                                      //IMU_A
  Wire.begin();                                      //IMU_B
  // Try to initialize!
  now = millis();
  if ( selectEncA )
  {
    if (!bno08x_A.begin_I2C(0x4A, &Wire1)) {
      Serial.println("Failed to find BNO08x_A chip");     //Need to reboot Teensy if it fails
      // while (1) { delay(10); }
      imuPacket_A.status = false;     //not initialized 
      delay(10);          
    }
    else
    {
      Serial.println("BNO08x_A Found!");  
      imuPacket_A.status = true;      //initialized
      for (int n = 0; n < bno08x_A.prodIds.numEntries; n++) {
        Serial.print("Part ");
        Serial.print(bno08x_A.prodIds.entry[n].swPartNumber);
        Serial.print(": Version :");
        Serial.print(bno08x_A.prodIds.entry[n].swVersionMajor);
        Serial.print(".");
        Serial.print(bno08x_A.prodIds.entry[n].swVersionMinor);
        Serial.print(".");
        Serial.print(bno08x_A.prodIds.entry[n].swVersionPatch);
        Serial.print(" Build ");
        Serial.println(bno08x_A.prodIds.entry[n].swBuildNumber);
      }

      setReports_A();                                     //Important: Used to set what we will get from IMU

      Serial.println("Reading events IMU_A");
    }
  }

  else
  {
    if (!bno08x_B.begin_I2C(0x4A, &Wire)) {
      Serial.println("Failed to find BNO08x_B chip");     //Need to reboot Teensy if it fails
      // while (1) { delay(10); }
      imuPacket_B.status = false;                   //not initialized
      delay(10000);          
    }
    else
    {
      Serial.println("BNO08x_B Found!");
      imuPacket_B.status = true;                    //initialized
      for (int n = 0; n < bno08x_B.prodIds.numEntries; n++) {
        Serial.print("Part ");
        Serial.print(bno08x_B.prodIds.entry[n].swPartNumber);
        Serial.print(": Version :");
        Serial.print(bno08x_B.prodIds.entry[n].swVersionMajor);
        Serial.print(".");
        Serial.print(bno08x_B.prodIds.entry[n].swVersionMinor);
        Serial.print(".");
        Serial.print(bno08x_B.prodIds.entry[n].swVersionPatch);
        Serial.print(" Build ");
        Serial.println(bno08x_B.prodIds.entry[n].swBuildNumber);
        delay(1000);
      }

      setReports_B();                                     //Important: Used to set what we will get from IMU


      Serial.println("Reading events IMU_B");
    }
  }
  Serial.println("IMU part done");
  delay(100);

  // now = millis();
  // if (!bno08x_A.begin_I2C(0x4A, &Wire1)) {
  //   Serial.println("Failed to find BNO08x_A chip");     //Need to reboot Teensy if it fails
  //   // while (1) { delay(10); }
  //   imuPacket_A.initFlag = false;     //not initialized 
  //   delay(10);          
  // }
  // else
  // {
  //   Serial.println("BNO08x_A Found!");  
  //   imuPacket_A.initFlag = true;      //initialized
  //   for (int n = 0; n < bno08x_A.prodIds.numEntries; n++) {
  //     Serial.print("Part ");
  //     Serial.print(bno08x_A.prodIds.entry[n].swPartNumber);
  //     Serial.print(": Version :");
  //     Serial.print(bno08x_A.prodIds.entry[n].swVersionMajor);
  //     Serial.print(".");
  //     Serial.print(bno08x_A.prodIds.entry[n].swVersionMinor);
  //     Serial.print(".");
  //     Serial.print(bno08x_A.prodIds.entry[n].swVersionPatch);
  //     Serial.print(" Build ");
  //     Serial.println(bno08x_A.prodIds.entry[n].swBuildNumber);
  //   }

  //   setReports_A();                                     //Important: Used to set what we will get from IMU

  //   Serial.println("Reading events IMU_A");
  // }

  // delay(100);

  //Interrupts Priority   !!change priority according to the rule => Highest for encoder receiving; then after that; higher priority for less frequently occuring events i.e. those with bigger time period
  // encoderTimer_tx_A.priority(1);
  // // Pi_rx.priority(2);
  // barTimer_tx.priority(2);
  // rangingTimer_tx.priority(3);
  // barTimer_tx.priority(4);

  //Interrupts Begin
  // barTimer_tx.begin(barSend, 50000); //20Hz         //INTERRUPTS setting timers

  // rangingTimer_tx.begin(rangingSend, 66000);  //15Hz

  encoderTimer_tx_A.begin(encoderSend_A, 70000);   //70Khz
  encoderTimer_tx_B.begin(encoderSend_B, 70000);   //70Khz
  
  // Pi_rx.begin(piReceive, 50000);              //20Hz

  rx_storer.begin(fill_cb, 2500);   //40Hz   //to store the bytes in rx buffer (~64 Bytes) into larger circular buffer cb

  Serial.println("Leaving void setup");
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
void setReports_A(void) {
  Serial.println("Setting desired reports");
  if (! bno08x_A.enableReport(SH2_ROTATION_VECTOR)) {
    Serial.println("Could not enable rotation vector");
  }
  else
    Serial.println("rotation vector enabled");

  if (! bno08x_A.enableReport(SH2_MAGNETIC_FIELD_CALIBRATED, 5000)) {
    Serial.println("Could not enable magnetic field");
  }
  else
    Serial.println("magnetic field enabled");
}


void setReports_B(void) {
  Serial.println("Setting desired reports");
  if (! bno08x_B.enableReport(SH2_ROTATION_VECTOR)) {
    Serial.println("Could not enable rotation vector");
  }
  else
    Serial.println("rotation vector enabled");


  if (! bno08x_B.enableReport(SH2_MAGNETIC_FIELD_CALIBRATED, 5000)) {
    Serial.println("Could not enable magnetic field");
  }
  else
    Serial.println("magnetic field enabled");
}

///////////////////////////////////////////////////////////////////////////////////////////

void loop()
{
  if ( millis() - main_timer > 25)    //40Hz
  {
    Serial.println("HELLO");
    // Serial.println("IN VOID LOOP");
    timestamp = micros();
    // Serial.println(timestamp);
    // Serial.println(millis());
    // Serial.println(micros());
    // Serial.println("HELLO");
    if ( millis() - barTimer >= barTP_rx) //&& (Wire2.available()))        //contains one pressure sensor
    {
      barTimer = millis();
      if ( barPacket.initFlag )
      {
        barSensor.read();
        // Serial.println("Bar sensor OK");
        barPacket.depth = (int)barSensor.depth();
        // Serial.println(barSensor.depth());
        barPacket.temp = (int)barSensor.temperature();
        barPacket.pressure = (int)barSensor.pressure();
      // barSensorRead();
      }
      else
      {
        barPacket.depth = -999;
        barPacket.temp = -999;
        barPacket.pressure = -999;
      }
    }

    if (millis() - gpsTimer >= gpsTP_rx)
    {
      gpsTimer = millis();
      if (GPS_SERIAL.available() > 0)
      {
        if (gps.encode(GPS_SERIAL.read()))
        {
          if (gps.location.isUpdated())
          {
            double E, N;
            latLonToUTM_Zone43(
                gps.location.lat(),
                gps.location.lng(),
                &E, &N
            );
            gpsPacket.easting = E;
            gpsPacket.northing = N;
            gpsPacket.status = 1;
          }
          else
            gpsPacket.status = 0;
        }
        else
          gpsPacket.status = 0;
      }
      else
        gpsPacket.status = 0;
    }



    if ( (millis() - rangingTimer_A >= rangingTP_rx_A) && (!trigDone_A))
    {
      rangingTimer_A = millis();
      digitalWrite(TriggerPin_A, LOW);
      delayMicroseconds(500);
      digitalWrite(TriggerPin_A, HIGH);
      T1_delay_A = millis();
      trigDone_A = true;
      range_rx_index_A = 0;
      // triggerSensor();
      // debugRawBytes();
    }

    
    if ( trigDone_A && ((millis() - T1_delay_A) >= 10) )
    {
      if ( range_rx_index_A < 4 )      //replaced while with if
      {
        if ( Serial8.available() )
        {
          buf_A[range_rx_index_A++] = Serial8.read();
        }
      }
      else
        readDone_A = true;
    }

    

    if ( readDone_A )       //oneshot
    {
      if ( buf_A[0] == 0xFF )
      {
        int calcsum_A = (buf_A[0] + buf_A[1] + buf_A[2]) & 0xFF;
        if (calcsum_A == buf_A[3])
        {
          rangingPacket_A.distance = buf_A[1] * 256 + buf_A[2];
          trigDone_A = false;
          readDone_A = false;
        }
        else
        {
          //CHECKSUM ERROR
          trigDone_A = false;
          readDone_A = false;
        }
      }
      //HEADER NOT MATCHED
      trigDone_A = false;
      readDone_A = false;
    }




    if ( (millis() - rangingTimer_B >= rangingTP_rx_B) && (!trigDone_B))
    {
      rangingTimer_B = millis();
      digitalWrite(TriggerPin_B, LOW);
      delayMicroseconds(500);
      digitalWrite(TriggerPin_B, HIGH);
      T1_delay_B = millis();
      trigDone_B = true;
      range_rx_index_B = 0;
      // triggerSensor();
      // debugRawBytes();
    }

    
    if ( trigDone_B && ((millis() - T1_delay_B) >= 10) )
    {
      if ( range_rx_index_B < 4 )      //replaced while with if
      {
        if ( Serial7.available() )
        {
          buf_B[range_rx_index_B++] = Serial7.read();
        }
      }
      else
        readDone_B = true;
    }

    

    if ( readDone_B )       //oneshot
    {
      if ( buf_B[0] == 0xFF )
      {
        int calcsum_B = (buf_B[0] + buf_B[1] + buf_B[2]) & 0xFF;
        if (calcsum_B == buf_B[3])
        {
          rangingPacket_B.distance = buf_B[1] * 256 + buf_B[2];
          trigDone_B = false;
          readDone_B = false;
        }
        else
        {
          //CHECKSUM ERROR
          trigDone_B = false;
          readDone_B = false;
        }
      }
      //HEADER NOT MATCHED
      trigDone_B = false;
      readDone_B = false;
    }



    if ( millis() - imuTimer >= imuTP_rx )      //contains both IMU_A and IMU_B
    {
      imuTimer = millis();
      if ( imuPacket_A.status )
      // if ( bno08x_A.getSensorEvent(&imu_value_A) )
      {
        bno08x_A.getSensorEvent(&imu_value_A);
        // Serial.println("updating imu value");
        // Serial.println(imu_value_A.sensorId);
        Serial.println("IMU Packet initialized");
        Serial.println(imu_value_A.sensorId);
        switch (imu_value_A.sensorId)
        {
          case SH2_ROTATION_VECTOR:
            imuPacket_A.game_real = imu_value_A.un.rotationVector.real;
            imuPacket_A.game_i = imu_value_A.un.rotationVector.i;
            imuPacket_A.game_j = imu_value_A.un.rotationVector.j;
            imuPacket_A.game_k = imu_value_A.un.rotationVector.k;
            Serial.println("Correct Case");
            // Serial.println("Setting IMU packet");
            // Serial.println(imu_value_A.un.gameRotationVector.real);
            // Serial.println(imu_value_A.un.gameRotationVector.i);
            // Serial.println(imu_value_A.un.gameRotationVector.j);
            // Serial.println(imu_value_A.un.gameRotationVector.k);
            break;

          case SH2_MAGNETIC_FIELD_CALIBRATED:
            imuPacket_A.magnetic_x = imu_value_A.un.magneticField.x;
            imuPacket_A.magnetic_y = imu_value_A.un.magneticField.x;
            imuPacket_A.magnetic_z = imu_value_A.un.magneticField.x;
            break;
        }
      }
      else      //if not initialized
      {
        imuPacket_A.game_real = -999.0;
        imuPacket_A.game_i = -999.0;
        imuPacket_A.game_j = -999.0;
        imuPacket_A.game_k = -999.0;
        imuPacket_A.magnetic_x = -999.0;
        imuPacket_A.magnetic_y = -999.0;
        imuPacket_A.magnetic_z = -999.0;
      }

      if ( imuPacket_B.status )
      {
        bno08x_B.getSensorEvent(&imu_value_B);
        // Serial.println("updating imu value");
        // Serial.println(imu_value_A.sensorId);
        switch (imu_value_B.sensorId)
        {
          case SH2_ROTATION_VECTOR:
            imuPacket_B.game_real = imu_value_B.un.rotationVector.real;
            imuPacket_B.game_i = imu_value_B.un.rotationVector.i;
            imuPacket_B.game_j = imu_value_B.un.rotationVector.j;
            imuPacket_B.game_k = imu_value_B.un.rotationVector.k;
            // Serial.println("Setting IMU packet");
            // Serial.println(imu_value_A.un.gameRotationVector.real);
            // Serial.println(imu_value_A.un.gameRotationVector.i);
            // Serial.println(imu_value_A.un.gameRotationVector.j);
            // Serial.println(imu_value_A.un.gameRotationVector.k);
            break;

          case SH2_MAGNETIC_FIELD_CALIBRATED:
            imuPacket_B.magnetic_x = imu_value_B.un.magneticField.x;
            imuPacket_B.magnetic_y = imu_value_B.un.magneticField.x;
            imuPacket_B.magnetic_z = imu_value_B.un.magneticField.x;
            break;
        }
      }
      else      //if not initialized
      {
        imuPacket_B.game_real = -999.0;
        imuPacket_B.game_i = -999.0;
        imuPacket_B.game_j = -999.0;
        imuPacket_B.game_k = -999.0;
        imuPacket_B.magnetic_x = -999.0;
        imuPacket_B.magnetic_y = -999.0;
        imuPacket_B.magnetic_z = -999.0;
      }
    }



    build_sensor_packet();    //send all sensors' data to Raspberry Pi

    rx_parser();              //to go through stored incoming bytes in circular buffer cb

    if ( micros() - timestamp >= time_period )
      time_period = micros() - timestamp;
    // Serial.println(time_period);
    // Serial.println(millis() - timestamp);
    // Serial.println(millis());
    // Serial.println(micros());
    // Serial.println("IN VOID LOOP");

    main_timer = millis();
  }
}
//------------------------------------------------------------------------------------------------------------




void fill_cb(void)
{
  if (Serial.available())
  {
    cb_write(Serial.read());
  }
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
    // Serial.println("RX Buffer empty");
    // delay(1000);
    // return '\0';  //for false
    return 'L';
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
  Serial.println(state);
  if (!strcmp(state, "IDLE"))
  {
    Serial.println("IDLE");
    // char 
    if (cb_read() == '<')
    {
      // state = "CHECK_ST";
      // Serial.println("IDLE if");
      Serial.println();
      strcpy(state, "CHECK_ST");
      wait_for_ST_bytes = millis();
    }
    else
    {
      Serial.println("Error during parsing");
      // state = "IDLE";
      strcpy(state, "IDLE");
    }
  }
  if (!strcmp(state, "CHECK_ST") && (millis() - wait_for_ST_bytes > 10))
  {
    Serial.println("CHECK_ST");
    char c1,c2,c3;
    c1 = c2 = c3 = 0;
    if (((c1 = cb_read()) == 'S') && ((c2 = cb_read()) == 'T') && ((c3 = cb_read()) == '>'))
    {
      strcpy(state, "START_PARS");
    }
    else
    {
      Serial.println(c1);
      Serial.println(c2);
      Serial.println(c3);
      Serial.println("Parse error ST");
      // delay(1000);
      strcpy(state, "IDLE");
    }
  }
  if (!strcmp(state, "START_PARS"))
  {
    Serial.println("START_PARS");
    if ((c = cb_read()) == '<')
    {
      strcpy(state, "CHECK_EN");
      // buf_write('\0')
      cmd_buf[cmd_buf_ind++] = '\0';
      wait_for_EN_bytes = millis();
    }
    else
    {
      // buf_write( c );
      cmd_buf[cmd_buf_ind++] = c;
    }
  }
  if (!strcmp(state, "CHECK_EN") && (millis() - wait_for_EN_bytes > 10))
  {
    Serial.println("CHECK_EN");
    char c4,c5,c6;
    c4 = c5 = c6 = 0;
    if (((c4 = cb_read()) == 'E') && ((c5 = cb_read()) == 'N') && ((c6 = cb_read()) == '>'))
    {
      // state = "IDLE";
      strcpy(state, "IDLE");
      // cmd_complete = true;      //tells void loop() that a full cmd is available to execute
      cmd_exec(cmd_buf);
      cmd_buf_ind = 0;
      Serial.println("OUTSIDE CMD_EXEC");
      delay(1000);
    }
    else
    {
      Serial.println("Parse error EN");
      delay(1000);
      // state = "IDLE";
      strcpy(state, "IDLE");
    }
  }
  // delay(10000);
}


void cmd_exec (char buf[])
{
  delay(1000);
  Serial.println("INSIDE CMD_EXEC");
  char cmd[5];
  int i, j;
  int val[12] = {0};
  for ( i = 0 , j = 0 ; j < 4; )
  {
    cmd[j++] = buf[i++];
  }
  //last i++ skips a colon(:)
  cmd[j] = '\0';
  Serial.println(cmd);
  Serial.println(buf);
  delay(1000);

  // if ( cmd == "CTRL" )
  if ( !strcmp(cmd, "CTRL") )
  {
    
    // int val[12] = {0};
    for ( unsigned int j = 0; j < sizeof(val)/sizeof(val[0]); j++ )
    {
      while ( (buf[++i] != ',') && (buf[i] != '\0') )
      {
        val[j] *= 10;
        val[j] += buf[i] - '0';
        Serial.println( buf[i]);
        delay(1000);
      }
      if ( val[j] > 255 || val[j] < 0)
      {
        val[j] = 0;
      }
      Serial.println(val[j]);
    }

    analogWrite(PWM_PIN_A, val[0]);
    digitalWrite(DIR_PIN_A, val[1]);
    analogWrite(PWM_PIN_B, val[2]);
    digitalWrite(DIR_PIN_B, val[3]);
    Serial.println(val[0]);
    Serial.println(val[1]);
    return;
  }
  else if ( !strcmp(cmd, "RSET"))     //used to stop all motors and thrusters and all interrupt functions
  {
    analogWrite(PWM_PIN_A, 0);
    analogWrite(PWM_PIN_B, 0);
    noInterrupts();
    delay(1000);
    interrupts();
    return;
  }
  // else if ( !strcmp(cmd, "RBOT"))
  // {
  //   SCB_AIRCR = 0x05FA0004;
  //   return;
  // }
  else if ( !strcmp(cmd, "STAT"))
  {
    Serial.println(imuPacket_A.status);
    Serial.println(imuPacket_B.status);
    Serial.println(barPacket.initFlag);
    delay(1000);
    return;
  }
  else if ( !strcmp(cmd, "PING"))
  {
    Serial.println("HI");
    delay(1000);
    return;
  }
  else if ( !strcmp(cmd, "SELA"))
  {
    if ( !imuPacket_A.status )
    {
      // selectIMU_A();
      // imuPacket_B.initFlag = false;
      logFile.println("EOF");
      logFile.close();
      rebootSaveState(1);
    }
    else
      Serial.println("Already Chosen");
    delay(1000);
  }
  else if ( !strcmp(cmd, "SELB"))
  {
    if ( !imuPacket_B.status )
    {
      // selectIMU_B();
      // imuPacket_A.initFlag = false;
      logFile.println("EOF");
      logFile.close();
      rebootSaveState(0);
    }
    else
      Serial.println("Already chosen");
    delay(1000);
  }
  else if ( !strcmp(cmd, "LOGD"))
  {
    logFile.println(unifiedSensor.bar);
    logFile.println(unifiedSensor.enc1);
    logFile.println(unifiedSensor.enc2);
    logFile.println(unifiedSensor.rang1);
    logFile.println(unifiedSensor.rang2);

    logFile.println(unifiedSensor.imu1[0]);
    logFile.println(unifiedSensor.imu1[1]);
    logFile.println(unifiedSensor.imu1[2]);
    logFile.println(unifiedSensor.imu1[3]);
    logFile.println(unifiedSensor.imu2[0]);
    logFile.println(unifiedSensor.imu2[1]);
    logFile.println(unifiedSensor.imu2[2]);
    logFile.println(unifiedSensor.imu2[3]);

    logFile.println(unifiedSensor.imu1[4]);
    logFile.println(unifiedSensor.imu1[5]);
    logFile.println(unifiedSensor.imu1[6]);
    logFile.println(unifiedSensor.imu2[4]);
    logFile.println(unifiedSensor.imu2[5]);
    logFile.println(unifiedSensor.imu2[6]);

    logFile.println();
  }
  else if ( !strcmp(cmd, "STOP"))
  {
    logFile.println("EOF");
    logFile.close();
    noInterrupts();
    EEPROM.put(firstBoot, 1);
    long reset_enc = 0;
    EEPROM.put(adrEncA, reset_enc);
    EEPROM.put(adrEncB, reset_enc);
    EEPROM.put(adrSelEnc, 1);
    Serial.println("Sleeping for 10 seconds");
    delay(1000);     //10sec delay without interrupts
    SCB_AIRCR = 0x05FA0004;
    return;
  }
  else
  {
    Serial.println("WrngCmd");
    return;
  }
}


//Sensor Packet Builder and Sender
//------------------------------------------------------
void build_sensor_packet(void)
{
  unifiedSensor.bar = barPacket.pressure;
  unifiedSensor.enc1 = encoderPacket_A.encoder_copy;
  unifiedSensor.enc2 = encoderPacket_B.encoder_copy;
  unifiedSensor.rang1 = rangingPacket_A.distance;
  unifiedSensor.rang2 = rangingPacket_B.distance;
  
  unifiedSensor.imu1[0] = imuPacket_A.game_real;
  unifiedSensor.imu1[1] = imuPacket_A.game_i;
  unifiedSensor.imu1[2] = imuPacket_A.game_j;
  unifiedSensor.imu1[3] = imuPacket_A.game_k;
  
  unifiedSensor.imu2[0] = imuPacket_B.game_real;
  unifiedSensor.imu2[1] = imuPacket_B.game_i;
  unifiedSensor.imu2[2] = imuPacket_B.game_j;
  unifiedSensor.imu2[3] = imuPacket_B.game_k;

  unifiedSensor.imu1[4] = imuPacket_A.magnetic_x;
  unifiedSensor.imu1[5] = imuPacket_A.magnetic_y;
  unifiedSensor.imu1[6] = imuPacket_A.magnetic_z;
  
  unifiedSensor.imu2[4] = imuPacket_B.magnetic_x;
  unifiedSensor.imu2[5] = imuPacket_B.magnetic_y;
  unifiedSensor.imu2[6] = imuPacket_B.magnetic_z;

  unifiedSensor.gps[0] = gpsPacket.easting;
  unifiedSensor.gps[1] = gpsPacket.northing;
  
  // uint8_t header = 0xAA;
  // Serial.write(&header, 1);
  // Serial.write((uint8_t*)&unifiedSensor, sizeof(unifiedSensor));

  // Serial.println("Sensors:");
  Serial.print("<ST>");
  Serial.print("SDAT:");
  Serial.println(unifiedSensor.bar);
  Serial.println(unifiedSensor.enc1);
  Serial.println(unifiedSensor.enc2);
  Serial.println(unifiedSensor.rang1);
  Serial.println(unifiedSensor.rang2);

  Serial.println(unifiedSensor.imu1[0]);
  Serial.println(unifiedSensor.imu1[1]);
  Serial.println(unifiedSensor.imu1[2]);
  Serial.println(unifiedSensor.imu1[3]);

  Serial.println(unifiedSensor.imu2[0]);
  Serial.println(unifiedSensor.imu2[1]);
  Serial.println(unifiedSensor.imu2[2]);
  Serial.println(unifiedSensor.imu2[3]);

  Serial.println(unifiedSensor.imu1[4]);
  Serial.println(unifiedSensor.imu1[5]);
  Serial.println(unifiedSensor.imu1[6]);

  Serial.println(unifiedSensor.imu2[4]);
  Serial.println(unifiedSensor.imu2[5]);
  Serial.println(unifiedSensor.imu2[6]);

  Serial.println(unifiedSensor.gps[0]);
  Serial.println(unifiedSensor.gps[1]);
  Serial.println("<EN>");

  // logFile.println(unifiedSensor.bar);
  // logFile.println(unifiedSensor.enc1);
  // logFile.println(unifiedSensor.enc2);
  // logFile.println(unifiedSensor.rang1);
  // logFile.println(unifiedSensor.rang2);
  // logFile.println(unifiedSensor.imu1[0]);
  // logFile.println(unifiedSensor.imu1[1]);
  // logFile.println(unifiedSensor.imu1[2]);
  // logFile.println(unifiedSensor.imu1[3]);
  // logFile.println(unifiedSensor.imu2[0]);
  // logFile.println(unifiedSensor.imu2[1]);
  // logFile.println(unifiedSensor.imu2[2]);
  // logFile.println(unifiedSensor.imu2[3]);
  // logFile.println();
}
//------------------------------------------------------


void rebootSaveState ( uint8_t imu )
{
  noInterrupts();
  // persist.imu_flag = imu;
  // persist.encoder_A = encoderValue_A;
  // persist.encoder_B = encoderValue_B;
  long copyA = encoderValue_A;
  long copyB = encoderValue_B;
  Serial.println(encoderValue_A);
  Serial.println(encoderValue_B);
  EEPROM.put(adrSelEnc, imu);
  EEPROM.put(adrEncA, copyA);
  EEPROM.put(adrEncB, copyB);
  EEPROM.update(rebootAdr, 0);      //firstBoot un set to show not first boot for next booting
  SCB_AIRCR = 0x05FA0004;
}


void latLonToUTM_Zone43(double lat, double lon,
                        double *easting, double *northing)
{
    // WGS84 constants
    const double a  = 6378137.0;
    const double f  = 1 / 298.257223563;
    const double k0 = 0.9996;

    const double e2 = f * (2 - f);
    const double ep2 = e2 / (1 - e2);

    // Convert to radians
    double phi = lat * DEG_TO_RAD;
    double lambda = lon * DEG_TO_RAD;

    // Central meridian (Zone 43)
    double lambda0 = 75.0 * DEG_TO_RAD;

    double N = a / sqrt(1 - e2 * sin(phi) * sin(phi));
    double T = tan(phi) * tan(phi);
    double C = ep2 * cos(phi) * cos(phi);
    double A = cos(phi) * (lambda - lambda0);

    // Meridian arc
    double M = a * (
        (1 - e2/4 - 3*e2*e2/64 - 5*e2*e2*e2/256) * phi
        - (3*e2/8 + 3*e2*e2/32 + 45*e2*e2*e2/1024) * sin(2*phi)
        + (15*e2*e2/256 + 45*e2*e2*e2/1024) * sin(4*phi)
        - (35*e2*e2*e2/3072) * sin(6*phi)
    );

    // Easting
    *easting = k0 * N * (
        A + (1 - T + C) * pow(A,3)/6
        + (5 - 18*T + T*T + 72*C - 58*ep2) * pow(A,5)/120
    ) + 500000.0;

    // Northing (Northern hemisphere → no offset)
    *northing = k0 * (
        M + N * tan(phi) * (
            A*A/2
            + (5 - T + 9*C + 4*C*C) * pow(A,4)/24
            + (61 - 58*T + T*T + 600*C - 330*ep2) * pow(A,6)/720
        )
    );
}

// void selectIMU_A ( void )
// {
//   Wire1.begin();
//   // Adafruit_BNO08x bno08x_A(BNO08X_RESET);
//   if (!bno08x_A.begin_I2C(0x4A, &Wire1)) {
//     Serial.println("Failed to find BNO08x_A chip");     //Need to reboot Teensy if it fails
//     // while (1) { delay(10); }
//     imuPacket_A.initFlag = false;     //not initialized 
//     delay(10);          
//   }
//   else
//   {
//     Serial.println("BNO08x_A Found!");  
//     imuPacket_A.initFlag = true;      //initialized
//     for (int n = 0; n < bno08x_A.prodIds.numEntries; n++) {
//       Serial.print("Part ");
//       Serial.print(bno08x_A.prodIds.entry[n].swPartNumber);
//       Serial.print(": Version :");
//       Serial.print(bno08x_A.prodIds.entry[n].swVersionMajor);
//       Serial.print(".");
//       Serial.print(bno08x_A.prodIds.entry[n].swVersionMinor);
//       Serial.print(".");
//       Serial.print(bno08x_A.prodIds.entry[n].swVersionPatch);
//       Serial.print(" Build ");
//       Serial.println(bno08x_A.prodIds.entry[n].swBuildNumber);
//     }

//     // setReports_A();                                     //Important: Used to set what we will get from IMU

//     Serial.println("Setting desired reports");
//     if (! bno08x_A.enableReport(SH2_GAME_ROTATION_VECTOR)) {
//       Serial.println("Could not enable game vector");
//     }
//     else
//       Serial.println("game vector enabled");
//   }
// }

// void selectIMU_B ( void )
// {
//   Wire.begin();
//   // Adafruit_BNO08x bno08x_B(BNO08X_RESET);
//   if (!bno08x_B.begin_I2C(0x4A, &Wire)) {
//     Serial.println("Failed to find BNO08x_B chip");     //Need to reboot Teensy if it fails
//     // while (1) { delay(10); }
//     imuPacket_B.initFlag = false;     //not initialized 
//     delay(10);          
//   }
//   else
//   {
//     Serial.println("BNO08x_B Found!");  
//     imuPacket_B.initFlag = true;      //initialized
//     for (int n = 0; n < bno08x_B.prodIds.numEntries; n++) {
//       Serial.print("Part ");
//       Serial.print(bno08x_B.prodIds.entry[n].swPartNumber);
//       Serial.print(": Version :");
//       Serial.print(bno08x_B.prodIds.entry[n].swVersionMajor);
//       Serial.print(".");
//       Serial.print(bno08x_B.prodIds.entry[n].swVersionMinor);
//       Serial.print(".");
//       Serial.print(bno08x_B.prodIds.entry[n].swVersionPatch);
//       Serial.print(" Build ");
//       Serial.println(bno08x_B.prodIds.entry[n].swBuildNumber);
//     }

//     // setReports_B();                                     //Important: Used to set what we will get from IMU

//     Serial.println("Setting desired reports");
//     if (! bno08x_B.enableReport(SH2_GAME_ROTATION_VECTOR)) {
//       Serial.println("Could not enable game vector");
//     }
//     else
//       Serial.println("game vector enabled");
//   }
// }

// void barSend ()
// {
//   // uint8_t buf[] = {barPacket.depth, barPacket.pressure, barPacket.temp};    //THIS WILL CAUSE PROBLEMS LATER, PLEASE FIX IT LATER ACCORDING TO RX DATA FROM RASPBERRY PI
//   // Serial.write(buf, sizeof(buf));
//   // Serial.println("bar packet");
//   // Serial.println(barPacket.temp);
//   // Serial.println();
// }

// void rangingSend ()
// {
//   // Serial.write(rangingPacket.distance);
//   // Serial.println("ranging packet");
//   // Serial.println(rangingPacket.distance);
//   // Serial.println();
// }

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
