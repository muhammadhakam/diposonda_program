#include <Wire.h>
#include <SPI.h>
#include <Adafruit_BMP280.h>

#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include "DFRobot_MICS.h"

#include <TinyGPSPlus.h>
#include <SoftwareSerial.h>

#include <TimeLib.h> //RTC lib

#define CALIBRATION_TIME   0  // Default calibration time is three minutes
#define ADC_PIN   A0  //Pin input MiCS5524    
#define POWER_PIN 10  //Pin Enable MiCS5524

//scheduller
unsigned long time_now;
unsigned long time_last=0;

//incomming byte
String inStr, CMD, ID, Command, Act;
char c;
int Index1,Index2,Index3,Index4,Index5;

//ID & command validation
bool ID_valid;
bool TX_on=false;
bool simulation=false;
byte flt_md=70;
byte probe_stat=78;
byte selfStand_stat=78;

//counting data package
unsigned int data_count=0;

//GPS
//static const byte RXPin = 7, TXPin = 8;
static const uint32_t GPSBaud = 9600;
byte sat_val, hr, mt, sc, csc;
float lat_val, lng_val, hdop_val, alt_gps;
TinyGPSPlus gps;
//SoftwareSerial ss(RXPin, TXPin);

//IMU
float x_acc, y_acc, z_acc, x_ori, y_ori, z_ori, x_g, y_g, z_g;

//BMP280
int SLP = 1008;

//I2C Adress
Adafruit_BMP280 bmp; //BMP280 I2C addr
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire); // BNO055 I2C addr

//MiCS pin Setup
DFRobot_MICS_ADC mics(ADC_PIN, POWER_PIN);
float  gasCO, gasCH4, gasEthanol, gasH2, gasNH3, gasNO2;// gas data
  
float alt; // variabel untuk altitude
float alt_reff; // variabel untuk altitude reff
float alt_rel; //variabel untuk ketinggian relatif
float temp; //variabel untuk suhu

//RTC
int jam = 00;
int menit = 00;
int det = 00;
int tgl = 25;
int bln = 6;
int th = 23;

//tranmit
char buff[250];
void displayInfo()
{

//  Serial.print(F("Altitude GPS: "));
 if (gps.altitude.isValid())
  {
    alt_gps=gps.altitude.meters();
//    Serial.print(alt_gps);
//    Serial.print(F(" "));
  }
  else
  {
    Serial.print(F("INVALID"));
  }
//  Serial.print(F("Satelite: "));
  if (gps.satellites.isValid())
  {
    sat_val=gps.satellites.value();
//    Serial.print(sat_val);
//    Serial.print(F(" "));
  }
  else
  {
    Serial.print(F("INVALID"));
  }

//  Serial.print(F("HDOP: ")); 
  if (gps.hdop.isValid())
  {
    hdop_val=gps.hdop.hdop();
//    Serial.print(hdop_val);
//    Serial.print(F(" "));
  }
  else
  {
    Serial.print(F("INVALID"));
  }
  
//  Serial.print(F("Location: ")); 
  if (gps.location.isValid())
  {
    lat_val=gps.location.lat();
    lng_val=gps.location.lng();
//    Serial.print(lat_val, 6);
//    Serial.print(F(","));
//    Serial.print(lng_val, 6);
  }
  else
  {
    Serial.print(F("INVALID"));
  }

//  Serial.print(F("  Date/Time: "));
  if (gps.date.isValid())
  {
//    Serial.print(gps.date.month());
//    Serial.print(F("/"));
//    Serial.print(gps.date.day());
//    Serial.print(F("/"));
//    Serial.print(gps.date.year());
  }
  else
  {
    Serial.print(F("INVALID"));
  }

  Serial.print(F(" "));
  if (gps.time.isValid())
  {
    hr=gps.time.hour();
//    if (hr < 10) Serial.print(F("0"));
//    Serial.print(hr);
//    Serial.print(F(":"));
    mt=gps.time.minute();    
//    if (mt < 10) Serial.print(F("0"));
//    Serial.print(mt);
//    Serial.print(F(":"));
    sc=gps.time.second();
//    if (sc < 10) Serial.print(F("0"));
//    Serial.print(sc);
//    Serial.print(F("."));
    csc=gps.time.centisecond();
//    if (csc < 10) Serial.print(F("0"));
//    Serial.print(csc);
  }
  else
  {
    Serial.print(F("INVALID"));
  }

//  Serial.println();
}

//BNO055 print
void printserialmonitor(float x, float y, float z){
  Serial.print("\tx= ");
  Serial.print(x);
  Serial.print(" |\ty= ");
  Serial.print(y);
  Serial.print(" |\tz= ");
  Serial.println(z);
}

void printEvent(sensors_event_t* event) {
//  if (event->type == SENSOR_TYPE_ACCELEROMETER) {
//    Serial.print("Accl:");
//    x_acc = event->acceleration.x;
//    y_acc = event->acceleration.y;
//    z_acc = event->acceleration.z;

//    printserialmonitor(x_acc, y_acc, z_acc);
//  }
//    else if (event->type == SENSOR_TYPE_ORIENTATION) {
    if (event->type == SENSOR_TYPE_ORIENTATION) {
//    Serial.print("Orient:");
    x_ori = event->orientation.x;
    y_ori = event->orientation.y;
    z_ori = event->orientation.z;

//    printserialmonitor(x_ori, y_ori, z_ori);
  }
//  else if (event->type == SENSOR_TYPE_GRAVITY) {
//    Serial.print("Gravity:");
//    x_g = event->acceleration.x;
//    y_g = event->acceleration.y;
//    z_g = event->acceleration.z;

//    printserialmonitor(x_g, y_g, z_g);
//  }
}
void digitalClockDisplay(){
   // digital clock display of the time
   Serial.print(hour());
   printDigits(minute());
   printDigits(second());
   jam=hour();
   menit=minute();
   det=second();
   Serial.print(" ");
   Serial.print(day());
   Serial.print(" ");
   Serial.print(month());
   Serial.print(" ");
   Serial.print(year());
   Serial.println();
}

void printDigits(int digits) {
   // utility function for digital clock display: prints preceding colon and leading 0
   Serial.print(":");
   if(digits < 10)
      Serial.print('0');
      Serial.print(digits);
}

void setup() {
  Serial.begin(9600);
  Serial1.begin(9600);
  
//-----------------------------------

//start BMP280
  Serial.println(F("BMP280 : Starting..."));
  unsigned status;
  //status = bmp.begin(BMP280_ADDRESS_ALT, BMP280_CHIPID);
  status = bmp.begin();
  if (!status) {
    Serial.println(F("Could not find a valid BMP280 sensor, check wiring or "
                      "try a different address!"));
    Serial.print("SensorID was: 0x"); Serial.println(bmp.sensorID(),16);
    Serial.print("        ID of 0xFF probably means a bad address, a BMP 180 or BMP 085\n");
    Serial.print("   ID of 0x56-0x58 represents a BMP 280,\n");
    Serial.print("        ID of 0x60 represents a BME 280.\n");
    Serial.print("        ID of 0x61 represents a BME 680.\n");
    while (1) delay(10);
  }

  /* Default settings from datasheet. */
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_125); /* Standby time. */

  Serial.println("BMP280 : Ready");

//--------------------------------------
//Start GPS
  Serial7.begin(GPSBaud);
  Serial.println("GPS : Starting");
//--------------------------------------
//Start BNO055
  Serial.println("BNO055 : starting...");

  /* Initialise the sensor */
  if (!bno.begin())
  {
    Serial.print("BNO055 : Failed");
    while (1);
  }
  Serial.println("BNO055 : Ready");

//----------------------------------------
//Start MiCS5524

 while(!mics.begin()){
    Serial.println("Check MiCS5524");
    delay(1000);
  } 

  uint8_t mode = mics.getPowerState();
  if(mode == SLEEP_MODE){
    mics.wakeUpMode();
    Serial.println("MiCS5524 : wake up sensor success!");
  }else{
    Serial.println("MiCS5524 : The sensor is wake up mode");
  }

  while(!mics.warmUpTime(CALIBRATION_TIME)){
    Serial.println("MiCS5524 : warm up...");
    delay(1000);
  }
  Serial.println("");
//-----------------------------------------------------------
//Altimeter refference calibration  
  int count=0; 
  Serial.print("Altimeter initialization");              
  while(count <= 10000){ 
    alt_reff+=bmp.readAltitude(SLP);
    count++; 
 }
    alt_reff/=count;
    Serial.println("");
    Serial.print(F("Refference altitude = "));
    Serial.print(alt_reff); 
    Serial.println(" m");  
    delay(1000);
//RTC Set Time
   setTime(jam, menit, det, tgl, bln, th);
//Telemetry setup
  Serial1.write("B");
  delay(100);
  Serial1.write("B");
}



void loop() {
    // can now print out the new measurements
    if (simulation == false)
    {
      alt = bmp.readAltitude(SLP);
      alt_rel=alt-alt_reff;
      temp=bmp.readTemperature();
    }
//    Serial.print(F("Temperature = "));
//    Serial.print(bmp.readTemperature());
//    Serial.println(" *C");

//    Serial.print(F("Pressure = "));
//    Serial.print(bmp.readPressure());
//    Serial.println(" Pa");

//    Serial.print(F("Approx altitude = "));
//    Serial.print(alt);
//    Serial.println(" m");

//    Serial.print(F("Relative altitude = "));
//    Serial.print(alt_rel);
//    Serial.println(" m");
//    Serial.println();
//------------------------------------------------------    

//BNO055
//could add VECTOR_ACCELEROMETER, VECTOR_MAGNETOMETER,VECTOR_GRAVITY...
  sensors_event_t orientationData , angVelocityData , linearAccelData, magnetometerData, accelerometerData, gravityData;
  bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
  bno.getEvent(&accelerometerData, Adafruit_BNO055::VECTOR_ACCELEROMETER);
  bno.getEvent(&gravityData, Adafruit_BNO055::VECTOR_GRAVITY);

  printEvent(&orientationData);
//  printEvent(&accelerometerData);
//  printEvent(&gravityData);

  uint8_t system, gyro, accel, mag = 0;
  bno.getCalibration(&system, &gyro, &accel, &mag);

//--------------------------------------------------------

//MiCS 5524
  gasCO = mics.getGasData(CO);
  gasCH4 = mics.getGasData(CH4);
  gasEthanol = mics.getGasData(C2H5OH);
  gasH2 = mics.getGasData(H2);
  gasNH3 = mics.getGasData(NH3);
  gasNO2 = mics.getGasData(NO2);
//-------------------------------------------------------
//GPS
  // This sketch displays information every time a new sentence is correctly encoded.
  while (Serial7.available() > 0)
    if (gps.encode(Serial7.read()))
      displayInfo();
//----------------------------------------------------
//Recieve Command
while (Serial1.available()>0)
{
  delay(5);
  c = Serial1.read();
  inStr += c;
}


//----------------------------------------------------
//Command Parse
if (inStr.length()>0)
{
  Serial.println(inStr);
  Index1 = inStr.indexOf('<');
  Index2 = inStr.indexOf(',', Index1+1);
  Index3 = inStr.indexOf(',', Index2+1);
  Index4 = inStr.indexOf(',', Index3+1);
  Index5 = inStr.indexOf('>', Index4+1);
  CMD = inStr.substring(Index1+1, Index2);
  ID = inStr.substring(Index2+1, Index3);
  Command = inStr.substring(Index3+1, Index4);
  Act = inStr.substring(Index4+1, Index5);  
  Serial.print("ID : ");Serial.println(ID);
  Serial.print("Command : ");Serial.println(Command);
  //----------------------------------------------------
  //Command Validation
  if (CMD=="CMD" & ID=="Diposonda")
  {
    Serial.println("ID Valid");
    ID_valid = true;
  }
  else
  {
    Serial.println("ID not valid");
  }
  if (ID_valid==true)
{
  if (Command=="CX")
  {
    if (Act=="ON")
    {
      TX_on=true;
    }
    else if (Act=="OFF")
    {
      TX_on=false;
    }
  }
  else if (Command=="ST")
  {
    if (Act=="GPS")
    {
      
      setTime(hr, mt, sc, tgl, bln, th);
    }
    else
    {
      Serial.print("time set");
      byte idx1 = Act.indexOf('|');
      byte idx2 = Act.indexOf(':', idx1+1);
      byte idx3 = Act.indexOf(':', idx2+1);
      byte idx4 = Act.indexOf('|', idx3+1);
      String str_jam = Act.substring(idx1+1, idx2);
      String str_menit = Act.substring(idx2+1, idx3);
      String str_det = Act.substring(idx3+1, idx4);
      jam=str_jam.toInt();
      menit=str_menit.toInt();
      det=str_det.toInt();
      setTime(jam, menit, det, tgl, bln, th);
    }
  }
    else if (Command=="SIM")
  {
    if (Act=="ENA")
    {
      simulation=true;
      flt_md=83;
    }
    else if (Act=="DIS")
    {
      simulation=false;
      flt_md=70;
    }
  }
    else if (Command=="SIMP" & simulation==true)
  {
    int Sim_Pressure=Act.toInt();
    alt = 44330.0 * (1.0 - pow(Sim_Pressure /SLP, 0.19029495));
    alt_rel=alt-alt_reff;
  }
}
}
inStr="";
ID_valid==false;
Command="";
//----------------------------------------------------
//Transmit data
  time_now = millis();
  if(time_now - time_last >= 1000){
    time_last=time_now;
    sprintf(buff, "<Diposonda,%d : %d : %d : %d,%d,%c,%.2f,%c,%c,%.2f,%.2f,%f,%f,%d,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f>", 
    jam,menit,det,csc,data_count,flt_md,alt_rel,probe_stat,selfStand_stat,temp,alt_gps,lat_val,lng_val,sat_val,y_ori,z_ori,gasCO,gasCH4,gasEthanol,gasH2,gasNH3,gasNO2);
    digitalClockDisplay();
    Serial.println(buff);
    if (TX_on==true)
    {
    Serial1.write(buff);
    }
    data_count++;
  }
}