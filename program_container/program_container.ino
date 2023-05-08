#include <Wire.h>
#include <SPI.h>
#include <Adafruit_BMP280.h>

#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include "DFRobot_MICS.h"

#include <TinyGPSPlus.h>
#include <SoftwareSerial.h>

#define CALIBRATION_TIME   0  // Default calibration time is three minutes
#define ADC_PIN   A0  //Pin input MiCS5524    
#define POWER_PIN 10  //Pin Enable MiCS5524

//scheduller
unsigned long time_now;
unsigned long time_last=0;

//GPS
static const byte RXPin = 7, TXPin = 8;
static const uint32_t GPSBaud = 9600;
byte sat_val, hr, mt, sc, csc;
float lat_val, lng_val, hdop_val;
TinyGPSPlus gps;
SoftwareSerial ss(RXPin, TXPin);

//IMU
double x_acc, y_acc, z_acc, x_ori, y_ori, z_ori, x_g, y_g, z_g;

//I2C Adress
Adafruit_BMP280 bmp; //BMP280 I2C addr
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire); // BNO055 I2C addr

//MiCS pin Setup
DFRobot_MICS_ADC mics(ADC_PIN, POWER_PIN);
//uint16_t SAMPLERATE_DELAY_MS = 1000; //sample rate 1 Hz
float alt; // variabel untuk altitude
float alt_reff; // variabel untuk altitude reff
float alt_rel; //variabel untuk ketinggian relatif
float temp; //variabel untuk suhu

//tranmit
char buff[1000];
void displayInfo()
{

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

//  Serial.print(F(" "));
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

void setup() {
  Serial.begin(9600);
  Serial1.begin(9600);
  Serial2.begin(9600);
  
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
//--------------------------------------
//Start BNO055
 // Serial.println("BNO055 : starting...");

  /* Initialise the sensor */
 // if (!bno.begin())
  //{
  //  Serial.print("BNO055 : Failed");
  //  while (1);
 // }
 // Serial.println("BNO055 : Ready");

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
  Serial.print("Altimeter initioalization");              
  while(count <= 10000){ 
    alt_reff+=bmp.readAltitude(1008);
    count++; 
 }
    alt_reff/=count;
    Serial.println("");
    Serial.print(F("Refference altitude = "));
    Serial.print(alt_reff); 
    Serial.println(" m");  
    delay(1000);

//Telemetry setup
  Serial1.write("B");
  delay(100);
  Serial1.write("B");
  Serial2.write("B");
  delay(100);
  Serial2.write("B");
}

void printEvent(sensors_event_t* event) {
  if (event->type == SENSOR_TYPE_ACCELEROMETER) {
//    Serial.print("Accl:");
    x_acc = event->acceleration.x;
    y_acc = event->acceleration.y;
    z_acc = event->acceleration.z;

    //printserialmonitor(x_acc, y_acc, z_acc);
  }
  else if (event->type == SENSOR_TYPE_ORIENTATION) {
//    Serial.print("Orient:");
    x_ori = event->orientation.x;
    y_ori = event->orientation.y;
    z_ori = event->orientation.z;

//    printserialmonitor(x_ori, y_ori, z_ori);
  }
  else if (event->type == SENSOR_TYPE_GRAVITY) {
//    Serial.print("Gravity:");
    x_g = event->acceleration.x;
    y_g = event->acceleration.y;
    z_g = event->acceleration.z;

//    printserialmonitor(x_g, y_g, z_g);
  }
}

void loop() {
    // can now print out the new measurements
    alt = bmp.readAltitude(1008);
    alt_rel=alt-alt_reff;
    temp=bmp.readTemperature();
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
  printEvent(&accelerometerData);
  printEvent(&gravityData);

  uint8_t system, gyro, accel, mag = 0;
  bno.getCalibration(&system, &gyro, &accel, &mag);

//--------------------------------------------------------

//MiCS 5524
  float gasdata = mics.getGasData(C2H5OH);
//  Serial.print("Ethanol");
//  Serial.print(gasdata,1);
//  Serial.println(" PPM");
  gasdata = mics.getGasData(CO);
//  Serial.print("Carbon Monoxide");
//  Serial.print(gasdata,1);
//  Serial.println(" PPM");
//-------------------------------------------------------
//GPS
  // This sketch displays information every time a new sentence is correctly encoded.

//----------------------------------------------------
//Transmit data
  time_now = millis();
  if(time_now - time_last >= 1000){
    time_last=time_now;
    sprintf(buff, "<Diposonda,%.4f,%.2f,%.2f>", alt_rel,temp,x_acc);
    Serial.println(buff);
    Serial1.write(buff);
    Serial2.write(buff);
  }
}