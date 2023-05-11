#include <Wire.h>
#include <SPI.h>
#include <Adafruit_BMP280.h>

#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
//#include "DFRobot_MICS.h"

#include <TinyGPSPlus.h>
#include <SoftwareSerial.h>

#include <TimeLib.h> //RTC lib

#include <Servo.h>

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
bool ID_valid=false;
bool TX_on=false;
bool simulation=false;
bool CAL_proc=false;
bool IMU_cal = false;
bool LVL_cal = false;
bool selfStanding = false;
bool alt_req = true;
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
float x_acc, y_acc, z_acc, x_ori, y_ori, z_ori, x_g, y_g, z_g, accRef, accSigma, accRel;
uint8_t sys, gyro, accel, mag = 0;
float acc[3]={0,0,0};

//BMP280
int SLP = 1008;

//I2C Adress
Adafruit_BMP280 bmp; //BMP280 I2C addr
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire); // BNO055 I2C addr

//MiCS pin Setup
//DFRobot_MICS_ADC mics(ADC_PIN, POWER_PIN);
//float  gasCO, gasCH4, gasEthanol, gasH2, gasNH3, gasNO2;// gas data
  
float alt; // variabel untuk altitude
float alt_reff; // variabel untuk altitude reff
float alt_rel; //variabel untuk ketinggian relatif
float alt_last = 0;
float temp; //variabel untuk suhu
float vSpeed; // variabel untuk kecepatan vertikal

//RTC
int jam = 00;
int menit = 00;
int det = 00;
int tgl = 25;
int bln = 6;
int th = 23;

//tranmit
char buff[500];

//Aktuator control
Servo servo1; 
Servo servo2;
Servo servo3;
Servo servo4;
byte i=0;

void displayInfo()
{
 if (gps.altitude.isValid())
  {
    alt_gps=gps.altitude.meters();
  }
  else
  {
    Serial.print(F("INVALID"));
  }
  if (gps.satellites.isValid())
  {
    sat_val=gps.satellites.value();
  }
  else
  {
    Serial.print(F("INVALID"));
  }

//  Serial.print(F("HDOP: ")); 
  if (gps.hdop.isValid())
  {
    hdop_val=gps.hdop.hdop();
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
  }
  else
  {
    Serial.print(F("INVALID"));
  }

  Serial.print(F(" "));
  if (gps.time.isValid())
  {
    hr=gps.time.hour();
    mt=gps.time.minute();    
    sc=gps.time.second();
    csc=gps.time.centisecond();
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
  if (event->type == SENSOR_TYPE_ACCELEROMETER) {
//    Serial.print("Accl:");
    x_acc = event->acceleration.x;
    y_acc = event->acceleration.y;
    z_acc = event->acceleration.z;

//    printserialmonitor(x_acc, y_acc, z_acc);
  }
//    else if (event->type == SENSOR_TYPE_ORIENTATION) {
    if (event->type == SENSOR_TYPE_ORIENTATION) {
//    Serial.print("Orient:");
    x_ori = event->orientation.x;
    y_ori = event->orientation.y;
    z_ori = event->orientation.z;

//    printserialmonitor(x_ori, y_ori, z_ori);
  }
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

//---------------SETUP----------------
//---------------SETUP----------------

void setup() {
  Serial.begin(9600);
  Serial7.begin(9600);
//Aktuator pin set up
  servo1.attach(2);
  servo2.attach(3); 
  servo3.attach(4); 
  servo4.attach(5);
//servo set to zero   
  servo1.write(i);              
  servo2.write(i);     
  servo3.write(i);
  servo4.write(i); 
  
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
                  Adafruit_BMP280::SAMPLING_X4,    /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_X8,      /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */

  Serial.println("BMP280 : Ready");
//--------------------------------------
//Start GPS
  Serial2.begin(GPSBaud);
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
  Serial.println("BNO055 : Please calibrate");  

//----------------------------------------
//Start MiCS5524

// while(!mics.begin()){
//    Serial.println("Check MiCS5524");
//    delay(1000);
//  } 
//
//  uint8_t mode = mics.getPowerState();
//  if(mode == SLEEP_MODE){
//    mics.wakeUpMode();
//    Serial.println("MiCS5524 : wake up sensor success!");
//  }else{
//    Serial.println("MiCS5524 : The sensor is wake up mode");
//  }
//
//  while(!mics.warmUpTime(CALIBRATION_TIME)){
//    Serial.println("MiCS5524 : warm up...");
//    delay(1000);
//  }
//  Serial.println("");
//-----------------------------------------------------------
//RTC Set Time
   setTime(jam, menit, det, tgl, bln, th);
//Telemetry setup
  Serial1.write("B");
  delay(100);
  Serial1.write("B");
}

//----------LOOP---------------------------
//----------LOOP---------------------------

void loop() {
    // can now print out the new measurements
    if (simulation == false)
    {
      alt = bmp.readAltitude(SLP);
      alt_rel=alt-alt_reff;
      temp=bmp.readTemperature();
    }
//------------------------------------------------------    
//BNO055
//could add VECTOR_ACCELEROMETER, VECTOR_MAGNETOMETER,VECTOR_GRAVITY...
  sensors_event_t orientationData , angVelocityData , linearAccelData, magnetometerData, accelerometerData, gravityData;
  bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
  bno.getEvent(&accelerometerData, Adafruit_BNO055::VECTOR_ACCELEROMETER);
//  bno.getEvent(&gravityData, Adafruit_BNO055::VECTOR_GRAVITY);

  printEvent(&orientationData);
  printEvent(&accelerometerData);
//  printEvent(&gravityData);

    acc[0]=x_acc;
    acc[1]=y_acc;
    acc[2]=z_acc;

  accSigma=pow(acc[0],2)+pow(acc[1],2)+pow(acc[2],2);
  accSigma=abs(pow(accSigma,0.5));
  accRel=accSigma-accRef;
  accSigma=0;

//--------------------------------------------------------

//MiCS 5524
//  gasCO = mics.getGasData(CO);
//  gasCH4 = mics.getGasData(CH4);
//  gasEthanol = mics.getGasData(C2H5OH);
//  gasH2 = mics.getGasData(H2);
//  gasNH3 = mics.getGasData(NH3);
//  gasNO2 = mics.getGasData(NO2);
//-------------------------------------------------------
//GPS
  // This sketch displays information every time a new sentence is correctly encoded.
  while (Serial2.available() > 0)
    if (gps.encode(Serial2.read()))
      displayInfo();
//----------------------------------------------------
//CALIBRATION
  if (CAL_proc==true)
  {
    uint8_t sys, gyro, accel, mag = 0;
    bno.getCalibration(&sys, &gyro, &accel, &mag); 

    if (gyro!=0 & accel !=0 & mag !=0)
    {
     IMU_cal=true;
          //Altimeter refference calibration  
     int count=0; 
     Serial.println("Start altimeter calibration");              
     while(count <= 100){ 
       alt_reff+=bmp.readAltitude(SLP);
       delay(10);
       count++; 
     }
     alt_reff/=count;
     Serial.println("");
     Serial.print(F("Refference altitude = "));
     Serial.print(alt_reff); 
     Serial.println(" m");  
     delay(1000);
    CAL_proc=false;
    Serial.println("IMU & altimeter calibration Succes");
    }
    else
    {
     IMU_cal=false;
     Serial.println("IMU not calibrate");
     Serial.println("Please calibrate IMU"); 
    }
  }

//-----------------------------------------------------  
//Recieve Command
while (Serial7.available()>0)
{
  delay(5);
  c = Serial7.read();
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
  else if (Command=="CAL")
  {
    if (Act=="START")
    {
      CAL_proc=true;
    }
    else if (Act=="FINISH")
    {
      CAL_proc=false;
    }
  }
  else if (Command=="LVL")
  {
    if (Act=="START")
    {
      LVL_cal=true;
        int count=0;
        float Acc_avg[3]={0,0,0}; 
        Serial.println("Start level calibration");              
        while(count <= 1000){ 
          Acc_avg[0]+=x_acc;
          Acc_avg[1]+=y_acc;
          Acc_avg[2]+=z_acc;
          count++;
          delay(10); 
          } 
          Acc_avg[0]/=count;
          Acc_avg[1]/=count;
          Acc_avg[2]/=count; 
      
        accRef=pow(Acc_avg[0],2)+pow(Acc_avg[1],2)+pow(Acc_avg[2],2);
        accRef=pow(accRef,0.5);
        Serial.print("Acc ref : ");
        Serial.println(accRef);
    }
    else if (Act=="FINISH")
    {
      LVL_cal=false;
    }
  }
}
}
inStr="";
ID_valid==false;
Command="";
Act="";
//----------------------------------------------------
//Aktuator Control
/* Periksa apakah wahana telah terbang atau belum dengan memeriksa ketinggian
 *  wahana. Self standing hanya akan aktif jika wahana telah mencapai ketinggian
 *  tertentu dan telah melakukan descent yang ditunjukan oleh kecepatan vertikal bernilai
 *  negatif
 */

if (alt_rel > 2 && vSpeed < -2)
{
  selfStanding=true; 
                       
}

if (selfStanding!=false && accRel<1 && abs(vSpeed)<0.5)
{
  Serial.println("Salah woy"); 
   for (i = 0; i < 180; i++) { 
    servo1.write(i);              
    servo2.write(i);     
    servo3.write(i);
    servo4.write(i);   
//    Serial.println(accRel);
//    Serial.println(abs(vSpeed));      
    delay(20);  
    }
    Serial.print(selfStanding);
}

//----------------------------------------------------
//Transmit data
  time_now = millis();
  if(time_now - time_last >= 1000){
    time_last=time_now;
    vSpeed=(alt_rel-alt_last);
    sprintf(buff, "<Diposonda,%d : %d : %d : %d,%d,%c,%.2f,%c,%c,%.2f,%.2f,%f,%f,%d,%.2f,%.2f,%d,%d,%d,%d,%.2f>", 
    jam,menit,det,csc,data_count,flt_md,alt_rel,probe_stat,selfStand_stat,temp,alt_gps,lat_val,lng_val,sat_val,y_ori,z_ori,gyro,accel,mag,CAL_proc,vSpeed);
    digitalClockDisplay();
    Serial.println(buff);
    Serial.print("Vspeed");
    Serial.println(abs(vSpeed)); 
    Serial.print("alt_rel");
    Serial.println(alt_rel);
    Serial.print("alt_last");
    Serial.println(alt_last);
    Serial.println(selfStanding);
    
    if (TX_on==true)
    {
      Serial7.write(buff);
    }
    alt_last=alt_rel;
    data_count++;
    
  }
}