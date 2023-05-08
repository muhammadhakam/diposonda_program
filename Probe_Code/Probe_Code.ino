
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_BMP280.h>
#include <TimeLib.h>
#include <Servo.h>

Servo diposervo;

String inStr;
int SLP = 1008;
float alt_reff;
float alt_rel;
float temp;
float alt;
unsigned long time_now;
unsigned long time_last=0;
unsigned int data_count=0;
Adafruit_BMP280 bmp; // I2C
char c;
int Index1,Index2,Index3,Index4,Index5;
int jam = 00;
int menit = 00;
int det = 00;
int tgl = 25;
int bln = 6;
int th = 23;



char buff[250];

void digitalClockDisplay(){
   // digital clock display of the time
   Serial.print(hour());
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

void setup() {
  Serial.begin(9600);
  Serial2.begin(9600);  // wait for native usb
  diposervo.attach(6);
  diposervo.write(0);
  Serial.println(F("BMP280 test"));
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
                  Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */
  Serial.println("BMP280 : Ready");
  int count=0;
  Serial.print("Altimeter Init");
  while (count<=10000) {
    alt_reff+=bmp.readAltitude(SLP);
    count++;
  }
  alt_reff/=count;
  Serial.println("");
  Serial.print(F("Refference altitude = "));
  Serial.print(alt_reff);
  Serial.println(" m");
  delay(1000);

  setTime(jam, menit, det, tgl, bln, th);
  //Telemetry Setup
  Serial2.write("A");
  delay(100);
  Serial2.write("A");
}

void loop() {
    alt = bmp.readAltitude(SLP);
    alt_rel = alt-alt_reff;
    temp = bmp.readTemperature();
    if (alt < 166) {
      diposervo.write(180);
    } else {
      diposervo.write(0);
    }
    while (Serial2.available()>0)
    {
      delay(5);
      c = Serial2.read();
      Serial2.println(c);
      inStr += c; 
    }

    if (inStr.length()>0){
      Serial.println(inStr);}

    time_now = millis();
    if (time_now - time_last >=1000) {
     time_last-time_now;
     sprintf(buff, "<Dipo,%.2f,%.2f,%2.f>", alt_rel,alt,temp);
     delay(1000);
     //digitalClockDisplay();
     //Serial.println(buff);
     Serial.println(buff);
     Serial2.write(buff);
     data_count++;
     
    }
    

}
