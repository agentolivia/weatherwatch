/*************************************************** 
  This is a program for the wearable weather watch
  Instructions on how you build your own can be found on instructables.com
  
    
  Written by Agent Mess.
  Much credit goes to Adafruit Industries and everyone involved in writing the libraries I used.
  Feel free to do whatever you want with this code.
    
  GNU license
 ****************************************************/

#include <Wire.h>
#include "Adafruit_LEDBackpack.h"
#include "Adafruit_GFX.h"
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>
#include <Adafruit_GPS.h>
#include <SoftwareSerial.h>
#include "TimerOne.h"


Adafruit_GPS GPS(&Serial1);

Adafruit_BMP280 bme;
Adafruit_8x16matrix matrix = Adafruit_8x16matrix();

#define GPSECHO false
boolean usingInterrupt = false;
boolean fixit=false;
int i;
float gps_altitude=0.0;
float values_pressure[12];
float mean_pressure=0.0;

volatile float pressure_read=0.0;

volatile float mom_altitude=0.0;
float delta_altitude=0.0;
float delta_pressure=0.0;
float delta_pressure_shorttime=0.0;
int j;
int k;

uint32_t timer = millis();
uint32_t timer2 = millis();
int forecast=2;


//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Weather Icons
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static const uint8_t PROGMEM
    cloud_bmp[] =  
    { B00001100,
      B00011110,
      B00111111,
      B00111111,
      B00111111,
      B00011111,
      B01111111,
      B11111111,
      B11111111,
      B11111111,
      B11111111,
      B01111111,
      B01111111,
      B00011111,
      B00001110,
      B00000100} ,
    sun_bmp[] =
    { B00010000,
      B10010010,
      B01010100,
      B00111000,
      B11111110,
      B00111000,
      B01010100,
      B10010010,
      B00010000},
    storm1_bmp[] =
    { B01000000,
      B11000000,    
      B11000000,
      B11000000,
      B11100010,
      B11010101,
      B11001000,
      B11000000,
      B11000000,      
      B01000000}, 
    storm2_bmp[] =
    { B01000000,
      B11000000,    
      B11000000,
      B11001000,
      B11010101,
      B11100010,
      B11000000,
      B11000000,
      B11000000,      
      B01000000},              
    rain1_bmp[] =
    { B01000000,
      B11000000,    
      B11110000,
      B11000000,
      B11011000,
      B11000000,
      B11100001,
      B11000000,
      B11000000,      
      B01000000},
    rain2_bmp[] =
    { B01000000,
      B11000000,    
      B11001100,
      B11000000,
      B11000110,
      B11000000,
      B11110000,
      B11000000,
      B11000000,      
      B01000000},
    rain3_bmp[] =
    { B01000000,
      B11000000,
      B11000011,
      B11000000,
      B11000001,
      B11000000,
      B11001100,
      B11000000,
      B11000000,
      B01000000},
    rain4_bmp[] =
    { B01000000,
      B11000000,    
      B11000000,
      B11000000,
      B11100000,
      B11000000,
      B11000011,
      B11000000,
      B11000000,      
      B01000000};


//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Setup
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++


void setup() {
  Serial.begin(9600);
    if (!bme.begin()) {  
    Serial.println("Could not find a valid BMP280 sensor, check wiring!");
    while (1);
  }
  matrix.begin(0x70);  // pass in the address
  Serial.println("Adafruit GPS library basic test!");
  GPS.begin(9600);
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ); // 1 Hz update rate
  GPS.sendCommand(PGCMD_ANTENNA);
  delay(1000);
  Serial1.println(PMTK_Q_RELEASE);

  Timer1.initialize(1000);
  Timer1.attachInterrupt(readGPS);   
}



//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// LOOP
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

void loop() {

 while (fixit==false){    
   if (GPS.fix) {
     gps_altitude = GPS.altitude;
     fixit=true; 
     init_forecast(gps_altitude);
  }
   else {
    matrix.setTextSize(1);
    matrix.setTextWrap(false);  // we dont want text to wrap so it scrolls nicely
    matrix.setTextColor(LED_ON);
    matrix.setRotation(1);
    for (int8_t x=20; x>=-75; x--) {
        matrix.clear();
        matrix.setCursor(x,0);
        matrix.print("Wait for GPS");
        matrix.writeDisplay();
        delay(100);    
     }    
     matrix.setRotation(2);
   }  
  }


  
  if (timer2 > millis()) timer2 = millis();       
   // approximately every  5 minutes  or so, make a new prediction
  if (millis() - timer2 > 300000) {
    timer2 = millis(); // reset the timer
    pressure_read = bme.readPressure(); 
    if (GPS.fix) {
      gps_altitude = GPS.altitude;
    }
    forecast = makeforecast(gps_altitude); 
  }
  

  switch (forecast) {
    case 1:
      drawsun();
      return;
    case 2:
      drawcloud();
      return;
    case 3:
      drawstorm();
      return;
    case 4:
      drawrain();
      return;
    case 5:
      if (bme.readTemperature() > 15) {
        drawsun();
        return;
      }
      else {
        drawcloud();  
        return;      
      }  
    default: 
      drawsun();
      return;      
    }    
}


/**************************
 * Functions
***************************/

void readGPS() {
 char c = GPS.read();
  if (GPSECHO)
    if (c) Serial.print(c);
  if (GPS.newNMEAreceived()) {
    Serial.println(GPS.lastNMEA()); // this also sets the newNMEAreceived() flag to false
    if (!GPS.parse(GPS.lastNMEA())) // this also sets the newNMEAreceived() flag to false
      return; // we can fail to parse a sentence in which case we should just wait for another
    }
}


void drawcloud() {
  matrix.clear();
  matrix.drawBitmap(0, 0, cloud_bmp, 8, 16, LED_ON);
  matrix.writeDisplay();
  delay(1200);
  }

void drawsun() {
  matrix.clear();
  matrix.drawBitmap(0, 3, sun_bmp, 8, 9, LED_ON);
  matrix.writeDisplay();
  delay(1200);
  }


void drawrain () {
  for (i=0; i<=2; i++) {  
    matrix.clear();
    matrix.drawBitmap(0, 3, rain1_bmp, 8, 10, LED_ON);
    matrix.writeDisplay();
    delay(100);
  
    matrix.clear();
    matrix.drawBitmap(0, 3, rain2_bmp, 8, 10, LED_ON);
    matrix.writeDisplay();
    delay(100);

    matrix.clear();
    matrix.drawBitmap(0, 3, rain3_bmp, 8, 10, LED_ON);
    matrix.writeDisplay();
    delay(100);
  
    matrix.clear();
    matrix.drawBitmap(0, 3, rain4_bmp, 8, 10, LED_ON);
    matrix.writeDisplay();
    delay(100);
  }  
}

void drawstorm () {
  for (i=0; i<=2; i++) {  
    matrix.clear();
    matrix.drawBitmap(0, 3, storm1_bmp, 8, 10, LED_ON);
    matrix.writeDisplay();
    delay(100);
  
    matrix.clear();
    matrix.drawBitmap(0, 3, storm2_bmp, 8, 10, LED_ON);
    matrix.writeDisplay();
    delay(100);

    matrix.clear();
    matrix.drawBitmap(0, 3, storm1_bmp, 8, 10, LED_ON);
    matrix.writeDisplay();
    delay(100);
  
    matrix.clear();
    matrix.drawBitmap(0, 3, storm2_bmp, 8, 10, LED_ON);
    matrix.writeDisplay();
    delay(100);
  }  
}



void init_forecast(float gps_altitude) {

  mean_pressure= pressure_read;

  //convert pressure to altitude
  mom_altitude= (288.15/0.0065)*(1-pow((mean_pressure/101325),0.1903));
  
  //delta_altitude = pressurealtitude - gpsaltitude
  delta_altitude = mom_altitude - gps_altitude;
  
  //convert delta_altitude to pressure
  mean_pressure = 101325*pow((1-((0,0065*delta_altitude)/288,15)),5.255);

    for (j=0;j<12;j++) {               //fill the array with the initial value
    values_pressure[j] = mean_pressure;
    }  
}



int makeforecast(float gps_altitude) {

  mean_pressure= pressure_read;

  //convert pressure to altitude
  mom_altitude= (288.15/0.0065)*(1-pow((mean_pressure/101325),0.1903));
  
  //delta_altitude = pressurealtitude - gpsaltitude
  delta_altitude = mom_altitude - gps_altitude;
  
  //convert delta_altitude to pressure
  mean_pressure = 101325*pow((1-((0.0065*delta_altitude)/288.15)),5.255);
  
  for (j=11;j>0;j--) {               //save momentaneous pressure in array in position 0
    values_pressure[j] = values_pressure[j-1];
    }
  values_pressure[0]=mean_pressure;

  //calculate difference between old pressure and momentaneous pressure
  delta_pressure = values_pressure[0] - values_pressure[11];
  delta_pressure_shorttime = values_pressure [0] - values_pressure[1];

  if (delta_pressure_shorttime < -30) return 3; // strong wind, storm (early warning)
  else {
   if (delta_pressure > 100) return 1; // weather gets better (short term) 
    else if ((delta_pressure <= 100) and (delta_pressure >= 20)) return 2; // weather gets better (long term)
  
    else if (delta_pressure < -200) return 3; // strong wind, storm 

    else if ((delta_pressure >= -200) and (delta_pressure <= -20)) return 4; // bad weather

    else if ((delta_pressure < 20) and (delta_pressure > -20)) return 5; // weather will remain the same  
  }
}
