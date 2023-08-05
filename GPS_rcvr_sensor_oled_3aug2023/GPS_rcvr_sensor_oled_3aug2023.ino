// GNSS receiver with OLED display, what you need is 
//
//  - Any 1.3 inch OLED display, I2C (mine runs on 5V)
//  - Adafruit ultimate v3 breakout board (runs on 5V)
//  - BMP085 sparkfun breakout board (it is I2C and it needs 3.3V, not 5v)
//  - Arduino Nano (I used the Nano every version, it should not matter)
//
//  I2C: SCL is line A5 
//  I2C: SDA is line A4
//  Serial : D8 is TX on GNSS breakout  
//  Serial : D7 is RX on GPS breakout
//
//  John Price (WA2FZW) provided the Maidenhead calculation routine
//
#include <Arduino.h>
#include <Adafruit_GPS.h>     // Library Adafruit for debugging the NMEA code
#include <Adafruit_BMP085.h>  // Bosch pressure and temperature sensor
#include <SoftwareSerial.h>   // Serial interface for the GNSS receiver
#include <U8g2lib.h>          // a very nice library that handles many type of displays
//
// For the Adafruit GPS ultimate V3 breakout board do the following:
//
// Connect the GPS Power pin to 5V
// Connect the GPS Ground pin to ground
// Connect the GPS TX (transmit) pin to Digital 8
// Connect the GPS RX (receive) pin to Digital 7
//
SoftwareSerial mySerial(8, 7);  
Adafruit_GPS GPS(&mySerial);
//
// This is for the 1.3 inch 128x64 OLED display
//
//
#ifdef U8X8_HAVE_HW_SPI
#include <SPI.h>
#endif
#ifdef U8X8_HAVE_HW_I2C
#include <Wire.h>
#endif
//
// Read the documentation of your OLED display, this constructor worked
//
U8G2_SH1106_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE); 
//
// Initialization commands of the GPS receiver
//
#define PMTK_SET_NMEA_UPDATE_1HZ  "$PMTK220,1000*1F"
#define PMTK_SET_NMEA_UPDATE_5HZ  "$PMTK220,200*2C"
#define PMTK_SET_NMEA_UPDATE_10HZ "$PMTK220,100*2F"
//
// turn on only the second sentence (GPRMC)
#define PMTK_SET_NMEA_OUTPUT_RMCONLY "$PMTK314,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*29"
// turn on GPRMC and GGA
#define PMTK_SET_NMEA_OUTPUT_RMCGGA "$PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*28"
// turn on ALL THE DATA
#define PMTK_SET_NMEA_OUTPUT_ALLDATA "$PMTK314,1,1,1,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0*28"
// turn off output
#define PMTK_SET_NMEA_OUTPUT_OFF "$PMTK314,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*28"
//
#define PMTK_Q_RELEASE "$PMTK605*31"
//
// Set GPSECHO to 'false' to turn off echoing the GPS data to the Serial console
// Set to 'true' if you want to debug and listen to the raw GPS sentences
#define GPSECHO  false
//
// Pressure sensor (BMP085 Adafruit) initialization
//
Adafruit_BMP085 bmp;
//
// Maidenhead square routine
//
/*
 *  GetGridSquare ()  By: John Price (WA2FZW)
 *
 *  This function computes the 6-character Maidenhead grid square based on the latitude
 *  and longitude. The arguments are:
 *
 *    double  latitude
 *    double  longitude
 *    char* buffer (at least 7 long)
 *
 *  The latitude and longitude are in the NMEA format. In other words, West longitudes 
 *  and South latitudes are negative numbers. They are specified as "double" types as
 *  that is the type used by the TinyGPS++ library that I use with this.
 *
 *  The buffer in which the grid square designation is constructed must have space for
 *  at least 7 characters (6 for the grid aquare plus a null terminator).
 *  
 *  A great explanation of the math can be found here:
 *
 *    http://n1sv.com/PROJECTS/How%20to%20calculate%20your%208-digit%20grid%20square.pdf
 */

void GetGridSquare ( double _lat, double _long, char* buff )
{

double  tempNumber;         // Used in the intermediate computations
int index;            // Determines character to display

/*
 *  First compute the first 2 characters:
 */

  _long += 180;         // 360 degrees starting from middle of the Pacific
  tempNumber = _long / 20;      // Each major square is 20 degrees wide
  index = (int) tempNumber;     // The index to upper case letters
  buff[0] = index + 'A';        // Set first character
  _long = _long - ( index * 20 );     // Remainder for step 2

  _lat += 90;         // 180 degrees starting from the South pole
  tempNumber = _lat / 10;       // Each major square is 10 degrees high
  index = (int) tempNumber;     // The index to upper case letters
  buff[1] = index + 'A';        // Set second character
  _lat = _lat - ( index * 10 );     // Remainder for step 2


/*
 *  Now the 2nd two digits:
 */

  tempNumber = _long / 2;       // Remainder from step 1 divided by 2
  index = (int) tempNumber;     // Index to digits
  buff[2] = index + '0';        // Set third character
  _long = _long - ( index * 2 );      // Remainder for step 3

  tempNumber = _lat;        // Remainder from step 1 divided by 1
  index = (int) tempNumber;     // Index to digits
  buff[3] = index + '0';        // Set fourth character
  _lat = _lat - index;        // Remainder for step 3


/*
 *  Now the third two characters:
 */

  tempNumber = _long / 0.083333;      // Remainder from step 2 divided by 0.083333
  index = (int) tempNumber;     // The index to lower case letters
  buff[4] = index + 'a';        // Set fifth character

  tempNumber = _lat / 0.0416665;      // Remainder from step 2 divided by 0.0416665
  index = (int) tempNumber;     // The index to lower case letters
  buff[5] = index + 'a';        // Set fifth character
  buff[6] = '\0';         // Null terminator
}

//-------------------------------------------------------------------------------------
//
// Setup routine
//
//-------------------------------------------------------------------------------------

void setup()
{
  // Initialize serial at 115200, you always need it
  //
  Serial.begin(115200);
  delay(5000);
  Serial.println(F("GNSS receiver PA1EJO 2023"));
  //
  // Initialize Adafruit Ultimate GPS Breakout V3
  //
  GPS.begin(9600); // 9600 NMEA is the default baud rate for Adafruit MTK GPS's- some use 4800
  // uncomment this line to turn on RMC (recommended minimum) and GGA (fix data) including altitude
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  // uncomment this line to turn on only the "minimum recommended" data
  //GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCONLY);
  // For parsing data, we don't suggest using anything but either RMC only or RMC+GGA since
  // the parser doesn't care about other sentences at this time
  // Set the update rate
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);   // 1 Hz update rate
  // For the parsing code to work nicely and have time to sort thru the data, and
  // print it out we don't suggest using anything higher than 1 Hz
  // Request updates on antenna status, comment out to keep quiet
  GPS.sendCommand(PGCMD_ANTENNA);
  delay(1000);
  // Ask for firmware version
  mySerial.println(PMTK_Q_RELEASE);
  //
  // initialize 1.3 OLED Display (it has already Wire.h
  //
  u8g2.begin();
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_ncenB08_tr);  // choose a suitable font
  u8g2.drawStr(0,10,"GNSS:"); // write something to the internal memory
  u8g2.sendBuffer(); // transfer internal memory to the display 
  //
  // Initialize by getting the calibration data for the pressure sensor
  //
  if (!bmp.begin()) {
    Serial.println("Could not find a valid BMP085 sensor, check wiring!");
    while (1) {}
  }
  Serial.print("Meteo: "); 
  Serial.print(bmp.readTemperature()); Serial.print(" degC  ");
  Serial.print(bmp.readPressure()/100.0); Serial.println(" hPa");
}

//-------------------------------------------------------------------------------------
//
// the loop updates the display 
//
//-------------------------------------------------------------------------------------

uint32_t timer = millis();
void loop()                     // run over and over again
{
  char c = GPS.read();
  // if you want to debug, this is a good time to do it!
  if ((c) && (GPSECHO))Serial.write(c);
  // if a sentence is received, we can check the checksum, parse it...
  if (GPS.newNMEAreceived()) {
    // a tricky thing here is if we print the NMEA sentence, or data
    // we end up not listening and catching other sentences!
    // so be very wary if using OUTPUT_ALLDATA and trytng to print out data
    //Serial.println(GPS.lastNMEA());   // this also sets the newNMEAreceived() flag to false
    if (!GPS.parse(GPS.lastNMEA()))   // this also sets the newNMEAreceived() flag to false
      return;  // we can fail to parse a sentence in which case we should just wait for another
  }
  // approximately every 1 second, print out the current stats
  if (millis() - timer > 3000) {
    timer = millis(); // reset the timer
    u8g2.clearBuffer();
    int line1 = 10; int line2 = 25; int line3 = 38; int line4 = 51; int line5 = 64;
    u8g2.setFont(u8g2_font_ncenB08_tr);  // choose a suitable font
    u8g2.drawStr(0,line1,"GNSS rx PA1EJO 2023"); // write something to the internal memory
    char info[50]; 
    sprintf(info,"%2.2d-%2.2d-20%2.2d %2.2d:%2.2d:%2.2d",GPS.day,GPS.month,GPS.year,GPS.hour,GPS.minute,GPS.seconds);
    u8g2.drawStr(0,line2,info);
    if (GPS.fix) {     
      float lati = GPS.latitude;   
      double latitude = floor(lati/100.0);
      latitude += (lati - latitude*100.0)/60.0;
      if (GPS.lat == 'S') latitude = -latitude;
      int leadLat = floor(lati); int restLat = (lati - leadLat)*10000;
      float longi = GPS.longitude; 
      double longitude = floor(longi/100.0);
      longitude += (longi - longitude*100.0)/60.0;
      if (GPS.lon == 'W') longitude = -longitude;
      int leadLon = floor(longi); int restLon = (longi - leadLon)*10000;
      snprintf(info,50,"%d.%4.4d%c %d.%4.4d%c", leadLat, restLat, GPS.lat, leadLon, restLon, GPS.lon );
      u8g2.drawStr(0,line3,info);     
      char buffer[10]; GetGridSquare( latitude,longitude,buffer );
      float T = bmp.readTemperature(); float P = bmp.readPressure()/100.0;
      int signT; signT = 0; if (T > 0.0) signT = 1; if (T < 0.0) signT = -1; T = fabs(T);
      int leadT = floor(T); int restT = round((T - float(leadT))*10.0);  //Serial.print(T); Serial.print(" "); Serial.print(leadT); Serial.print("."); Serial.println(restT);
      leadT = signT*leadT;
      int leadP = floor(P); int restP = round((P - float(leadP))*10.0); //Serial.print(P); Serial.print(" "); Serial.print(leadP); Serial.print("."); Serial.println(restP);
      snprintf(info,50,"%s  %d.%d  %d.%d",buffer,leadT,restT,leadP,restP);     
      u8g2.drawStr(0,line4,info);      
      int sp = round((GPS.speed) * 1.852);
      snprintf(info,50,"%2d/%1d S%3d H%3d A%5d",(int)GPS.satellites,(int)GPS.antenna,sp,(int)GPS.angle,(int)GPS.altitude);
      u8g2.drawStr(0,line5,info); 
    } else {
      snprintf(info,50," "); u8g2.drawStr(0,line3,info);
      snprintf(info,50," "); u8g2.drawStr(0,line4,info);
      snprintf(info,50," "); u8g2.drawStr(0,line5,info);
    }
    u8g2.sendBuffer(); 
  }
}
//
// Written by PA1EJO Ernst, last update 4-Aug-2023 15:55
