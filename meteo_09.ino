/*
    Reads the value from a Real Time Clock (RTC) DS1307 and the value from a thermistor, and 
    then save them in SD card and displays in the serial monitor. 
    Based on creation of D. Sjunnesson 1scale1.com d.sjunnesson (at) 1scale1.com
 
   &
 
   YourDuino.com Example Software Sketch
   DHT11 Humidity and Temperature Sensor test
   Credits: Rob Tillaart
   http://arduino-direct.com/sunshop/index.php?l=product_detail&p=162
   terry@yourduino.com
  
  &
  
   Modified version of VirtualWire lib based on:
  
   Virtual Wire implementation for Arduino
   See the README file in this directory fdor documentation
   
   Author: Mike McCauley (mikem@open.com.au)
   Copyright (C) 2008 Mike McCauley
   $Id: VirtualWire.h,v 1.3 2009/03/30 00:07:24 mikem Exp $

  
    Copyright (C) 2012  JLucasGL
    Copyright (C) 2012  JMChaconDLR

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/
/*
name: meteo
version: 0.9
autor: JLucasGL / JMChaconDLR
date: 01-9-12
*/

#define METEO_VERSION "meteo 0.9"

#define DEBUG 1           // comment to avoid serial print mesagges
//#define SET_RTC 1         // comment to avoid set RTC

#include <Wire.h>
#include <WProgram.h>

/*-----( WATCHDOG )-----*/

#include <avr/sleep.h>    //sleep library
#include <avr/wdt.h>      //watchdog timer library
#define sleepTime 75      //number of 8 second sleep cycles  // 75 = 10m, 450 = 1h
unsigned int time;

//volatile byte wdt=0;    //used as sleep counter incrimented in watchdog ISR
volatile byte wdt=0;    //used as sleep counter incrimented in watchdog ISR

/*-----( RTC )-----*/

#include <DS1307new.h>
//#include <DS1307.h>        // written by  mattt on the Arduino forum and
                           // modified by D. Sjunnesson
/*-----( SD )-----*/

#include <SdFat.h>
#include <SdFatUtil.h>      // Use PgmPrint

// SD chip select pin
const uint8_t chipSelect = SS;
// file system object
SdFat sd;

char name[] = "meteo.txt";
int SD_initialized = 0;

/*-----( Stream )-----*/

//#include <bufstream.h>

// Serial print stream
ArduinoOutStream cout(Serial);

// buffer to format data - makes it eaiser to echo to Serial
char buf[80];

// store error strings in flash to save RAM
//#define error(s) sd.errorHalt_P(PSTR(s))
#define error(s) sd.errorPrint_P(PSTR(s))

/*-----( INT )-----*/

#include <stdint.h>
#include <avr/interrupt.h>
#include <avr/wdt.h>

#include <avr/power.h>
#include <avr/sleep.h>

/*-----( DHT11 )-----*/
#include <dht11.h>

dht11 DHT11;

#define DHT11PIN 9

// Data vars
int t, h;
int sec, minu, hr, dow, day, mth, yr;
int chk;


/***************
* Initial setup
****************/
void setup()
{
  // Initializing Serial
  Serial.begin(9600);
  
  #ifdef DEBUG
  Serial.println(" -***************************- ");
  #endif
  
  #ifdef SET_RTC
  reset_RTC();
  #endif
    
  // DHT11
  #ifdef DEBUG
  PgmPrint("DHT11 LIBRARY VERSION: ");
  PgmPrintln(DHT11LIB_VERSION);
  
  PgmPrint("Checking DHT11 ...       ");
  if (!DHT11.read(DHT11PIN)) PgmPrintln("[OK]");
  else PgmPrintln("[FAIL]");  
  #endif
  
  // SD
  #ifdef DEBUG
  PgmPrint("SD_FAT_VERSION: ");
  cout << SD_FAT_VERSION << endl;
  PgmPrint("Initializing SD card...");
  #endif
  // make sure that the default chip select pin is set to
  // output, even if you don't use it:
  pinMode(10, OUTPUT);
  
  // see if the card is present and can be initialized:
  if (!sd.begin(chipSelect, SPI_HALF_SPEED)) {
     sd.initErrorPrint();
  }
  else {
    SD_initialized = 1;
    PgmPrintln("  [OK]");
  }
  
  PgmPrint("Checking DS1307 ...      ");
  if (RTC.isPresent()) PgmPrintln("[OK]");
  else PgmPrintln("[FAIL]");

  // Periodic reset evrey 8 s
  setup_watchdog();
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);     // sleep mode is set here Power Down uses the least current
                                           // system clock is turned off, so millis won't be reliable!
  #ifdef DEBUG
  PgmPrintln("Watchdog ...             [OK]");
  
  PgmPrintln(" -***************************- ");
  PgmPrintln(METEO_VERSION);
  PgmPrintln("");
  #endif
  
}

/***********
* Main loop
************/
void loop()
{

  //*************************************************
  delay(100);

  get_Data();
  dump_Data();

  //*************************************************  

  delay(100);     // this delay is needed, the sleep 
                      //function will provoke a Serial error otherwise!!
                      
  system_sleep();

}

/******************************************************************************
*
******************************************************************************/

void get_Data() {
  
  // Read RTC
  RTC.getTime();
  
  // Calculate dht11 sensor data
  chk = DHT11.read(DHT11PIN);
    
  #ifdef DEBUG
  Serial.print("Read sensor: ");
  switch (chk)
  {
    case 0: Serial.println("OK"); break;
    case -1: Serial.println("Checksum error"); break;
    case -2: Serial.println("Time out error"); break;
    default: Serial.println("Unknown error"); break;
  }
  #endif

  // Get sensors and RTC data
  if (!chk) get_dht11();
  get_time();
  
}

void get_time() {
  hr   = RTC.hour;
  minu = RTC.minute;
  sec  = RTC.second;
  dow  = RTC.dow;
  day  = RTC.day;
  mth  = RTC.month;
  yr   = RTC.year;
}

void get_dht11() {
  t    = DHT11.temperature;
  h    = DHT11.humidity;
}

/******************************************************************************
*
******************************************************************************/
ostream& flushOp (ostream &os) {
  os << flush << hr << pstr(":") << minu << pstr(":") << sec << pstr(" ");
  switch (dow)                      // Friendly printout the weekday
  {
    case 1: os << pstr("MON"); break;
    case 2: os << pstr("TUE"); break;
    case 3: os << pstr("WED"); break;
    case 4: os << pstr("THU"); break;
    case 5: os << pstr("FRI"); break;
    case 6: os << pstr("SAT"); break;
    case 0:
    case 7: os << pstr("SUN"); break;
    default: os << int(dow) << pstr("?");
  }  
  os << pstr(", ") << day << pstr("/") << mth << pstr("/") << yr << " |";
  return os;
}

void dump_Data() {
  
  // use buffer stream to format line
  obufstream bout(buf, sizeof(buf));
  
  if (chk == 0) {
    bout << pstr("T:") << int(t) << pstr("C H:");
    bout << int(h) << pstr("%   ");
  } else {
    bout << pstr("Err: ") << int(chk);
    bout << pstr("        ");
  }
  flushOp(bout);
  
  // If SD is not initialized then dump by serial and exit
  if (!SD_initialized) {
    cout << buf << endl;
    return;
  }
  
  // open the file. note that only one file can be open at a time,
  // so you have to close this one before opening another.
  ofstream sdout(name, ios::out | ios::app);

  // if the file isn't open, pop up an error:
  if (!sdout) {    
    cout << buf << endl;
    error("open failed");
  }
  // if the file is available, write to it:
  else {
    sdout << buf << endl << flush;
    sdout.close();
  
    if (!sdout) error("append data failed");
  
    // Print data to Serial
    #ifdef DEBUG
    cout << buf << endl;
    #endif
  }
  
}

/******************************************************************************
*
******************************************************************************/
/*double Thermister(int RawADC) {
 double Temp;
 Temp = log(((10240000/RawADC) - 10000));
 Temp = 1 / (0.001129148 + (0.000234125 * Temp) + (0.0000000876741 * Temp * Temp * Temp));
 Temp = Temp - 273.15;            // Convert Kelvin to Celcius
 //Temp = (Temp * 9.0)/ 5.0 + 32.0; // Convert Celcius to Fahrenheit
 return Temp;
}*/

/******************************************************************************
*
******************************************************************************/
void set_time() {  
  RTC.stopClock();
  RTC.fillByHMS(hr, minu, sec);      //set hour minutes & seconds
  RTC.fillByYMD(yr, mth, day);       //set the date
  RTC.setTime();
  RTC.startClock();
}

/******************************************************************************
*
******************************************************************************/
int serial_get_byte(unsigned int timeout=5000)
{
  int incomingByte = 0;	// for incoming serial data
  unsigned long time = millis();
  
  do {
    // send data only when you receive data:
    if (Serial.available() > 0) {
      // read the incoming byte:
      incomingByte = Serial.read();
      return incomingByte;
    }
  } while(millis()-time < timeout);
  return 0;
}

/******************************************************************************
*
******************************************************************************/
String get_string() {
  // Reads first character
  char ch = serial_get_byte(15000);
  String str = String(ch);

  // If nothing was recived in 15s return "0"
  if (!ch) return "0";

  // Reads remaining characters
  unsigned long time = millis();
  while(millis()-time < 500) {
    ch = serial_get_byte(50);
    str += String(ch);
  }
  
  // Return input string
  //Serial.println("get_string() devuelve: '" + str + "'");
  return str;
}

/******************************************************************************
*
******************************************************************************/
void reset_RTC() 
{
  PgmPrint("Modify date? (Y/n): ");
  if (serial_get_byte() == 'Y') {
    get_string();
    
    PgmPrint("Year(p.e. 2011): ");   
    yr = get_string().toInt();
    PgmPrintln("");
    
    PgmPrint("Month(p.e. 6): ");
    mth = get_string().toInt();
    PgmPrintln("");
    
    PgmPrint("Day of the month(p.e. 31): ");
    day = get_string().toInt();
    PgmPrintln("");
    
    PgmPrint("Day of the week(1 = Monday): ");
    dow = get_string().toInt();    
    PgmPrintln("");
    
    PgmPrint("Hour(p.e. 22): "); 
    hr = get_string().toInt();
    PgmPrintln("");
    
    PgmPrint("Minutes(0-59): "); 
    minu = get_string().toInt();
    PgmPrintln("");
    
    PgmPrint("Seconds(0-59): ");
    sec = get_string().toInt();
    PgmPrintln("");

    set_time();
  }

  // use buffer stream to format line
  obufstream bout(buf, sizeof(buf));
  dump_Data();
  cout << "Present time: " << buf << endl;
}

/******************************************************************************
* WATCHDOG from Sol_Arduino V1.0 Prototype Code
******************************************************************************/

//****************************************************************
void setup_watchdog() {
  cli();
  MCUSR = 0x00;  //clear all reset flags 
  //set WD_ChangeEnable and WD_resetEnable to alter the register
  WDTCSR |= (1<<WDCE) | (1<<WDE);   // this is a timed sequence to protect WDTCSR
  // set new watchdog timeout value to 1024K cycles (~8.0 sec)
  WDTCSR = (1<<WDP3) | (1<<WDP0);
  //enable watchdog interrupt
  WDTCSR |= (1<<WDIE);    
  sei();  
}

//****************************************************************

void system_sleep() {
  
  power_all_disable();
  ADCSRA |= (0<<ADEN);                     // switch ADC off
  sleep_enable();                          // enable sleeping
                                           // activate system sleep
  while (wdt < sleepTime) {                // sleep for sleepTime * 8sec
    sleep_mode();                          // activate system sleep
    }
  sleep_disable();                      // disable sleep
  ADCSRA |= (1<<ADEN);                  // switch ADC on    
  time = 8*wdt;                         // calculate total seconds slept
  wdt = 0;                              // reset sleep counter
  power_all_enable();
  delay(2);
}

/*
 * Watchdog Interrupt Service Routine
 * Very first thing after sleep wakes with WDT Interrupt
 */
ISR(WDT_vect)
{
  wdt++;  // increment the watchdog sleep counter
  //cout << int(wdt) << "-" << sleepTime << " " << time << " " << (int(wdt) < sleepTime) << endl;
}

