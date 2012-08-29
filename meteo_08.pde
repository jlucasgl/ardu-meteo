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

  
    Copyright (C) 2011  JLucasGL

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
version: 0.8
autor: JLucasGL
date: 6-9-11
*/

#define METEO_VERSION "meteo 0.8"

#include <WProgram.h>
#include <Wire.h>

/*-----( RTC )-----*/

#include <DS1307.h>        // written by  mattt on the Arduino forum and
                           // modified by D. Sjunnesson
/*-----( RF )-----*/

#include <VirtualWire.h>

#define DATA_TX_PIN 5
#define NOT_USE_RX          // Not used rx pin
#define NOT_USE_PTT         // Not used ptt pin

/*-----( SD )-----*/

#include <SD.h>

// On the Ethernet Shield, CS is pin 4. Note that even if it's not
// used as the CS pin, the hardware CS pin (10 on most Arduino boards,
// 53 on the Mega) must be left as an output or the SD library
// functions will not work.
const int chipSelect = 10;

/*-----( INT )-----*/

#include <stdint.h>
#include <avr/interrupt.h>
#include <avr/wdt.h>

#include <avr/power.h>
#include <avr/sleep.h>

/*-----( POWER )-----*/

#define VCCPIN  3
#define GNDPIN  2

#define THERM_PIN   0  // 10ktherm & 10k resistor as divider.

/*-----( DHT11 )-----*/
#include <dht11.h>

dht11 DHT11;

#define DHT11PIN  4

/***************
* Initial setup
****************/
void setup()
{
  Serial.begin(9600);
  Serial.println(" -***************************- ");

  Serial.print("Powering RTC...  ");
  pinMode(VCCPIN, OUTPUT);
  digitalWrite(VCCPIN, HIGH);
  pinMode(GNDPIN, OUTPUT);
  digitalWrite(GNDPIN, LOW);
  Serial.println("        [OK]");
  //reset_RTC();
  
  // Initialise the IO and ISR
  Serial.print("Initializing RF-TX...");
  vw_set_tx_pin(DATA_TX_PIN);
  vw_setup(2000);	 // Bits per sec
  Serial.println("    [OK]");
  
  // Transmit start signal
  char *msg = METEO_VERSION;
  digitalWrite(13, true); // Flash a light to show transmitting
  vw_send((uint8_t *)msg, strlen(msg));
  vw_wait_tx(); // Wait until the whole message is gone
  digitalWrite(13, false);
  
  
  // DHT11
  Serial.print("DHT11 LIBRARY VERSION: ");
  Serial.println(DHT11LIB_VERSION);

  Serial.print("Initializing SD card...");
  // make sure that the default chip select pin is set to
  // output, even if you don't use it:
  pinMode(10, OUTPUT);
  
  // see if the card is present and can be initialized:
  if (!SD.begin(chipSelect)) {
    Serial.println("Card failed, or not present");
    // don't do anything more:
    return;
  }
  else {
    Serial.println("  [OK]");
  }

  // Periodic reset evrey 8 s
  wdt_enable(WDTO_8S);        
  Serial.println("Watchdog ...             [OK]");
  
  Serial.println(" -***************************- ");
  Serial.println(METEO_VERSION);
}

// Data vars
int t, h;
int sec, minu, hr, dow, day, mth, yr;

// Reset counter
unsigned int despertar = 30;   // 30 = every 4 min

/***********
* Main loop
************/
void loop()
{
  //*************************************************
  
  if (despertar >= 30) {
    //*************************************************
    digitalWrite(VCCPIN, HIGH);
    delay(100);
  
    // Calculate thermistor sensors data
    //therm = Thermister(analogRead(THERM_PIN));
    
    // Calculate dht11 sensor data    
    int chk = DHT11.read(DHT11PIN);
    
    Serial.print("Read sensor: ");
    switch (chk)
    {
      case 0: Serial.println("OK"); break;
      case -1: Serial.println("Checksum error"); break;
      case -2: Serial.println("Time out error"); break;
      default: Serial.println("Unknown error"); break;
    }

    // Get sensors and RTC data
    if (!chk) get_dht11();
    get_time();

    // Print data to Serial
    to_Serial(chk);
    Serial.println();
    
    // open the file. note that only one file can be open at a time,
    // so you have to close this one before opening another.
    File dataFile = SD.open("datalog.txt", FILE_WRITE);

    // if the file is available, write to it:
    if (dataFile) {
      to_File(dataFile, chk);
      dataFile.println();
      dataFile.close();
    }  
    // if the file isn't open, pop up an error:
    else {
      Serial.println("error opening datalog.txt");
    }

    char msg[101];  //buffer used to format a line (+1 is for trailing 0)
    if (!chk)
      sprintf(msg, "T:%dC H:%d\% %d:%d:%d %d, %d/%d/%d", t, h, hr, minu, sec, dow, day, mth, yr);
    else sprintf(msg, "            %d:%d:%d %d, %d/%d/%d", hr, minu, sec, dow, day, mth, yr);
    Serial.print(msg);

    // Transmit dataString via RF
    digitalWrite(13, true); // Flash a light to show transmitting
    vw_send((uint8_t *)msg, strlen(msg));
    vw_wait_tx(); // Wait until the whole message is gone
    digitalWrite(13, false);

    // Going to sleep ...
    digitalWrite(VCCPIN, LOW);
    
    despertar = 0;

    //*************************************************  
  }

  //*************************************************  

  delay(100);     // this delay is needed, the sleep 
                      //function will provoke a Serial error otherwise!!
  sleepNow();     // sleep function called here

  //*************************************************  

  despertar++;

  //*************************************************  
}

/******************************************************************************
*
/******************************************************************************/

void get_time() {
  hr   = RTC.get(DS1307_HR,true);
  minu = RTC.get(DS1307_MIN,false);
  sec  = RTC.get(DS1307_SEC,false);
  dow  = RTC.get(DS1307_DOW,false);
  day  = RTC.get(DS1307_DATE,false);
  mth  = RTC.get(DS1307_MTH,false);
  yr   = RTC.get(DS1307_YR,false);
}

/******************************************************************************
*
/******************************************************************************/
void get_dht11() {
  t    = DHT11.temperature;
  h    = DHT11.humidity;
}

/******************************************************************************
*
******************************************************************************/
void to_Serial(int chk) {
  if (chk == 0) {
    Serial.print("T:");
    Serial.print(t);
    Serial.print("C H:");
    Serial.print(h);
    Serial.print("%   ");
  } else {
    Serial.print("Err: ");
    Serial.print(chk);
    Serial.print("        ");
  }
  Serial.print(hr);
  Serial.print(":");
  Serial.print(minu);
  Serial.print(":");
  Serial.print(sec);
  Serial.print(" ");
  Serial.print(dow);
  Serial.print(", ");
  Serial.print(day);
  Serial.print("/");
  Serial.print(mth);
  Serial.print("/");
  Serial.print(yr);
}

/******************************************************************************
*
******************************************************************************/
void to_File(File dataFile, int chk) {
  if (chk == 0) {
    dataFile.print("T:");
    dataFile.print(t);
    dataFile.print("C H:");
    dataFile.print(h);
    dataFile.print("%   ");
  } else {
    dataFile.print("Err: ");
    dataFile.print(chk);
    dataFile.print("        ");
  }
  dataFile.print(hr);
  dataFile.print(":");
  dataFile.print(minu);
  dataFile.print(":");
  dataFile.print(sec);
  dataFile.print(" ");
  dataFile.print(dow);
  dataFile.print(", ");
  dataFile.print(day);
  dataFile.print("/");
  dataFile.print(mth);
  dataFile.print("/");
  dataFile.print(yr);
}

/******************************************************************************
*
******************************************************************************/
double Thermister(int RawADC) {
 double Temp;
 Temp = log(((10240000/RawADC) - 10000));
 Temp = 1 / (0.001129148 + (0.000234125 * Temp) + (0.0000000876741 * Temp * Temp * Temp));
 Temp = Temp - 273.15;            // Convert Kelvin to Celcius
 //Temp = (Temp * 9.0)/ 5.0 + 32.0; // Convert Celcius to Fahrenheit
 return Temp;
}

/******************************************************************************
*
******************************************************************************/
void set_time() {
  RTC.stop();
  RTC.set(DS1307_SEC, sec);        //set the seconds
  RTC.set(DS1307_MIN, minu);     //set the minutes
  RTC.set(DS1307_HR, hr);       //set the hours
  RTC.set(DS1307_DOW, dow);       //set the day of the week
  RTC.set(DS1307_DATE, day);       //set the date
  RTC.set(DS1307_MTH, mth);        //set the month
  RTC.set(DS1307_YR, yr);         //set the year
  RTC.start();
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
  Serial.println("Modify date? (Y/n): ");
  if (serial_get_byte() == 'Y') {
    Serial.println("Year(p.e. 2011): ");   
    yr = get_string().toInt() - 2000;
    
    Serial.println("Month(p.e. 6): ");
    mth = get_string().toInt();
    
    Serial.println("Day of the month(p.e. 31): ");
    day = get_string().toInt();
    
    Serial.println("Day of the week(1 = Monday): ");
    dow = get_string().toInt();
    
    Serial.println("Hour(p.e. 22): "); 
    hr = get_string().toInt();
    
    Serial.println("Minutes(0-59): "); 
    minu = get_string().toInt();
    
    Serial.println("Seconds(0-59): ");
    sec = get_string().toInt();

    set_time();
  }

  Serial.print("Present time: ");
  to_Serial(1);
  Serial.println();
}

/*
 * Watchdog Timer Interrupt
 */
ISR(WDT_vect)
{
  //Serial.println("Watchdog!");
}

/******************************************************************************
*
******************************************************************************/
void sleepNow()
{
    /* Now is the time to set the sleep mode. In the Atmega8 datasheet
     * http://www.atmel.com/dyn/resources/prod_documents/doc2486.pdf on page 35
     * there is a list of sleep modes which explains which clocks and 
     * wake up sources are available in which sleep modus.
     *
     * In the avr/sleep.h file, the call names of these sleep modus are to be found:
     *
     * The 5 different modes are:
     *     SLEEP_MODE_IDLE         -the least power savings 
     *     SLEEP_MODE_ADC
     *     SLEEP_MODE_PWR_SAVE
     *     SLEEP_MODE_STANDBY
     *     SLEEP_MODE_PWR_DOWN     -the most power savings
     *
     *  the power reduction management <avr/power.h>  is described in 
     *  http://www.nongnu.org/avr-libc/user-manual/group__avr__power.html
     */  
     
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);   // sleep mode is set here

  sleep_enable();          // enables the sleep bit in the mcucr register
                             // so sleep is possible. just a safety pin 
  power_all_disable();
  Serial.println("to sleep ...");  
  sleep_mode();            // here the device is actually put to sleep!!
                              // THE PROGRAM CONTINUES FROM HERE AFTER WAKING UP
  sleep_disable();         // first thing after waking from sleep:
                            // disable sleep...                          
  power_all_enable();
  Serial.println("end.");
}
