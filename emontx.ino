/*
  emonTX, Patrik Hermansson 140324
  Measures:
  Temp: DS18B20 (On D4)
  Vcc: Internal Vref
  Hunidity: DHT22 (on D9)
  Air pressure: BMP085 (SCL/SCA)
  Ambient Light: LDR (On A0)
  
  
  Based on:
  emonTX LowPower Temperature Example 
 
  Example for using emonTx with two AA batteries connected to 3.3V rail. 
  Voltage regulator must not be fitted
  Jumper between PWR and Dig7 on JeePort 4

  WHEN IN OPERATION CURRENT CONSUMPTION @ 3.137V = 6.89mA (peak 7mA)
  WHEN SLEEPING                                    0.01mA or maybe less (limit of meter) 

  Temperature sensing takes: 782308 us
  RFM12 Sending takes 2920 us
  TOTAL = 0.785s @ 7mA = 0.017242822 J per pulse
  one every 10s = 0.001724282 W
  baseload = 0.00003137W (worst case)
  = 0.001755652 W
  3000 mah = 2x 10517J ~ 138 Days

  This setup allows the DS18B20 to be switched on/off by turning Dig7 HIGH/LOW
 
  Part of the openenergymonitor.org project
  Licence: GNU GPL V3
 
  Authors: Glyn Hudson, Trystan Lea
  Builds upon JeeLabs RF12 library and Arduino

  THIS SKETCH REQUIRES:

  Libraries in the standard arduino libraries folder:
	- JeeLib		https://github.com/jcw/jeelib
	- OneWire library	http://www.pjrc.com/teensy/td_libs_OneWire.html
	- DallasTemperature	http://download.milesburton.com/Arduino/MaximTemperature/DallasTemperature_LATEST.zip

  Other files in project directory (should appear in the arduino tabs above)
	- emontx_lib.ino
	- print_to_serial.ino

  Set Board as "Uno"
 
*/

/*Recommended node ID allocation
------------------------------------------------------------------------------------------------------------
-ID-	-Node Type- 
0	- Special allocation in JeeLib RFM12 driver - reserved for OOK use
1-4     - Control nodes 
5-10	- Energy monitoring nodes
11-14	--Un-assigned --
15-16	- Base Station & logging nodes
17-30	- Environmental sensing nodes (temperature humidity etc.)
31	- Special allocation in JeeLib RFM12 driver - Node31 can communicate with nodes on any network group
-------------------------------------------------------------------------------------------------------------
*/

#define RF_freq RF12_433MHZ                                                // Frequency of RF12B module can be RF12_433MHZ, RF12_868MHZ or RF12_915MHZ. You should use the one matching the module you have.
const int nodeID = 18;                                                  // emonTx temperature RFM12B node ID - should be unique on network
const int networkGroup = 210;                                           // emonTx RFM12B wireless network group - needs to be same as emonBase and emonGLCD
                                                                                 //DS18B20 resolution 9,10,11 or 12bit corresponding to (0.5, 0.25, 0.125, 0.0625 degrees C LSB), lower resolution means lower power
const int time_between_readings= 60000;                                  //in ms
//const int time_between_readings= 5000;                                  //in ms
//const int time_between_readings= 500;

#include <JeeLib.h>                                                     // Download JeeLib: http://github.com/jcw/jeelib
#include <avr/sleep.h>
ISR(WDT_vect) { Sleepy::watchdogEvent(); }                              // Attached JeeLib sleep function to Atmega328 watchdog -enables MCU to be put into sleep mode inbetween readings to reduce power consumption 

#include <OneWire.h>
#include <DallasTemperature.h>

#define ONE_WIRE_BUS 4                                                  // Data wire is plugged into port 2 on the Arduino
OneWire oneWire(ONE_WIRE_BUS);                                          // Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
DallasTemperature sensors(&oneWire);                                    // Pass our oneWire reference to Dallas Temperature.
DeviceAddress sensor;                                                   // arrays to hold device address

// DHT22
#include <dht.h>
dht DHT;
#define DHT22_PIN 7

// BMP085
#include "Wire.h"
#include <Adafruit_BMP085.h>
Adafruit_BMP085 bmp;

// LDR
// "Power" connected to D8
int ldrPwrPin = 8;
int ldrPin = A0;
int ldrValue = 0;

// TLS257 Optical sensor for electricity meter led
int tlsPwr = 5;
int tlsPin = 6;
unsigned long start, finished;
long pulsetime;
double elapsed, elapsedS;

typedef struct {
  	  int temp;		                                      
	  int battery;	
          int humidity;	 
          int pressure;    
          int ldrValue;      
          int pulseLen;
} Payload;
Payload emontx;

void setup() {
  /*
  Serial.begin(9600);
  Serial.println("emonTX Low-power temperature/humidity/air pressure"); 
  Serial.println("OpenEnergyMonitor.org");
  Serial.print("Node: "); 
  Serial.print(nodeID); 
  Serial.print(" Freq: "); 
  if (RF_freq == RF12_433MHZ) Serial.print("433Mhz");
  if (RF_freq == RF12_868MHZ) Serial.print("868Mhz");
  if (RF_freq == RF12_915MHZ) Serial.print("915Mhz"); 
  Serial.print(" Network: "); 
  Serial.println(networkGroup);
  */
  
  // Led on
  /*                                                    // DS18B20 power control pin - see jumper setup instructions above
  digitalWrite(9,HIGH);                                                 // turn on DS18B20
  delay(10);
  */
  
  // Blink led to confirm boot
  pinMode(9,OUTPUT);
  digitalWrite(9,HIGH);                                                 // turn on DS18B20
  delay(500);
  digitalWrite(9,LOW);
  
  // DS18B20
  sensors.begin();
  if (!sensors.getAddress(sensor, 0)) Serial.println("Unable to find DS18B20 Temperature Sensor");
  
  // BMP85 Temp & pressure
  bmp.begin(); 
  
  // TLS257
  //pinMode(tlsPin,INPUT);                                                   
  pinMode(tlsPwr,OUTPUT);                                                   
  //digitalWrite(tlsPwr,HIGH);                                                 // turn on DS18B20
  //digitalWrite(6,HIGH);                                                 // turn on DS18B20

  Serial.println("Setup done");

  rf12_initialize(nodeID, RF_freq, networkGroup);                          // initialize RFM12B
  rf12_control(0xC040);                                                 // set low-battery level to 2.2V i.s.o. 3.1V
  delay(10);
  //Serial.println("RF12 Sleep");

  rf12_sleep(RF12_SLEEP);
}

void loop()
{ 
  // Air pressure
  float pressure = bmp.readPressure();
  
  /*
  Serial.print("Pressure = ");
  Serial.print(pressure/100);
  Serial.println(" Pa");
  */
  
  //pinMode(tlsPwr,OUTPUT);                                                   
  digitalWrite(tlsPwr, HIGH);
  delay(20);
  
  /*
  int tlsVal = digitalRead(tlsPin);
  //digitalWrite(tlsPwr, LOW);
  Serial.print("tls = ");
  Serial.print(tlsVal);
  Serial.println("");
  */
  
  // Wait for pulse
  // Wait for a high pulse, let the pulse end, then move on
  long int startmillis, curmillis;
  
  startmillis=millis();
  while(digitalRead(tlsPin) == 0){
    curmillis=millis();
    // If nothing ever happens
    if (curmillis-startmillis>10000) {
//      Serial.println("No TLS-pulse, break");
      break;
    }    
  }
  
  startmillis=millis();
  while(digitalRead(tlsPin) == 1){
    curmillis=millis();
    // If nothing ever happens
    if (curmillis-startmillis>10000) {
//      Serial.println("No TLS-pulse, break");
      break;
    }    
  }
  
  // Count time between pulses
  
  Serial.println("Start...");
  start=millis();
  startmillis=millis();
  while(digitalRead(tlsPin) == 0){
    curmillis=millis();
    // If nothing ever happens
    if (curmillis-startmillis>10000) {
  //    Serial.println("No TLS-pulse, break");
      break;
    }      
  }
  finished=millis();
  //Serial.println("Finished");
  elapsed=finished-start;
  /*
  Serial.print(elapsed);
  Serial.println(" milliseconds elapsed");
  Serial.println();
  */
  
  /*
   Power (kW) = 3600 (secs in 1hr) divided by (the seconds between 
   flashes * number of Imp/kWh printed on meter).
   3600 / (time * 1000)
   http://www.reuk.co.uk/Flashing-LED-on-Electricity-Meter.htm
  */
  elapsedS=elapsed/1000;
  
  /*
  Serial.print(elapsedS,3);  // Print with three decimals
  Serial.println(" seconds between pulses");  
  */
  
  float curPwr = 3600/(elapsedS*1000);
  int pwrK=curPwr*1000;
  
  /*
  Serial.print("Current: ");
  Serial.print(curPwr);
  Serial.println(" kW.");
  */

  //Serial.println(pulsetime);

  // Light level, read LDR
  digitalWrite(ldrPwrPin, HIGH);
  delay(20);
  ldrValue = analogRead(ldrPin);
  digitalWrite(ldrPwrPin, LOW);
  
  /*
  Serial.print("ldrValue: ");  
  Serial.print(ldrValue);
  Serial.println();
  */
  
  // DS1820
  //digitalWrite(7,HIGH); delay(2);  // turn on DS18B20 and wait for it to come online
  sensors.requestTemperatures();                                        // Send the command to get temperatures
  float temp=(sensors.getTempCByIndex(0));
  //digitalWrite(7,LOW); 
  
  /*
  Serial.print("DS1820: ");
  Serial.print(temp*100);
  Serial.println();
  */
  
  // Humidity DHT-22  
  //Serial.print("DHT22: ");
  int chk = DHT.read22(DHT22_PIN);
  
  /*
  Serial.print(DHT.humidity, 1);
  Serial.print(" : ");
  Serial.print(DHT.temperature, 1);
  Serial.println();
  */
  
  // Values to send
  emontx.temp=(temp*100);
  emontx.battery=readVcc();
  emontx.humidity=DHT.humidity;
  emontx.pressure=pressure/100;
  emontx.ldrValue=ldrValue;
  emontx.pulseLen=pwrK;
  
  rf12_sleep(RF12_WAKEUP);
  // if ready to send + exit loop if it gets stuck as it seems too
  int i = 0; while (!rf12_canSend() && i<10) {rf12_recvDone(); i++;}
  rf12_sendStart(0, &emontx, sizeof emontx);
  // set the sync mode to 2 if the fuses are still the Arduino default
  // mode 3 (full powerdown) can only be used with 258 CK startup fuses
  rf12_sendWait(2);
  rf12_sleep(RF12_SLEEP);  
  
  // http://openenergymonitor.org/emon/node/2599
  // time_between_readings = 60000 -> i value in minutes
  // I.e. sleep for 5 minutes between readings.
  for (byte i = 0; i < 5; i++)
    Sleepy::loseSomeTime(time_between_readings);
}

// Measure Vcc  by measuring the internal Vref
// http://provideyourown.com/2012/secret-arduino-voltmeter-measure-battery-voltage/
long readVcc() {
  long result;
  // Set Mux 1-3. 
  ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  delay(2);
  ADCSRA |= _BV(ADSC);
  while (bit_is_set(ADCSRA,ADSC));
  result = ADCL;
  result |= ADCH<<8;
  result = 1126400L / result;
  return result;
}

