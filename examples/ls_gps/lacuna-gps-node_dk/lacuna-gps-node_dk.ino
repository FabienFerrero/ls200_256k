/*
 *  _                                   ____                       
 * | |    __ _  ___ _   _ _ __   __ _  / ___| _ __   __ _  ___ ___ 
 * | |   / _` |/ __| | | | '_ \ / _` | \___ \| '_ \ / _` |/ __/ _ \
 * | |__| (_| | (__| |_| | | | | (_| |  ___) | |_) | (_| | (_|  __/
 * |_____\__,_|\___|\__,_|_| |_|\__,_| |____/| .__/ \__,_|\___\___|
 *                                           |_|                   
 * Copyright (C) 2021 Lacuna Space Ltd.
 *
 * Description: Lacuna GPS node sketch.
 * 
 * Licensae: Revised BSD License, see LICENSE-LACUNA.TXT file included in the project
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 */

/*
 * You can install these libraries via the Arduino library manager:
 *  
 *  http://librarymanager/All#Sodaq_LSM303AGR
 *  http://librarymanager/All#SparkFun_BME280
 *  http://librarymanager/All#SparkFun_Ublox
 *  http://librarymanager/All#SparkFun_ATECCX08a
 * 
 */

#ifndef REGION
#define REGION R_EU868
#endif

#define UBLOX
//#define QUECTEL


#include <LibLacuna.h>
#include "lpp_messages.h"
#include <SPI.h>
#include <time.h>
#include <EEPROM.h>
#include <RTC.h>
#include <Wire.h>
//#include <SparkFunBME280.h>   // http://librarymanager/All#SparkFun_BME280
//#include <Sodaq_LSM303AGR.h>  // http://librarymanager/All#Sodaq_LSM303AGR
#include <SparkFun_Ublox_Arduino_Library.h>   // http://librarymanager/All#SparkFun_Ublox
#include <MicroNMEA.h>   // http://librarymanager/All#MicroNMEA

// Interval between satellite transmissions and number of messages
#define INTERVAL 10
#define NUMTX 50

// Sleep time between periodic GPS checks (and TTN updates)
#define SLEEPMINUTES 30

// Minimum elevation for satellite
#define MINELEVATION 15

// max. 250 seconds for GPS fix
#define FIXTIME 200

// Dont sync GPS within this interval before a scheduled transmission
#define GUARDTIME (FIXTIME+30)

// Only use for testing
//#define FAKEPASS
//#define DEBUGSLEEP

// Define this to get the settings for the four test nodes, or leave it undefined to fill in your own settings in the #else block below
#define LACUNA_TEST 1

#if LACUNA_TEST == 1
// Do not modify
static byte networkKey[] = {     0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
static byte appKey[] = {   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
static byte deviceAddress[] = { 0x00, 0x00, 0x00, 0x00 };
#elif LACUNA_TEST == 2
// Do not modify
static byte networkKey[] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
static byte appKey[] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
static byte deviceAddress[] = { 0x00, 0x00, 0x00, 0x00 };

#else
///////////////////////////////////
// Fill in your own settings here
///////////////////////////////////
// Keys and device address are MSB
static byte networkKey[] = {
   // Replace with your network key
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};
static byte appKey[] = {
   // Replace with your application key
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};
// Replace with your device address
static byte deviceAddress[] = { 0x00, 0x00, 0x00, 0x00 };
#endif // LACUNA_TEST

// Multicast session
static byte mc_networkKey[] = { 0xCA, 0x86, 0x46, 0xF8, 0x17, 0xB9, 0xC6, 0x0E, 0xF8, 0xBD, 0xEB, 0xD5, 0xB9, 0xE1, 0x55, 0xE4 };
static byte mc_appKey[] = { 0xEA, 0xC8, 0xA3, 0x8A, 0xC9, 0x44, 0x12, 0x5D, 0xD0, 0xFF, 0x52, 0xAD, 0x85, 0xA0, 0xAD, 0xA4 };
static byte mc_deviceAddress[] = { 0x26, 0x01, 0x13, 0xEF };

static lsLoraWANParams loraWANParams;
static lsLoraSatTxParams SattxParams;
static lsLoraTxParams txParams;
static lsLoraTxParams SatrxParams;

static char payload[255];
static int payloadLength;

float gnss_lat;
float gnss_lon;
float gnss_alt;

uint32_t FixEpoch;
uint8_t  TimeToFix;

uint32_t last_epoch;
uint32_t alarmepoch;

uint32_t rxtxepoch = 0;
uint8_t  txinterval = 0;
uint16_t rx_duration;
bool     rxtx;
uint8_t  satellite_id;

int32_t sleepseconds;
uint8_t wakeupreason = 0;

// interval between GPS checks
uint16_t sleeptimesec = 60 * SLEEPMINUTES; // in seconds

// LPP
uint8_t  store[4096];
uint8_t  lpp_age = 0;
uint32_t oldest_record;
uint32_t newest_record;

// Env variable
int16_t temp = 0;
uint16_t pressure = 0;


// BME280 sensor
//BME280 bme280;

// Accelerometer
//Sodaq_LSM303AGR lsm303(LSM303_WIRE);

// Pressure sensor
// HP203B I2C address is 0x76(118)
#define HP203B_Addr 0x76

// GPS
#ifdef UBLOX
SFE_UBLOX_GPS myGPS;

#else  // Quectel
uint8_t GPS_Address = 0x10;
//MicroNMEA library structures
char nmeaBuffer[100];
MicroNMEA nmea(nmeaBuffer, sizeof(nmeaBuffer));

#endif

static char const *sattelite_name(uint8_t id) {
    switch(id) {
        case 1: return "LS2D";
        case 2: return "LS3";
        case 3: return "LS2C";
        case 4: return "LS2B";
        case 5: return "LS2E";
        case 6: return "LS2X";
        default: return "Unknown";
    }
}

uint32_t parse_payload_32(char *payload, uint8_t offset) {
  return ((payload[offset+3]) + (payload[offset+2] << 8)   + (payload[offset+1]<<16)  + (payload[offset]<<24));
}

uint16_t parse_payload_16(char *payload, uint8_t offset) {
  return ((payload[offset+1])  + (payload[offset]<<8));
}

void setup() {
  
  Serial.begin(115200);

  while (!Serial && millis() < 3000);
  
  Serial.println("Initializing");

  Serial.print("Configured Region: ");
#if REGION == R_EU868
  Serial.println("Europe 862-870 Mhz");
#elif REGION  == R_US915
  Serial.println("US 902-928 Mhz");
#elif REGION == R_AS923
  Serial.println("Asia 923 Mhz");
#elif REGION == R_IN865
  Serial.println("India 865-867 Mhz");
#else 
  Serial.println("Undefined");  
#endif

  analogReadResolution(12);

  Wire.begin();
 
 // LSM303_WIRE.begin();

  pinMode(LS_GPS_ENABLE, OUTPUT);
  digitalWrite(LS_GPS_ENABLE, HIGH);
  pinMode(LS_GPS_V_BCKP, OUTPUT);
  digitalWrite(LS_GPS_V_BCKP, HIGH);

  digitalWrite(LS_VERSION_ENABLE, LOW);

  pinMode(LS_INT_MAG, OUTPUT); // make INT_MAG LOW for low-power 
  digitalWrite(LS_INT_MAG, LOW);

  // LSM303
 // lsm303.enableAccelerometer();

  // BME 280
  //bme280.setI2CAddress(0x76);
 // if (bme280.beginI2C(BME280_WIRE) == false)
 // {
 //   Serial.println("The sensor did not respond. Please check wiring.");
 // }

  delay(70);

  // GPS
#ifdef  UBLOX
  
  if (myGPS.begin() == false) //Connect to the Ublox module using Wire port
  {
      Serial.println(F("Ublox GPS not detected at default I2C address. Please check wiring."));
      //while (1);
  }

  myGPS.setI2COutput(COM_TYPE_UBX); //Set the I2C port to output UBX only (turn off NMEA noise)
  myGPS.saveConfiguration();        //Save the current settings to flash and BBR

  #endif

  
  
  pinMode(LS_LED_BLUE, OUTPUT);
  digitalWrite(LS_LED_BLUE, HIGH);
  delay(200);
  digitalWrite(LS_LED_BLUE, LOW);

  // SX1262 configuration for lacuna LS200 board
  lsSX126xConfig cfg;
  lsCreateDefaultSX126xConfig(&cfg, BOARD_VERSION);

  // Initialize SX1262
  Serial.print("SX1262: ");
  int result = lsInitSX126x(&cfg, REGION);
  Serial.println(lsErrorToString(result));

  // LoRaWAN session parameters
  lsCreateDefaultLoraWANParams(&loraWANParams, networkKey, appKey, deviceAddress);
  loraWANParams.txPort = 1;
  loraWANParams.rxEnable = true;

   // Configure multicast session
  memcpy(loraWANParams.mc_networkKey, mc_networkKey, sizeof(loraWANParams.mc_networkKey));
  memcpy(loraWANParams.mc_applicationKey, mc_appKey, sizeof(loraWANParams.mc_applicationKey));
  memcpy(loraWANParams.mc_deviceAddress, mc_deviceAddress, sizeof(loraWANParams.mc_deviceAddress));
 
  // transmission parameters for Lacuna satellites
  lsCreateDefaultLoraSatTxParams(&SattxParams, REGION);

  // transmission parameters for terrestrial LoRa
  lsCreateDefaultLoraTxParams(&txParams, REGION);

  // modify LoRa parameters
  txParams.power = 14;
  txParams.spreadingFactor = lsLoraSpreadingFactor_8;

  // reception parameters for Lacuna downlink 
  lsCreateDefaultLoraTxParams(&SatrxParams, REGION);

  // modify reception parameters
  SatrxParams.spreadingFactor = lsLoraSpreadingFactor_10;
  SatrxParams.bandwidth = lsLoraBandwidth_500_khz;
  SatrxParams.frequency = 862800000;
  SatrxParams.preambleLength = 488;
  SatrxParams.ldro= true;
  SatrxParams.rxTimeout = 10000; // in ms

  // LPP init
  int32_t lpp_result = lpp_setup(store, sizeof store);
  if(lpp_result) {
      Serial.print("Error setting up LPP: ");
      Serial.println(lpp_error_to_string(lpp_result));
  } else {
      Serial.println("Success setting up LPP");
  }

  // Read messages
  for(size_t l=0; l<LPP_MESSAGE_COUNT; l++) {
     Serial.print("Read message with length ");
     Serial.print(lpp_records[l].size);
     int32_t lpp_result = lpp_add_records(store, lpp_records[l].data, lpp_records[l].size);
     if(lpp_result) {
        Serial.print(": Error ");
        Serial.println(lpp_error_to_string(lpp_result));
      } else {
        Serial.println(": Success");
      }
  }      
  
  lpp_result = lpp_get_records_age(store, &oldest_record, &newest_record);
  if(lpp_result) {
      Serial.print("Error: ");
      Serial.println(lpp_error_to_string(result));
  } else {
     Serial.print("Newest LPP record: ");
     Serial.println(newest_record);
     Serial.print("Oldest LPP record: ");
     Serial.println(oldest_record);
  } 
  Serial.print("Here's a random number for you: ");
  Serial.println(random(10000),DEC);
}

void loop() {

  digitalWrite(LS_LED_BLUE, HIGH);
  delay(50);
  digitalWrite(LS_LED_BLUE, LOW);
  delay(100);
  digitalWrite(LS_LED_BLUE, HIGH);
  delay(50);
  digitalWrite(LS_LED_BLUE, LOW);
  
  last_epoch = RTC.getEpoch();

  if (wakeupreason == 0) {
    // periodic wakeup
    Serial.println("Periodic wakeup");
    Serial.println("Enable GPS");
    digitalWrite(LS_GPS_ENABLE, HIGH);
    periodic_check();
    
  } else if (wakeupreason == 1 && rxtx == lpp_contact_type_rx) {
    // TX
    Serial.println("device TX wakeup");
    satellite_uplink();
  
  } else if (wakeupreason == 1 && rxtx == lpp_contact_type_tx) {
    // RX
    Serial.println("device RX wakeup");
    process_downlink();
    
  } else {
    // this should not happen
    Serial.println("Unknown wakeup reason");
  }

  // Calculate sleep time
  
  if ( (rxtxepoch != 0) && (last_epoch + sleeptimesec) > (rxtxepoch - GUARDTIME) && (rxtxepoch > RTC.getEpoch())) {
    
    Serial.println("Sleep till RX/TX epoch");
    alarmepoch = rxtxepoch;
    wakeupreason = 1;
    
  } else {

    Serial.println("Sleep till periodic update");
    alarmepoch = last_epoch + sleeptimesec;
    wakeupreason = 0;

  }

  sleepseconds = alarmepoch - RTC.getEpoch();

  Serial.println("-- SLEEP");
  Serial.print("  Sleep Time: ");
  Serial.println(sleepseconds);

#ifdef DEBUGSLEEP

  if (sleepseconds > 0 ) {
    delay(sleepseconds * 1000);
  }

#else

  if (alarmepoch > RTC.getEpoch()) {

    time_t t;
    struct tm tm;

    t = (time_t)alarmepoch;
    gmtime_r(&t, &tm);
    
    RTC.setAlarmTime(tm.tm_hour, tm.tm_min, tm.tm_sec);
    RTC.setAlarmDay(tm.tm_mday);
    
    RTC.enableAlarm(RTC.MATCH_HHMMSS);
    RTC.attachInterrupt(alarmMatch);

    // put LS200 board in deepsleep
    LS200_sleep();

    delay(100);
  }

#endif

  Serial.println("-- WAKE");
  
}

void alarmMatch() { }

void LS200_sleep()  
{
  digitalWrite(LS_GPS_ENABLE, LOW);
  digitalWrite(LS_VERSION_ENABLE, LOW);
  digitalWrite(LS_GPS_V_BCKP, HIGH);
  
  //bme280.setMode(MODE_SLEEP); 
 // lsm303.disableMagnetometer();
 // lsm303.disableAccelerometer();

  SPI.end();
  delay(10);
  STM32.stop();

  // Sleep...
  
  SPI.begin();
//  digitalWrite(LS_GPS_ENABLE, HIGH);
  digitalWrite(LS_VERSION_ENABLE, LOW);
  digitalWrite(LS_GPS_V_BCKP, HIGH);
 // bme280.setMode(MODE_FORCED);
  //lsm303.enableAccelerometer();
}

void periodic_check() {

#ifdef UBLOX
  bool fix = updategps();

#else // L96

bool fix = updategps_L96();

#endif
    
  if (fix) {
      Serial.println("New position fix");
      // predict satellite passes
      uint32_t start;
      uint32_t duration;
      uint16_t maxelevation = predictpass(gnss_lat, gnss_lon, gnss_alt, &start, &duration, &rxtx, &satellite_id);
  
      if (maxelevation != 0) {

        Serial.print("Next pass set, ");
        Serial.print("max. elevation: ");
        Serial.print(maxelevation);
        Serial.print(", start at ");
        Serial.print(start);
        Serial.print(" duration: ");
        Serial.println(duration);

        if (rxtx == lpp_contact_type_rx) {
          // TX
          txinterval = INTERVAL + (lpp_age * 1);
          rxtxepoch = start + (duration/2) - (txinterval*(NUMTX-1))/2;
          rx_duration = 0;

          Serial.print("TX Epoch set to: ");
          Serial.print(rxtxepoch);
          Serial.print(", interval: ");
          Serial.print(txinterval);
          Serial.println(" seconds");
        } else {
          // RX
          txinterval = 0;
          rxtxepoch = start - (lpp_age * 1);
          rx_duration = duration + (lpp_age * 2);

          Serial.print("RX Epoch set to: ");
          Serial.print(rxtxepoch);
          Serial.print(", duration: ");
          Serial.print(rx_duration);
          Serial.println(" seconds");
        }
      } else {
        Serial.println("No pass found?");
      }
      
  } 
  
  generateMessage(false);
 
  Serial.println("Sending LoRa message");  
  int lora_result = lsSendLoraWAN(&loraWANParams, &txParams, (byte *)payload, payloadLength);
  Serial.print("Result: ");
  Serial.println(lsErrorToString(lora_result));
}

void satellite_uplink() {
  
    uint8_t seq = 1;

    while (seq <= NUMTX) {

      Serial.println("Transmit");

      digitalWrite(LS_LED_BLUE, HIGH);
      delay(50);
      digitalWrite(LS_LED_BLUE, LOW);
      delay(100);
      digitalWrite(LS_LED_BLUE, HIGH);
      delay(50);
      digitalWrite(LS_LED_BLUE, LOW);

      unsigned long starttx = millis();

      generateMessage(satellite_id);
      
      Serial.println("Sending LoraSat message");
      int sat_result = lsSendLoraSatWAN(&loraWANParams, &SattxParams, (byte *)payload, payloadLength);
      Serial.print("Result: ");
      Serial.println(lsErrorToString(sat_result));

      unsigned long endtx = millis();

      int16_t waittime = txinterval*1000 - (endtx - starttx);
   
      Serial.print("Wait ");
      Serial.print(waittime);
      Serial.println(" milliseconds");

      // don't wait after the last transmission
      if (waittime>0 && seq < NUMTX) { 
        // TODO: MAKE THIS LOW-POWER!
        delay( waittime );
      }
      seq++;
    } 
}

void process_downlink() {

  int snr;
  int rssi;
  int signalrssi;
  int frequency_error;
  int sendRxData = 0;

  while ( RTC.getEpoch() < ( rxtxepoch + rx_duration ) ) { 
    Serial.println("Waiting for LoRa message"); 
    int bytes = lsReceiveLoraSat(&loraWANParams, &SatrxParams, (byte *)payload);
    if (bytes > 0) {    
      Serial.println("LoRa message received");
    
      Serial.print(" . Timestamp: ");
      Serial.println(millis());
      Serial.print(" . Length: ");
      Serial.println(bytes);
      Serial.print(" . SNR: ");
      Serial.println(SatrxParams.snr);
      Serial.print(" . RSSI: ");
      Serial.println(SatrxParams.rssi);
      Serial.print(" . SignalRSSI: ");
      Serial.println(SatrxParams.signalrssi);
      Serial.print(" . Frequency Error: ");
      Serial.println(SatrxParams.frequency_error);

      if (loraWANParams.rxpayloadLength) {
        Serial.println("** LoRaWAN packet!");
        Serial.print(  "   Port: ");
        Serial.println(loraWANParams.rxPort);
        Serial.print(  "   Multicast: ");
        Serial.println(loraWANParams.rxMc ? "yes" : "no");
        Serial.print("   Payload: ");
        for (char n = 0; n < bytes; n++)
            {
              Serial.print (payload[n],HEX);
              Serial.write (" ");      
            }
        Serial.println();
        // Do something with data?
      }
  
      uint32_t signature = parse_payload_32(payload, 0);
      if ((signature & 0xFFFFFF00) == 0xe14c5300) { 
        Serial.println("** Wake-up packet! **");
        uint32_t sat_time = parse_payload_32(payload, 11);
        Serial.print("  UTC time: ");
        Serial.println(sat_time, DEC);
        // Set RTC?
  //      RTC.setEpoch(sat_time);
        time_t t = (time_t)sat_time;
        struct tm tm;
        gmtime_r(&t, &tm);
        Serial.print("  Time: ");
        Serial.print(tm.tm_hour,DEC);
        Serial.print(":");
        Serial.print(tm.tm_min,DEC);
        Serial.print(":");
        Serial.println(tm.tm_sec,DEC);
        uint16_t offset = parse_payload_16(payload, 15);
        Serial.print("  Offset: ");
        Serial.print(offset, DEC);
        Serial.println(" ms");
        Serial.print("  Data packets: ");
        Serial.println( (uint8_t)(payload[29]), DEC);
        uint8_t packet_num = (uint8_t)(payload[29]);
        uint32_t data_freq = parse_payload_32(payload, 17);
        Serial.print("  Freq: ");
        Serial.println(data_freq, DEC);
      }
      if ( (signature & 0xFF000000) == 0xE4000000 ) { 
        Serial.println("** LPP packet **");

        snr = SatrxParams.snr;
        rssi = SatrxParams.rssi;
        signalrssi = SatrxParams.signalrssi;
        frequency_error = SatrxParams.frequency_error;
        sendRxData++;
        
        int32_t lpp_result = lpp_add_records(store, (uint8_t*)payload+1, bytes);
  
        if(lpp_result) {
           Serial.print(" . LPP ADD: ");
           Serial.println(lpp_error_to_string(lpp_result));
         } else {
           Serial.println("LPP ADD success");
         }
       }
    } else {
      Serial.print("No packet: ");
      if (bytes == LS_ERR_TIMEOUT) { 
        Serial.println("timeout"); 
      } else if (bytes == LS_ERR_CRC) {
        Serial.println("crc error"); 
      } else {
        Serial.println("unknown error"); 
      }
    }
    if (sendRxData > 6) { // Wait for 5 valid LPP packet to send a notification 

        sendRxData = 0; // reset counter
        // Set payload
        payload[0] = 2; // it is LS3 sat
        payload[1] = snr;
        payload[2] = rssi;
        payload[3] = signalrssi;
        payload[4] = frequency_error;
        payloadLength = 5;  
    
        Serial.println("Sending LoRa message"); 
        int lora_result  = lsSendLoraWAN(&loraWANParams, &txParams, (byte *)payload, payloadLength);

        digitalWrite(LS_LED_BLUE, HIGH);
        delay(50);
        digitalWrite(LS_LED_BLUE, LOW);
        delay(100);
        digitalWrite(LS_LED_BLUE, HIGH);
        delay(50);
        digitalWrite(LS_LED_BLUE, LOW);

    }
  }
}

void generateMessage(uint8_t satellite) {

HP203B(); // Update temperature and pressure

  //int16_t temp = 0;// (int16_t)(100*bme280.readTempC());
  uint8_t humidity = 0;// (int8_t)(bme280.readFloatHumidity());
  //uint16_t pressure = 0; // (uint16_t)(bme280.readFloatPressure()/10);

  

  uint16_t voltage_adc = (uint16_t)analogRead(LS_BATVOLT_PIN);
  uint16_t voltage = (uint16_t)((LS_ADC_AREF / 4.096) * (LS_BATVOLT_R1 + LS_BATVOLT_R2) / LS_BATVOLT_R2 * (float)voltage_adc);

  int8_t acc_x =0; // (int8_t)(100*lsm303.getX());
  int8_t acc_y = 0;//(int8_t)(100*lsm303.getY());
  int8_t acc_z =0;// (int8_t)(100*lsm303.getZ());

  // STM32 vbat voltage
  // uint16_t voltage = (uint16_t)(1000*STM32.getVBAT());

  Serial.println("Read sensors");
  Serial.print("  Voltage: ");
  Serial.println(voltage/1000.0f);
  Serial.print("  Temp: ");
  Serial.println(temp/100.0f);
  Serial.print("  Humidity: ");
  Serial.println(humidity);
  Serial.print("  Pressure: ");
  Serial.println(pressure);

  Serial.print("  x = ");
  Serial.print(acc_x);
  Serial.print(", y = ");
  Serial.print(acc_y);
  Serial.print(", z = ");
  Serial.println(acc_z);
 
  uint32_t LatitudeBinary = ((gnss_lat + 90) / 180) * 16777215;
  uint32_t LongitudeBinary = ((gnss_lon + 180) / 360) * 16777215;
  int16_t  altitudeGps = gnss_alt; 
  
  uint32_t fixage = (RTC.getEpoch() - FixEpoch)/60; // in minutes

  payload[0] = satellite;
  payload[1] = (temp >> 8) & 0xff;
  payload[2] = temp & 0xff;
  payload[3] = (voltage_adc >> 8) & 0xff;
  payload[4] = voltage_adc & 0xff;
  payload[5] = humidity; 
  payload[6] = (pressure >> 8) & 0xff;
  payload[7] = pressure & 0xff;
  payload[8] = ( LatitudeBinary >> 16 ) & 0xFF;
  payload[9] = ( LatitudeBinary >> 8 ) & 0xFF;
  payload[10] = LatitudeBinary & 0xFF;
  payload[11] = ( LongitudeBinary >> 16 ) & 0xFF;
  payload[12] = ( LongitudeBinary >> 8 ) & 0xFF;
  payload[13] = LongitudeBinary & 0xFF;
  payload[14] = lpp_age;
  payload[15] = TimeToFix;
  payloadLength = 16;
}


#ifdef UBLOX

byte updategps() {

   digitalWrite(LS_GPS_ENABLE, HIGH);
   delay(200);

   long startTime = millis(); 
    
   while (millis() - startTime < (FIXTIME*1000))
   {

    delay(10);
      digitalWrite(LS_LED_BLUE, HIGH);
      delay(50);
      digitalWrite(LS_LED_BLUE, LOW);
      delay(100);
      digitalWrite(LS_LED_BLUE, HIGH);
      delay(50);
      digitalWrite(LS_LED_BLUE, LOW);
      

      long latitude = myGPS.getLatitude();
      long longitude = myGPS.getLongitude();    
      long altitude = myGPS.getAltitudeMSL();      
      byte SIV = myGPS.getSIV();
 
      Serial.println("Checking GPS");

      Serial.print("  GPS position: ");
      Serial.print(latitude/1.0e7,4);
      Serial.print(", ");
      Serial.print(longitude/1.0e7,4);
      Serial.print(" alt: ");
      Serial.print(altitude/1.0e6,2);
      Serial.print(" (");
      Serial.print(SIV);
      Serial.println(" satellites)");
 
      byte fixType = myGPS.getFixType();
      Serial.print(F("  GPS Fix: "));
      if(fixType == 0) Serial.print(F("No fix"));
      else if(fixType == 1) Serial.print(F("Dead reckoning"));
      else if(fixType == 2) Serial.print(F("2D"));
      else if(fixType == 3) Serial.print(F("3D"));
      else if(fixType == 4) Serial.print(F("GNSS+Dead reckoning"));
      else if(fixType == 5) Serial.print(F("Time only"));
      
      uint16_t new_pdop = (uint16_t)myGPS.getPDOP();
      Serial.print(", PDOP: ");
      Serial.println(new_pdop);
      
      Serial.print("  GPS time: ");
      Serial.print(myGPS.getYear());
      Serial.print("-");
      Serial.print(myGPS.getMonth());
      Serial.print("-");
      Serial.print(myGPS.getDay());
      Serial.print(" ");
      Serial.print(myGPS.getHour());
      Serial.print(":");
      Serial.print(myGPS.getMinute());
      Serial.print(":");
      Serial.println(myGPS.getSecond());
      
      uint32_t unixt = unixTimestamp(myGPS.getYear(),myGPS.getMonth(),myGPS.getDay(),myGPS.getHour(),myGPS.getMinute(),myGPS.getSecond());
      
      Serial.print("  Unix time GPS: ");
      Serial.println(unixt);
      
      if (fixType == 2 || fixType == 3 ) {
      // wait for 3D fix
      //if ( fixType == 3 ) {
        delay(1000);
        Serial.println("Updating time");
        unixt = unixTimestamp(myGPS.getYear(),myGPS.getMonth(),myGPS.getDay(),myGPS.getHour(),myGPS.getMinute(),myGPS.getSecond());

        
        Serial.print("  Unix time GPS: ");
        Serial.println(unixt);
        Serial.println("Updating position");
        gnss_lat = latitude/1e7;
        gnss_lon = longitude/1e7;
        if (fixType == 3) {
          gnss_alt = altitude/1e3;
        } else {
          gnss_alt = 0;
        }
        FixEpoch=unixt;

        int lpp_result = lpp_get_records_age(store, &oldest_record, &newest_record);
        if(lpp_result) {
            Serial.print("Error: ");
            Serial.println(lpp_error_to_string(lpp_result));
        } else {
           Serial.print("Newest LPP record: ");
           Serial.println(newest_record);
           Serial.print("Oldest LPP record: ");
           Serial.println(oldest_record);
        } 

        lpp_age = ((unixt-oldest_record)/86400);

        Serial.print("  LPP Epoch age (days): ");
        Serial.println( lpp_age );
      
        // if we have a fix, time is valid
        int32_t diff = unixt - RTC.getEpoch();
        Serial.print("  Correcting RTC by ");
        Serial.print(diff);
        Serial.println(" seconds.");
        // compensate our wakeup time
        last_epoch = last_epoch + diff;
        RTC.setEpoch(unixt);
        TimeToFix = (millis() - startTime)/1000;
        
        return(1);
    }
  }
  return(0);  
}


#else // L96

byte updategps_L96() {

   digitalWrite(LS_GPS_ENABLE, HIGH);
   delay(200);
   nmea.clear(); // reset NMEA

   long startTime = millis(); 
    
  
while (!nmea.isValid()  && (millis() - startTime < (FIXTIME*1000)))
   {

    delay(10);
      digitalWrite(LS_LED_BLUE, HIGH);
      delay(50);
      digitalWrite(LS_LED_BLUE, LOW);
      delay(100);
      digitalWrite(LS_LED_BLUE, HIGH);
      delay(50);
      digitalWrite(LS_LED_BLUE, LOW);

      Serial.print("GPS time: ");
      Serial.print(nmea.getYear());
      Serial.print("-");
      Serial.print(nmea.getMonth());
      Serial.print("-");
      Serial.print(nmea.getDay());
      Serial.print(" ");
      Serial.print(nmea.getHour());
      Serial.print(":");
      Serial.print(nmea.getMinute());
      Serial.print(":");
      Serial.println(nmea.getSecond());
      
      Serial.print(nmea.getNumSatellites());
      Serial.println(" satellites");
      
      
       int GPS_Address = 0x10;
      Wire.requestFrom(GPS_Address, 255);
      
      // Read GPS on I2C
      while (Wire.available()) {
         
      char c = Wire.read();
       nmea.process(c);
       //Serial.print(c);
      }
    delay(1000);
      
   }

      long latitude = nmea.getLatitude();
      long longitude = nmea.getLongitude();    
      long altitude;
      bool is3D = nmea.getAltitude(altitude);      
      byte SIV = nmea.getNumSatellites();
      byte fixType =nmea.isValid();
      // Print final position 
      TimeToFix = (millis() - startTime)/1000;
      Serial.print("Valid fix in ");
      Serial.print(TimeToFix);
      Serial.println(" seconds");

      Serial.print("Nav. system: ");
      if (nmea.getNavSystem()== 'P') {Serial.println("Navigation results based only on GPS satellites.");}
      if (nmea.getNavSystem()== 'L') {Serial.println("Navigation results based only on GLONASS satellites.");}
      if (nmea.getNavSystem()== 'A') {Serial.println("Navigation results based only on GAlileo satellites.");}
      if (nmea.getNavSystem()== 'N') {Serial.println("GNSS, navigation results from multiple satellite constellations.");}
      Serial.print("GPS time: ");
      Serial.print(nmea.getYear());
      Serial.print("-");
      Serial.print(nmea.getMonth());
      Serial.print("-");
      Serial.print(nmea.getDay());
      Serial.print(" ");
      Serial.print(nmea.getHour());
      Serial.print(":");
      Serial.print(nmea.getMinute());
      Serial.print(":");
      Serial.println(nmea.getSecond());

      Serial.println("Checking GPS");

      Serial.print("  GPS position: ");
      Serial.print(latitude/1.0e6,4);
      Serial.print(", ");
      Serial.print(longitude/1.0e6,4);
      Serial.print(" alt: ");
      Serial.print(altitude/1.0e3,2);
      Serial.print(" : ");
      
      Serial.print(nmea.getNumSatellites());
      Serial.println(" satellites");
      
      
      if(fixType == 1) {        
      fixType = 2;
      uint32_t unixt = unixTimestamp(nmea.getYear(),nmea.getMonth(),nmea.getDay(),nmea.getHour(),nmea.getMinute(),nmea.getSecond());
           
      if (is3D) {fixType =3;}

        
        Serial.print("  Unix time GPS: ");
        Serial.println(unixt);
        Serial.println("Updating position");
        gnss_lat = latitude/1e6;
        gnss_lon = longitude/1e6;
        if (fixType == 3) {
          gnss_alt = altitude/1e3;
        } else {
          gnss_alt = 0;
        }
        FixEpoch=unixt;

        int lpp_result = lpp_get_records_age(store, &oldest_record, &newest_record);
        if(lpp_result) {
            Serial.print("Error: ");
            Serial.println(lpp_error_to_string(lpp_result));
        } else {
           Serial.print("Newest LPP record: ");
           Serial.println(newest_record);
           Serial.print("Oldest LPP record: ");
           Serial.println(oldest_record);
        } 

        lpp_age = ((unixt-oldest_record)/86400);

        Serial.print("  LPP Epoch age (days): ");
        Serial.println( lpp_age );
      
        // if we have a fix, time is valid
        int32_t diff = unixt - RTC.getEpoch();
        Serial.print("  Correcting RTC by ");
        Serial.print(diff);
        Serial.println(" seconds.");
        // compensate our wakeup time
        last_epoch = last_epoch + diff;
        RTC.setEpoch(unixt);
        TimeToFix = (millis() - startTime)/1000;
        
        return(1);
    }
  
  return(0);  
}

#endif

unsigned long unixTimestamp(int year, int month, int day, int hour, int min, int sec) {
  const short days_since_beginning_of_year[12] = {0,31,59,90,120,151,181,212,243,273,304,334};
  int leap_years = ((year-1)-1968)/4
                  - ((year-1)-1900)/100
                  + ((year-1)-1600)/400;
  long days_since_1970 = (year-1970)*365 + leap_years
                      + days_since_beginning_of_year[month-1] + day-1;
  if ( (month>2) && (year%4==0 && (year%100!=0 || year%400==0)) )
    days_since_1970 += 1; /* +leap day, if year is a leap year */
  return sec + 60 * ( min + 60 * (hour + 24*days_since_1970) );
}

uint16_t predictpass(float latitude, float longitude, float altitude, uint32_t* start, uint32_t* duration, bool* rxtx, uint8_t* sat_id){

    uint32_t flags = LPP_FLAGS_SAT_TYPE_ANY;
    lpp_contact contact;
    
    time_t t = RTC.getEpoch();
    
    unsigned long startcalc = millis();

    int32_t lpp_result = lpp_next_contact(store, t, LPP_DEFAULT_LOOKAHEAD, longitude, latitude, MINELEVATION, &contact, flags);

    if ( !lpp_result ) {  
      
       unsigned long einde = millis();
       Serial.println("LPP computation time: " + String(einde-startcalc) + " milliseconds");
       
       Serial.println("Contact found");
       Serial.print("  Elevation: ");
       Serial.println(contact.max_elevation);
       Serial.print("  Time: ");
       Serial.println(contact.tca);
       Serial.print("  Duration: ");
       Serial.println(contact.contact_end-contact.contact_start);
       Serial.print("  Satellite ID: ");
       Serial.println(contact.sat_id);
       Serial.print("  Satellite name: ");
       Serial.println(sattelite_name(contact.sat_id));
       Serial.print("  Satellite type: ");
       switch(contact.contact_type) {
         case lpp_contact_type_rx: Serial.println("RX"); break;
         case lpp_contact_type_tx: Serial.println("TX"); break;
         default: Serial.println("Unknown"); break;
       }

       if (contact.contact_type == lpp_contact_type_rx) {
         // TX
         Serial.println("TX config:");
         Serial.print("  Frequency: ");
         Serial.println(contact.config.tx.frequency);
         Serial.print("  Power: ");
         Serial.println(contact.config.tx.power);
         Serial.print("  Bandwidth: ");
         Serial.println(contact.config.tx.bandwidth);
         Serial.print("  Coding rate: ");
         Serial.println(contact.config.tx.coding_rate);
         Serial.print("  Grid: ");
         Serial.println(contact.config.tx.grid);
         Serial.print("  Hopping: ");
         Serial.println(contact.config.tx.hopping_on ? "On" : "Off");      
         Serial.print("  Sync headers: ");
         Serial.println(contact.config.tx.nr_sync);
         Serial.print("  Spreading: ");
         Serial.println(contact.config.tx.time_spread);
  
         // Set TX parameters for next uplink
         SattxParams.frequency = contact.config.tx.frequency;
         SattxParams.power = contact.config.tx.power;
         SattxParams.codingRate = (lsLrfhssCodingRate)contact.config.tx.coding_rate;
         SattxParams.bandwidth = (lsLrfhssBandwidth)contact.config.tx.bandwidth;
         SattxParams.nbSync = contact.config.tx.nr_sync;
         SattxParams.grid = (lsLrfhssGrid)contact.config.tx.grid;
         SattxParams.hopping = contact.config.tx.hopping_on;
       } else {
         // RX
         Serial.println("RX config:");
         Serial.print("  Frequency: ");
         Serial.println(contact.config.rx.frequency);
         Serial.print("  Bandwidth: ");
         Serial.println(contact.config.rx.bandwidth);                
         Serial.print("  Preamble length: ");
         Serial.println(contact.config.rx.preamble_length);
         Serial.print("  Spreading factor: ");
         Serial.println(contact.config.rx.spreading_factor);

         // Set RX parameters for next downlink
         SatrxParams.frequency = contact.config.rx.frequency;
         SatrxParams.bandwidth = (lsLoraBandwidth)contact.config.rx.bandwidth;
         SatrxParams.preambleLength = contact.config.rx.preamble_length;
         SatrxParams.spreadingFactor = (lsLoraSpreadingFactor)contact.config.rx.spreading_factor;
       }
       
#ifdef FAKEPASS
        *start = RTC.getEpoch()+40;
        *duration = 30;
#else
        *start = contact.contact_start;
        *duration = contact.contact_end-contact.contact_start;
#endif
        *rxtx = contact.contact_type;
        *sat_id = contact.sat_id;
       
    } else {
          Serial.print("LPP Error: ");
          Serial.println(lpp_error_to_string(lpp_result));
          return(0);
    }
    
    return( (uint16_t)contact.max_elevation);
}


// Read HP203B
void HP203B(){

// HP203B 
unsigned int data[6];

  // Start I2C transmission
  Wire.beginTransmission(HP203B_Addr);
  // Send OSR and channel setting command
  Wire.write(0x40 |0x04 | 0x00);
  // Stop I2C transmission
  Wire.endTransmission();
  delay(500);

  // Start I2C transmission
  Wire.beginTransmission(HP203B_Addr);
  // Select data register
  Wire.write(0x10);
  // Stop I2C transmission
  Wire.endTransmission();

  // Request 6 bytes of data
  Wire.requestFrom(HP203B_Addr, 6);

  // Read 6 bytes of data
  // cTemp msb, cTemp csb, cTemp lsb, pressure msb, pressure csb, pressure lsb
  if (Wire.available() == 6)
  {
    data[0] = Wire.read();
    data[1] = Wire.read();
    data[2] = Wire.read();
    data[3] = Wire.read();
    data[4] = Wire.read();
    data[5] = Wire.read();
  }

  // Convert the data to 20-bits
  float cTemp = (((data[0] & 0x0F) * 65536) + (data[1] * 256) + data[2]) / 100.00;
  float fTemp = (cTemp * 1.8) + 32;
  float pressure_hp = (((data[3] & 0x0F) * 65536) + (data[4] * 256) + data[5]) / 100.00;

  // Start I2C transmission
  Wire.beginTransmission(HP203B_Addr);
  // Send OSR and channel setting command
  Wire.write(0x40 | 0x04 | 0x01);
  // Stop I2C transmission
  Wire.endTransmission();
  delay(500);

  // Start I2C transmission
  Wire.beginTransmission(HP203B_Addr);
  // Select data register
  Wire.write(0x31);
  // Stop I2C transmission
  Wire.endTransmission();

  // Request 3 bytes of data
  Wire.requestFrom(HP203B_Addr, 3);

  // Read 3 bytes of data
  // altitude msb, altitude csb, altitude lsb
  if (Wire.available() == 3)
  {
    data[0] = Wire.read();
    data[1] = Wire.read();
    data[2] = Wire.read();
  }

  // Convert the data to 20-bits
  float altitude = (((data[0] & 0x0F) * 65536) + (data[1] * 256) + data[2]) / 100.00;

  // Output data to serial monitor
  Serial.print("HP203B Altitude : ");
  Serial.print(altitude);
  Serial.println(" m");
  Serial.print("HP203B Pressure : ");
  Serial.print(pressure);
  Serial.println(" Pa");
  Serial.print("HP203B Temperature in Celsius : ");
  Serial.print(cTemp);
  Serial.println(" C");
  //Serial.print("HP203B Temperature in Fahrenheit : ");
  //Serial.print(fTemp);
  //Serial.println(" F");

 

  temp = (cTemp)*100; // Convert signed to unsigned
  pressure = pressure_hp;

 
}
