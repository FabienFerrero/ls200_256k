/*******************************************************************************
 * Copyright (c) 2015 Thomas Telkamp and Matthijs Kooijman
 *
 * Permission is hereby granted, free of charge, to anyone
 * obtaining a copy of this document and accompanying files,
 * to do whatever they want with them without any restriction,
 * including, but not limited to, copying, modification and redistribution.
 * NO WARRANTY OF ANY KIND IS PROVIDED.
 *
 * This example sends a valid LoRaWAN packet with payload "Hello,
 * world!", using frequency and encryption settings matching those of
 * the The Things Network.
 *
 * This uses ABP (Activation-by-personalisation), where a DevAddr and
 * Session keys are preconfigured (unlike OTAA, where a DevEUI and
 * application key is configured, while the DevAddr and session keys are
 * assigned/generated in the over-the-air-activation procedure).
 *
 * Note: LoRaWAN per sub-band duty-cycle limitation is enforced (1% in
 * g1, 0.1% in g2), but not the TTN fair usage policy (which is probably
 * violated by this sketch when left running for longer)!
 *
 * To use this sketch, first register your application and device with
 * the things network, to set or generate a DevAddr, NwkSKey and
 * AppSKey. Each device should have their own unique values for these
 * fields.
 *
 * Do not forget to define the radio type correctly in config.h.
 *
 *******************************************************************************/


#define SHOW_DEBUGINFO


#include <basicmac.h>
#include <hal/hal.h>
#include <SPI.h>
#include <time.h>
#include <AHT10.h> // https://github.com/enjoyneering/AHT10.git
#include <LTR303.h> // https://github.com/automote/LTR303


// HP203B I2C address is 0x76(118)
#define Addr 0x76

// KX023
union data {
  int16_t data16;
  byte  byteStr[2];//{*DATAL,*DATAH}
};
union data xData;
union data yData;
union data zData;

const int sendorDeviceAddress = 0x1E;//I2C7bitAddressMode was 1F
const int regAddressXOUTL = 0x06;
const int regAddressYOUTL = 0x08;
const int regAddressZOUTL = 0x0A;
const int regAddressODCNTL = 0x1B;
const int regAddressCNTL1 = 0x18;

// KX023 
  byte CNTL1 = 0;

uint8_t readStatus = 0;

AHT10 myAHT10(AHT10_ADDRESS_0X38); // Create Temp/Hum sensor

// Create an LTR303 object, here called "light":
LTR303 light;  // Create Light sensor


// global enviromental parameters (exemples)
static float env_temp = 0.0;
static float env_pressure = 0.0;
static float env_humidity = 0.0;
static float env_batvalue;
static float env_luminosity;
static int env_acc_x = 0.0;
static int env_acc_y = 0.0;
static int env_acc_z = 0.0;


// Global variables LTR303:
unsigned char gain;     // Gain setting, values = 0-7 
unsigned char integrationTime;  // Integration ("shutter") time in milliseconds
unsigned char measurementRate;  // Interval between DATA_REGISTERS update

// LoRaWAN NwkSKey, network session key
// This is the default Semtech key, which is used by the early prototype TTN
// network.
static const PROGMEM u1_t NWKSKEY[16] = { 0x0B, 0x35, 0x9F, 0x53, 0x8B, 0x76, 0x99, 0xEE, 0xEC, 0x6C, 0x0C, 0x5F, 0xFA, 0x29, 0xCC, 0x55 };

// LoRaWAN AppSKey, application session key
// This is the default Semtech key, which is used by the early prototype TTN
// network.
static const u1_t PROGMEM APPSKEY[16] = { 0x9A, 0x41, 0xD3, 0xB4, 0xA7, 0x3B, 0x2E, 0x86, 0x1B, 0xAB, 0x16, 0xCA, 0x48, 0x07, 0x48, 0xCB };

// LoRaWAN end-device address (DevAddr)
static const u4_t DEVADDR = 0x2601394F ; // <-- Change this address for every node!

// These callbacks are only used in over-the-air activation, so they are
// left empty here (we cannot leave them out completely unless
// DISABLE_JOIN is set in config.h, otherwise the linker will complain).
void os_getJoinEui (u1_t* /* buf */) { }
void os_getDevEui (u1_t* /* buf */) { }
void os_getNwkKey (u1_t* /* buf */) { }

// The region to use, this just uses the first one (can be changed if
// multiple regions are enabled).
u1_t os_getRegion (void) { return LMIC_regionCode(0); }

// Schedule TX every this many milliseconds (might become longer due to duty
// cycle limitations).
const unsigned TX_INTERVAL = 60000;

// When this is defined, a standard pinmap from standard-pinmaps.ino
// will be used.  If you need to use a custom pinmap, comment this line
// and enter the pin numbers in the lmic_pins variable below.
#define USE_STANDARD_PINMAP

#if !defined(USE_STANDARD_PINMAP)
// All pin assignments use Arduino pin numbers (e.g. what you would pass
// to digitalWrite), or LMIC_UNUSED_PIN when a pin is not connected.
const lmic_pinmap lmic_pins = {
    // NSS input pin for SPI communication (required)
    .nss = 0,
    // If needed, these pins control the RX/TX antenna switch (active
    // high outputs). When you have both, the antenna switch can
    // powerdown when unused. If you just have a RXTX pin it should
    // usually be assigned to .tx, reverting to RX mode when idle).
    //
    // The SX127x has an RXTX pin that can automatically control the
    // antenna switch (if internally connected on the transceiver
    // board). This pin is always active, so no configuration is needed
    // for that here.
    // On SX126x, the DIO2 can be used for the same thing, but this is
    // disabled by default. To enable this, set .tx to
    // LMIC_CONTROLLED_BY_DIO2 below (this seems to be common and
    // enabling it when not needed is probably harmless, unless DIO2 is
    // connected to GND or VCC directly inside the transceiver board).
    .tx = LMIC_UNUSED_PIN,
    .rx = LMIC_UNUSED_PIN,
    // Radio reset output pin (active high for SX1276, active low for
    // others). When omitted, reset is skipped which might cause problems.
    .rst = 1,
    // DIO input pins.
    //   For SX127x, LoRa needs DIO0 and DIO1, FSK needs DIO0, DIO1 and DIO2
    //   For SX126x, Only DIO1 is needed (so leave DIO0 and DIO2 as LMIC_UNUSED_PIN)
    .dio = {/* DIO0 */ 2, /* DIO1 */ 3, /* DIO2 */ 4},
    // Busy input pin (SX126x only). When omitted, a delay is used which might
    // cause problems.
    .busy = LMIC_UNUSED_PIN,
    // TCXO oscillator enable output pin (active high).
    //
    // For SX127x this should be an I/O pin that controls the TCXO, or
    // LMIC_UNUSED_PIN when a crystal is used instead of a TCXO.
    //
    // For SX126x this should be LMIC_CONTROLLED_BY_DIO3 when a TCXO is
    // directly connected to the transceiver DIO3 to let the transceiver
    // start and stop the TCXO, or LMIC_UNUSED_PIN when a crystal is
    // used instead of a TCXO. Controlling the TCXO from the MCU is not
    // supported.
    .tcxo = LMIC_UNUSED_PIN,
};
#endif // !defined(USE_STANDARD_PINMAP)


void updateEnvParameters()
{

delay(10);

  KX023();
  delay(10);
  HP203B();
  delay(10);
  AHT10_();
  delay(10);
  LTR303_();
  delay(10);
  readVcc(); 

          
      
        #ifdef SHOW_DEBUGINFO
        // print out the value you read:
        Serial.print("Humidity : ");
        Serial.println(env_humidity);
        Serial.print("T°c : ");
        Serial.println(env_temp);
        Serial.print("Vbatt : ");
        Serial.println(env_batvalue);
        #endif
  
}

void readVcc() {  
  
  uint16_t voltage = (uint16_t)((LS_ADC_AREF / 4.096) * (LS_BATVOLT_R1 + LS_BATVOLT_R2) / LS_BATVOLT_R2 * (float)analogRead(LS_BATVOLT_PIN));

  env_batvalue = voltage/1000; // set environment voltage
 
  Serial.print("Voltage: ");
  Serial.println(voltage/1000.0f);
  
  }

void KX023(){
  //readXout
  Wire.beginTransmission(sendorDeviceAddress);
  Wire.write(regAddressXOUTL);
  Wire.endTransmission();
  Wire.requestFrom(sendorDeviceAddress, 2);
  xData.byteStr[0] = Wire.read();
  xData.byteStr[1] = Wire.read();

  //readYout
  Wire.beginTransmission(sendorDeviceAddress);
  Wire.write(regAddressYOUTL);
  Wire.endTransmission();
  Wire.requestFrom(sendorDeviceAddress, 2);
  yData.byteStr[0] = Wire.read();
  yData.byteStr[1] = Wire.read();

  //readZout
  Wire.beginTransmission(sendorDeviceAddress);
  Wire.write(regAddressZOUTL);
  Wire.endTransmission();
  Wire.requestFrom(sendorDeviceAddress, 2);
  zData.byteStr[0] = Wire.read();
  zData.byteStr[1] = Wire.read();

//  //rawDataOutput
//  Serial.print("xdata:");
//  Serial.print(xData.data16, DEC);
//  Serial.print(",");
//  Serial.print("ydata:");
//  Serial.print(yData.data16, DEC);
//  Serial.print(",");
//  Serial.print("zdata:");
//  Serial.println(zData.data16, DEC);

env_acc_x = xData.data16/16.384;
env_acc_y = yData.data16/16.384;
env_acc_z = zData.data16/16.384;

  Serial.print("KX023 Accel: ");
  Serial.print("x:");
  Serial.print(env_acc_x, DEC);
  Serial.print(", y:");
  Serial.print(env_acc_y, DEC);
  Serial.print(", z:");
  Serial.println(env_acc_z, DEC);
}

void HP203B(){
// HP203B 
unsigned int data[6];

  // Start I2C transmission
  Wire.beginTransmission(Addr);
  // Send OSR and channel setting command
  Wire.write(0x40 |0x04 | 0x00);
  // Stop I2C transmission
  Wire.endTransmission();
  delay(500);

  // Start I2C transmission
  Wire.beginTransmission(Addr);
  // Select data register
  Wire.write(0x10);
  // Stop I2C transmission
  Wire.endTransmission();

  // Request 6 bytes of data
  Wire.requestFrom(Addr, 6);

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
  float pressure = (((data[3] & 0x0F) * 65536) + (data[4] * 256) + data[5]) / 100.00;

  // Start I2C transmission
  Wire.beginTransmission(Addr);
  // Send OSR and channel setting command
  Wire.write(0x40 | 0x04 | 0x01);
  // Stop I2C transmission
  Wire.endTransmission();
  delay(500);

  // Start I2C transmission
  Wire.beginTransmission(Addr);
  // Select data register
  Wire.write(0x31);
  // Stop I2C transmission
  Wire.endTransmission();

  // Request 3 bytes of data
  Wire.requestFrom(Addr, 3);

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


  env_pressure = pressure ; // set environment pressure
  
}

void AHT10_(){
  /* DEMO - 1, every temperature or humidity call will read 6 bytes over I2C, total 12 bytes */
  //Serial.println(F("DEMO 1: read 12-bytes, show 255 if communication error is occurred"));
  env_temp = myAHT10.readTemperature();
  env_humidity = myAHT10.readHumidity();
  
  Serial.print(F("AHT10: Temperature: ")); Serial.print(env_temp); Serial.println(F(" +-0.3C")); //by default "AHT10_FORCE_READ_DATA"
  Serial.print(F("AHT10: Humidity...: ")); Serial.print(env_humidity);    Serial.println(F(" +-2%"));   //by default "AHT10_FORCE_READ_DATA"

   
  
}

void LTR303_(){
  int ms = 1000;
  delay(ms);
  unsigned int data0, data1;

  if (light.getData(data0,data1)) {
    // getData() returned true, communication was successful
    
   // Serial.print("data0: ");
   // Serial.println(data0);
   // Serial.print("data1: ");
   // Serial.println(data1);

     double lux;    // Resulting lux value
    boolean good;  // True if neither sensor is saturated
    
    // Perform lux calculation:

    good = light.getLux(gain,integrationTime,data0,data1,lux);

    env_luminosity = lux; // Set environment parameter
    
    // Print out the results:
    Serial.println("LTR303: ");
    Serial.print(" lux: ");
    Serial.println(lux);
    if (good) Serial.println(" (good)"); else Serial.println(" (BAD)");
  }
  else {
    // getData() returned false because of an I2C error, inform the user.

    byte error = light.getError();
    printError(error);
  }
}

  

void onLmicEvent (ev_t ev) {
    Serial.print(os_getTime());
    Serial.print(": ");
    switch(ev) {
        case EV_SCAN_TIMEOUT:
            Serial.println(F("EV_SCAN_TIMEOUT"));
            break;
        case EV_BEACON_FOUND:
            Serial.println(F("EV_BEACON_FOUND"));
            break;
        case EV_BEACON_MISSED:
            Serial.println(F("EV_BEACON_MISSED"));
            break;
        case EV_BEACON_TRACKED:
            Serial.println(F("EV_BEACON_TRACKED"));
            break;
        case EV_JOINING:
            Serial.println(F("EV_JOINING"));
            break;
        case EV_JOINED:
            Serial.println(F("EV_JOINED"));
            break;
        case EV_RFU1:
            Serial.println(F("EV_RFU1"));
            break;
        case EV_JOIN_FAILED:
            Serial.println(F("EV_JOIN_FAILED"));
            break;
        case EV_REJOIN_FAILED:
            Serial.println(F("EV_REJOIN_FAILED"));
            break;
        case EV_TXCOMPLETE:
            Serial.println(F("EV_TXCOMPLETE (includes waiting for RX windows)"));
            if (LMIC.txrxFlags & TXRX_ACK)
              Serial.println(F("Received ack"));
            if (LMIC.dataLen) {
              Serial.print(F("Received "));
              Serial.print(LMIC.dataLen);
              Serial.println(F(" bytes of payload"));
            }
            break;
        case EV_LOST_TSYNC:
            Serial.println(F("EV_LOST_TSYNC"));
            break;
        case EV_RESET:
            Serial.println(F("EV_RESET"));
            break;
        case EV_RXCOMPLETE:
            // data received in ping slot
            Serial.println(F("EV_RXCOMPLETE"));
            break;
        case EV_LINK_DEAD:
            Serial.println(F("EV_LINK_DEAD"));
            break;
        case EV_LINK_ALIVE:
            Serial.println(F("EV_LINK_ALIVE"));
            break;
        case EV_SCAN_FOUND:
            Serial.println(F("EV_SCAN_FOUND"));
            break;
        case EV_TXSTART:
            Serial.println(F("EV_TXSTART"));
            break;
        case EV_TXDONE:
            Serial.println(F("EV_TXDONE"));
            break;
        case EV_DATARATE:
            Serial.println(F("EV_DATARATE"));
            break;
        case EV_START_SCAN:
            Serial.println(F("EV_START_SCAN"));
            break;
        case EV_ADR_BACKOFF:
            Serial.println(F("EV_ADR_BACKOFF"));
            break;
         default:
            Serial.print(F("Unknown event: "));
            Serial.println(ev);
            break;
    }
}



void setup() {
    Serial.begin(115200);

    // Wait up to 5 seconds for serial to be opened, to allow catching
    // startup messages on native USB boards (that do not reset when
    // serial is opened).
    unsigned long start = millis();
    while (millis() - start < 5000 && !Serial);

    pinMode(LS_LED_BLUE, OUTPUT);
 
  while (!Serial && millis() < 5000);
  
  Serial.println("Initializing");

  Serial.print("Board version: ");
  Serial.println(BOARD_VERSION);

  initiate_sensor();  // Initiate sensors in the right mode

    
    // LMIC init
    os_init(nullptr);
    // Reset the MAC state. Session and pending data transfers will be discarded.
    LMIC_reset();

    // Set static session parameters. Instead of dynamically establishing a session
    // by joining the network, precomputed session parameters are be provided.
    #ifdef PROGMEM
    // On AVR, these values are stored in flash and only copied to RAM
    // once. Copy them to a temporary buffer here, LMIC_setSession will
    // copy them into a buffer of its own again.
    uint8_t appskey[sizeof(APPSKEY)];
    uint8_t nwkskey[sizeof(NWKSKEY)];
    memcpy_P(appskey, APPSKEY, sizeof(APPSKEY));
    memcpy_P(nwkskey, NWKSKEY, sizeof(NWKSKEY));
    LMIC_setSession (0x1, DEVADDR, nwkskey, appskey);
    #else
    // If not running an AVR with PROGMEM, just use the arrays directly
    LMIC_setSession (0x1, DEVADDR, NWKSKEY, APPSKEY);
    #endif

    #if defined(CFG_eu868)
    // These are defined by the LoRaWAN specification
    enum {
        EU_DR_SF12 = 0,
        EU_DR_SF11 = 1,
        EU_DR_SF10 = 2,
        EU_DR_SF9 = 3,
        EU_DR_SF8 = 4,
        EU_DR_SF7 = 5,
        EU_DR_SF7_BW250 = 6,
        EU_DR_FSK = 7,
    };

    // Set up the channels used by the Things Network, which corresponds
    // to the defaults of most gateways. Without this, only three base
    // channels from the LoRaWAN specification are used, which certainly
    // works, so it is good for debugging, but can overload those
    // frequencies, so be sure to configure the full frequency range of
    // your network here (unless your network autoconfigures them).
    // Setting up channels should happen after LMIC_setSession, as that
    // configures the minimal channel set.
    LMIC_setupChannel(0, 868100000, DR_RANGE_MAP(EU_DR_SF12, EU_DR_SF7));      // g-band
    LMIC_setupChannel(1, 868300000, DR_RANGE_MAP(EU_DR_SF12, EU_DR_SF7_BW250)); // g-band
    LMIC_setupChannel(2, 868500000, DR_RANGE_MAP(EU_DR_SF12, EU_DR_SF7));      // g-band
    LMIC_setupChannel(3, 867100000, DR_RANGE_MAP(EU_DR_SF12, EU_DR_SF7));      // g-band
    LMIC_setupChannel(4, 867300000, DR_RANGE_MAP(EU_DR_SF12, EU_DR_SF7));      // g-band
    LMIC_setupChannel(5, 867500000, DR_RANGE_MAP(EU_DR_SF12, EU_DR_SF7));      // g-band
    LMIC_setupChannel(6, 867700000, DR_RANGE_MAP(EU_DR_SF12, EU_DR_SF7));      // g-band
    LMIC_setupChannel(7, 867900000, DR_RANGE_MAP(EU_DR_SF12, EU_DR_SF7));      // g-band
    LMIC_setupChannel(8, 868800000, DR_RANGE_MAP(EU_DR_FSK,  EU_DR_FSK));      // g2-band

    // TTN uses SF9 at 869.525Mhz for its RX2 window (frequency is
    // default LoRaWAN, SF is different, but set them both to be
    // explicit).
    LMIC.dn2Freq = 869525000;
    LMIC.dn2Dr = EU_DR_SF9;

    // Set data rate for uplink
    LMIC_setDrTxpow(EU_DR_SF7, KEEP_TXPOWADJ);
    #elif defined(CFG_us915)
    // NA-US channels 0-71 are configured automatically
    // but only one group of 8 should (a subband) should be active
    // TTN recommends the second sub band, 1 in a zero based count.
    // https://github.com/TheThingsNetwork/gateway-conf/blob/master/US-global_conf.json
    // TODO: How to configure these channels? LMIC had LMIC_selectSubBand,
    // but it seems BasicMac only has LMIC_disableChannel.
    #endif

    // Disable link check validation
    LMIC_setLinkCheckMode(0);

    // Enable this to increase the receive window size, to compensate
    // for an inaccurate clock.  // This compensate for +/- 10% clock
    // error, a lower value will likely be more appropriate.
    //LMIC_setClockError(MAX_CLOCK_ERROR * 10 / 100);

    // Queue first packet
    send_packet();
}

uint32_t last_packet = 0;

void loop() {
    // Let LMIC handle background tasks
    os_runstep();

    // If TX_INTERVAL passed, *and* our previous packet is not still
    // pending (which can happen due to duty cycle limitations), send
    // the next packet.
    if (millis() - last_packet > TX_INTERVAL && !(LMIC.opmode & (OP_JOINING|OP_TXRXPEND)))
        send_packet();
}

void send_packet(){
    // Prepare upstream data transmission at the next possible time.

      updateEnvParameters();
              
        
        #ifdef SHOW_DEBUGINFO
            Serial.print(F("T="));
            Serial.println(env_temp);
        
            Serial.print(F("H="));
            Serial.println(env_humidity);
            Serial.print(F("L="));
            Serial.println(env_luminosity);
            Serial.print(F("BV="));
            Serial.println(env_batvalue);
            Serial.print(F("LUM="));
            Serial.println(env_luminosity);
            Serial.print(F("ACCX="));
            Serial.println(env_acc_x);
        #endif
            int t = (int)((env_temp) * 10.0);
            int h = (int)(env_humidity * 2);
            int bat = env_batvalue*100; // *100 for Cayenne LPP
            int l = env_luminosity; // light sensor in Lx
            int ax = env_acc_x; // accelero x
            int ay = env_acc_y; // accelero y
            int az = env_acc_z; // accelero z
        
            unsigned char mydata[23];
            mydata[0] = 0x1; // CH1
            mydata[1] = 0x67; // Temp
            mydata[2] = t >> 8;
            mydata[3] = t & 0xFF;
            mydata[4] = 0x2; // CH2
            mydata[5] = 0x68; // Humidity
            mydata[6] = h & 0xFF;
            mydata[7] = 0x3; // CH3
            mydata[8] = 0x2; // Analog output
            mydata[9] = bat >> 8;
            mydata[10] = bat & 0xFF;
            mydata[11] = 0x4; // CH4
            mydata[12] = 0x65; // Luminosity
            mydata[13] = l >> 8;
            mydata[14] = l & 0xFF;
            mydata[15] = 0x5; // CH5
            mydata[16] = 0x71; // Acceleromter
            mydata[17] = ax >> 8;
            mydata[18] = ax & 0xFF;
            mydata[19] = ay >> 8;
            mydata[20] = ay & 0xFF;
            mydata[21] = az >> 8;
            mydata[22] = az & 0xFF;
            
            LMIC_setTxData2(1, mydata, sizeof(mydata), 0);
            #ifdef SHOW_DEBUGINFO
            Serial.print(F("PQ")); //Packet queued
            Serial.println(F("Packet queued"));
            #endif
    
    last_packet = millis();
}

void initiate_sensor(){
  // Initiate sensors

// STM32

  uint32_t uid[3];
  STM32.getUID(uid);
  Serial.print("STM32 UID: ");
  Serial.print(uid[0],HEX);
  Serial.print(" ");
  Serial.print(uid[1],HEX);
  Serial.print(" ");
  Serial.println(uid[2],HEX);

  uint32_t ls_devid = crc32b((uint8_t*)uid);
  Serial.print("STM32 LSDevID: ");
  Serial.println(ls_devid,HEX);

  analogReadResolution(12);

  Wire.begin();

  pinMode(LS_GPS_ENABLE, OUTPUT);
  digitalWrite(LS_GPS_ENABLE, HIGH);
  pinMode(LS_GPS_V_BCKP, OUTPUT);
  digitalWrite(LS_GPS_V_BCKP, HIGH);
  digitalWrite(LS_VERSION_ENABLE, LOW);
  
  pinMode(5, OUTPUT);   // HALL_On_Off
  digitalWrite(5, LOW);

  pinMode(36, OUTPUT);
  digitalWrite(36, LOW);
  
 Serial.println("==========================");

  // KX023
  Wire.beginTransmission(sendorDeviceAddress);
  Wire.write(0x18);
  Wire.write(0x40);
  Wire.endTransmission();
  //outPutDataLate 50Hz
  Wire.beginTransmission(sendorDeviceAddress);
  Wire.write(0x1B);
  Wire.write(0x02);
  Wire.endTransmission();
  //sensor WakeUp
  Wire.beginTransmission(sendorDeviceAddress);
  Wire.write(0x18);
  Wire.write(0xC0);
  Wire.endTransmission();
  
  //setup LPF--------------------------------------------------
  //
  
  //set device standbyMode
  //readCNTL1reg
  
  byte CNTL1 = 0;
  Wire.beginTransmission(sendorDeviceAddress);
  Wire.write(regAddressCNTL1);
  Wire.endTransmission();
  Wire.requestFrom(sendorDeviceAddress, 1);
  CNTL1 = Wire.read();
  //setCNTL1reg
  CNTL1 = CNTL1 & 0b01111111;
  Wire.beginTransmission(sendorDeviceAddress);
  Wire.write(regAddressCNTL1);
  Wire.write(CNTL1);
  Wire.endTransmission();

  //set LPF parameters
  //readODCNTLreg
  byte ODCNTL = 0;
  Wire.beginTransmission(sendorDeviceAddress);
  Wire.write(regAddressODCNTL);
  Wire.endTransmission();
  Wire.requestFrom(sendorDeviceAddress, 1);
  ODCNTL = Wire.read();
  //setODCNTLreg
  ODCNTL = ODCNTL | 0b01000000;//set filter corner frequency set to ODR/2
  ODCNTL = ODCNTL & 0b11110000;//set OutputDataRate 12.5Hz
  Wire.beginTransmission(sendorDeviceAddress);
  Wire.write(regAddressODCNTL);
  Wire.write(ODCNTL);
  Wire.endTransmission();
  
  //set device operating mode
  //readCNTL1reg
  Wire.beginTransmission(sendorDeviceAddress);
  Wire.write(regAddressCNTL1);
  Wire.endTransmission();
  Wire.requestFrom(sendorDeviceAddress, 1);
  CNTL1 = Wire.read();
  //setCNTL1reg
  CNTL1 = CNTL1 | 0b10000000;
  Wire.beginTransmission(sendorDeviceAddress);
  Wire.write(regAddressCNTL1);
  Wire.write(CNTL1);
  Wire.endTransmission();
  //--------------------------------------------------setup LPF

//AHT10
  while (myAHT10.begin() != true)
  {
    Serial.println(F("AHT10 not connected or fail to load calibration coefficient")); //(F()) save string to flash & keeps dynamic memory free
    delay(5000);
  }
  Serial.println(F("AHT10 OK"));


// Initialize the LTR303 library
// 100ms   initial startup time required
  delay(100);

  // You can pass nothing to light.begin() for the default I2C address (0x29)
  light.begin();

  unsigned char ID;
  
  if (light.getPartID(ID)) {
    Serial.print("Got Sensor Part ID: 0X");
    Serial.print(ID,HEX);
  }
   else {
    byte error = light.getError();
    printError(error);
  }

  gain = 0;
  light.setControl(gain, false, false);
  unsigned char time = 1;
  light.setMeasurementRate(time,3);
  light.setPowerUp();

}

void printError(byte error) {
  // If there's an I2C error, this function will
  // print out an explanation.

  Serial.print("I2C error: ");
  Serial.print(error,DEC);
  Serial.print(", ");
  
  switch(error) {
    case 0:
      Serial.println("success");
      break;
    case 1:
      Serial.println("data too long for transmit buffer");
      break;
    case 2:
      Serial.println("received NACK on address (disconnected?)");
      break;
    case 3:
      Serial.println("received NACK on data");
      break;
    case 4:
      Serial.println("other error");
      break;
    default:
      Serial.println("unknown error");
  }

}

uint32_t crc32b(uint8_t *data) {
   int i, j;
   uint32_t byte, crc, mask;

   i = 0;
   crc = 0xFFFFFFFF;
   while (i < (4*3)) {
      byte = data[i];            // Get next byte.
      crc = crc ^ byte;
      for (j = 7; j >= 0; j--) {    // Do eight times.
         mask = -(crc & 1);
         crc = (crc >> 1) ^ (0xEDB88320 & mask);
      }
      i = i + 1;
   }
   return ~crc;
}
