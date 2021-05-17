# RFThings DK Blue




Installing
Board Manager

Please use the official RFTHings repository to configure your Arduino IDE with RFThings DK Blue : https://github.com/RFThings/arduino-STM32L4

The board has been successfully tested with several SX1262 library :

LoRaWAN Mac protocol :
https://github.com/FabienFerrero/basicmac    forked from : https://github.com/LacunaSpace/basicmac

LoRa Physical layer :
https://github.com/StuartsProjects/SX12XX-LoRa


Known issues :

* For a GNSS fix with multiple GNSS system (not only GPS, but also Galileo or Glonass), the number of satellite and altitude is always 0 
*  AHT10 sensor has been removed because on conflict on I2C

