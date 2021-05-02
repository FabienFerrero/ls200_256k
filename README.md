# ls200_256k

From https://github.com/LacunaSpace/arduino-STM32L4/tree/lacuna


Installing
Board Manager

* Download and install the Arduino IDE (at least version v1.8.13)
* Start the Arduino IDE
* Go into Preferences
* Add https://lacunaspace.github.io/arduino-STM32L4-Lacuna/package_STM32L4_Lacuna_boards_index.json as an "Additional Board Manager URL"
* Open the Boards Manager from the Tools -> Board menu and install "STM32L4 Boards (Lacuna) by Tlera Corp/Lacuna Space"
* Select your STM32L4 board from the Tools -> Board menu

<<<<<<< Updated upstream
The board has been successfully tested with several SX1262 library :

LoRaWAN Mac protocol :
https://github.com/FabienFerrero/basicmac    forked from : https://github.com/LacunaSpace/basicmac

LoRa Physical layer :
https://github.com/StuartsProjects/SX12XX-LoRa


Known issues :

* For a GNSS fix with multiple GNSS system (not only GPS, but also Galileo or Glonass), the number of satellite and altitude is always 0 
*  AHT10 sensor has been removed because on conflict on I2C

