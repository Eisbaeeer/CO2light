![Logo](pics/logo.jpg)
# CO2light by Kidbuild

## Description
This project shows you the air quality of your enviroment. I use MH-Z19B sensor to get the enviroment air quality.
To get a quick view of the CO2 value, there is a WS2812B stripe who will show the quality in four steps. The SSD1306 display 
will give you more informations about the web page and the messured values. You can change the ranges of colours against the CO2 value. 

## Features
- the code creates a filesystem on flash storage of the esp8266
- all settings are stored on the filesystem in a JSON format
- Wifi-Manager for easy connection to available AccessPoints
- MQTT client to transmit the values to a central server like home-automation-systems
- Brightness configurable via MQTT subscription (e.g. for automated day/night mode)
- Publishing the colour of CO2 value via MQTT in HTML code (e.g. to display in homeautiomation)
- Publishing the CO2 value via MQTT
- Webpage to configure all settings or read the values
- OTA Over-The-Air update of firmware
- Colour picker for all colour states
- Colour ranges configurable
- Manual calibration of CO2 value possible via dashboard

## Schematic

| NodeMCU Board Pins  | Device Pin         | Device Name  |   
|---------------------|--------------------|--------------|   
| GND                 | GND                | SSD1309 OLED |   
| 3.3V                | Vin                | SSD1309 OLED |   
| D1                  | SCL                | SSD1309 OLED |   
| D2                  | SDA                | SSD1309 OLED |   
| D6                  | Data in            | WS2812B LED  |   
| Vin or 5V           | +5V                | WS2812B LED  |
| GND                 | GND                | WS2812B LED  |
| D7                  | Pin6 UART TXT (gn) | MH-Z19 CO2   |
| D8                  | Pin5 UART RXT (bl) | MH-Z19 CO2   |
| GND                 | Pin3 GND (sw)      | MH-Z19 CO2   |
| Vin or 5V           | Pin4 +5V (rt)      | MH-Z19 CO2   |


![Logo](pics/red.jpg)
![Logo](pics/sensor.jpg)
![Logo](pics/Config.jpg)
![Logo](pics/dashboard.jpg)
![Logo](pics/iobroker-mqtt.jpg)

## Steps to get running
1. Install VS-Code
2. https://github.com/maakbaas/esp8266-iot-framework/blob/master/docs/getting-started.md
3. Install required libraries: PubSubClient, Adafruit_GFX, Adafruit_SSD1306; Adafruit_NeoPixel; MHZ19 ... 
4. Compile


## Weblinks to get running
- http://www.kidbuild.de or https://shop.kidbuild.de
E-Mail info@kidbuild.de

## ToDo
- Adding piezo for audio alarm

## Changelog 

### Version 2.0   
- Added manual calibration via dashboard (takes 20 minutes in fresh air)
- Changed structure of code
- Moved default boarders of CO2 colors
- Updated framework
- Changed brightness to slider


### Version 1.3   
(Eisbaeeer 20211209)
- Added colour picker for each colour
- Added MQTT enable / disable button
- Brightness adjustable via MQTT subscription "Brightness"
- Publishing LED colour as HTML 
- LED pixels count configurable

### Version 1.0
(Eisbaeeer 20211126)
- Migrated project to iot framework   
- MQTT settings configurabel, now
- Colour ranges configurable, now
- Wifi hostname configurable, now
- Wifi RSSI visible on dashboard

### Verison 0.4
(Eisbaeeer 20201222)   
- Only send MQTT if request of CO2 is valid

### Version 0.3
(Eisbaeeer 20201220)   
- Added temperature, humidity and ppm on webpage

### Version 0.2 
(Eisbaeeer 20201220)   
- added WifiManager
- added Website for config brightness
- added HttpUpdateServer
- added MQTT

### Version 0.1 
(Eisbaeeer 20201216)   
- initial version
