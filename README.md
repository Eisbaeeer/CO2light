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
- Webpage to configure all settings or read the values
- OTA Over-The-Air update of firmware

![Logo](pics/red.jpg)
![Logo](pics/sensor.jpg)

## Steps to get running
1. Install VS-Code
2. https://github.com/maakbaas/esp8266-iot-framework/blob/master/docs/getting-started.md
3. Install required libraries: PubSubClient, Adafruit_GFX, Adafruit_SSD1306; Adafruit_NeoPixel; MHZ19 ... 
4. Compile


## Weblinks to get running
- http://www.kidbuild.de or https://shop.kidbuild.de
E-Mail info@kidbuild.de

## Changelog 

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
