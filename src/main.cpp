#include <Arduino.h>
#include "LittleFS.h"
#include "WiFiManager.h"
#include "webServer.h"
#include "updater.h"
#include "fetch.h"
#include "configManager.h"
#include "dashboard.h"
#include "timeSync.h"
#include <TimeLib.h>
#include <TZ.h>
#include <ArduinoJson.h>
#include <SPI.h>
#include <Adafruit_SSD1306.h>
#include "MHZ19.h"                                        
#include <SoftwareSerial.h>
#include <NeoPixelBrightnessBus.h> // instead of NeoPixelBus.h
#include <NeoPixelAnimator.h>
#include <Adafruit_BMP280.h>        // BMP280 lib
#include <ESP8266httpUpdate.h>     // Web Updater online


// I2C Scanner
uint8_t portArray[] = {5, 4};
//String portMap[] = {"D1", "D2"}; //for Wemos
String portMap[] = {"GPIO5", "GPIO4"};

// BME280 definitions

// BMP280 definitions
Adafruit_BMP280 bmp; // use I2C interface
Adafruit_Sensor *bmp_temp = bmp.getTemperatureSensor();
Adafruit_Sensor *bmp_pressure = bmp.getPressureSensor();


// WS2812 definitions
  const uint16_t PixelCount = 40; // this example assumes 4 pixels, making it smaller will cause a failure
  const uint8_t PixelPin = 3;  // make sure to set this to the correct pin, ignored for Esp8266
  //const RgbColor CylonEyeColor(HtmlColor(0x7f0000));
  const RgbColor CylonEyeColor(0,0,255);
#define colorSaturation 255 // saturation of color constants
RgbColor blue(0, 0, colorSaturation);

NeoPixelBrightnessBus<NeoGrbFeature, Neo800KbpsMethod> strip(PixelCount, PixelPin);
//NeoPixelBrightnessBus<NeoGrbFeature, Neo800KbpsMethod>* strip = NULL;
NeoPixelAnimator animations(2); // only ever need 2 animations

uint16_t lastPixel = 0; // track the eye position
int8_t moveDir = 1; // track the direction of movement

// uncomment one of the lines below to see the effects of
// changing the ease function on the movement animation
AnimEaseFunction moveEase =
//      NeoEase::Linear;
//      NeoEase::QuadraticInOut;
//      NeoEase::CubicInOut;
        NeoEase::QuarticInOut;
//      NeoEase::QuinticInOut;
//      NeoEase::SinusoidalInOut;
//      NeoEase::ExponentialInOut;
//      NeoEase::CircularInOut;

// NeoPixelBus TEST ENDE

// MH-Z19B definitions
#define RX_PIN 13                                          // Pin13 D7 Rx pin which the MHZ19 Tx pin is attached to
#define TX_PIN 15                                          // Pin15 D8 Tx pin which the MHZ19 Rx pin is attached to
#define BAUDRATE 9600                                      // Device to MH-Z19 Serial baudrate (should not be changed)

MHZ19 myMHZ19;                                             // Constructor for library
SoftwareSerial mySerial(RX_PIN, TX_PIN);                   // (Uno example) create device to MH-Z19 serial

// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

// no more delay with interval 
unsigned long previousMillis = 0;
const long interval = 100;   // 
byte periode = 0;

//--- BEGIN MQTT ------------------------------------
#include <PubSubClient.h>
boolean mqttconnected = false;
int mqttPublishTime;          // last publish time in seconds

// MQTT definitions
WiFiClient espclient;
PubSubClient MQTTclient(espclient);
#define MQTT_ID "CO2Light"
char buf[40];
#define MSG_BUFFER_SIZE	(20)
char result[MSG_BUFFER_SIZE];

// Update client
WiFiClient updateclient;

// Tasks
struct task
{    
    unsigned long rate;
    unsigned long previous;
};

task taskA = { .rate = 1000, .previous = 0 };     // 1 second
task taskC = { .rate = 15000, .previous = 0 };    // 15 seconds
task taskB = { .rate = 60000, .previous = 0 };    // 1 minute

// Global definitions
#define BUILTIN_LED 2             // On board LED
const int buzzer = D3;            // Buzzer
bool piep = false;                // helper Buzzer
bool reboot = false;              // Flag to reboot device
int seconds = 0; 
bool isCaptive = false; 	        // Captive portal active
unsigned long timeElapse = 0;
bool calibrationStarted = false;  
int prevseconds;
int displayPtr = 1;               // pointer for display items
bool played = false;              // helper for display
bool updatePending = false;       // Update pending
int smoothAnalog = 0;             // debounce analo value
int CursorX;                      // Cursor of display
int CursorY;                      // Cursor of display
boolean blinking = false;         // helper for blinking
int intensity = 30;               // LED intensity
String co2Colour;                 // LED colour

// SUBROUTINES
//*************************************************************************************

// I2C Portscanner
void check_if_exist_I2C() {
  byte error, address;
  int nDevices;
  nDevices = 0;
  for (address = 1; address < 127; address++ )  {
    // The i2c_scanner uses the return value of
    // the Write.endTransmisstion to see if
    // a device did acknowledge to the address.
    Wire.beginTransmission(address);
    error = Wire.endTransmission();

    if (error == 0){
      Serial.print(F("[DEBUG] I2C device found at address 0x"));
      if (address < 16) {
        Serial.print(F("0"));
        display.print("0");
        }
      Serial.print(address, HEX);
      Serial.println(F("  !"));

      display.print("0x");
      display.print(address, HEX);
      CursorY += 24;
      display.setCursor(0, CursorY);

      nDevices++;
    } else if (error == 4) {
      Serial.print(F("[DEBUG] Unknow error at address 0x"));
      if (address < 16)
        Serial.print(F("0"));
      Serial.println(address, HEX);
    }
  } 
      display.display();
  
  if (nDevices == 0)
    Serial.println(F("[DEBUG] No I2C devices found"));
  else
    Serial.println(F("[DEBUG] **********************************\n"));
}

void scanPorts() { 
  for (uint8_t i = 0; i < sizeof(portArray); i++) {
    for (uint8_t j = 0; j < sizeof(portArray); j++) {
      if (i != j){
        Serial.println("[DEBUG] Scanning (SDA : SCL) - " + portMap[i] + " : " + portMap[j]);
        Wire.begin(portArray[i], portArray[j]);
        check_if_exist_I2C();
      }
    }
  }
}


// NEOPIXELBUS ANIMATION BEGIN
void FadeAll(uint8_t darkenBy)
{
    RgbColor color;
    for (uint16_t indexPixel = 0; indexPixel < configManager.data.numofpixels; indexPixel++)
    {
        color = strip.GetPixelColor(indexPixel);
        color.Darken(darkenBy);
        strip.SetPixelColor(indexPixel, color);
    }
}

void FadeAnimUpdate(const AnimationParam& param)
{
    if (param.state == AnimationState_Completed)
    {
        FadeAll(10);
        animations.RestartAnimation(param.index);
    }
}

void MoveAnimUpdate(const AnimationParam& param)
{
    // apply the movement animation curve
    float progress = moveEase(param.progress);

    // use the curved progress to calculate the pixel to effect
    uint16_t nextPixel;
    if (moveDir > 0)
    {
        nextPixel = progress * configManager.data.numofpixels;
    }
    else
    {
        nextPixel = (1.0f - progress) * configManager.data.numofpixels;
    }

    // if progress moves fast enough, we may move more than
    // one pixel, so we update all between the calculated and
    // the last
    if (lastPixel != nextPixel)
    {
        for (uint16_t i = lastPixel + moveDir; i != nextPixel; i += moveDir)
        {
            strip.SetPixelColor(i, CylonEyeColor);
        }
    }
    strip.SetPixelColor(nextPixel, CylonEyeColor);

    lastPixel = nextPixel;

    if (param.state == AnimationState_Completed)
    {
        // reverse direction of movement
        moveDir *= -1;

        // done, time to restart this position tracking animation/timer
        animations.RestartAnimation(param.index);
    }
}

void SetupAnimations()
{
    // fade all pixels providing a tail that is longer the faster
    // the pixel moves.
    animations.StartAnimation(0, 5, FadeAnimUpdate);

    // take several seconds to move eye fron one side to the other
    animations.StartAnimation(1, 2000, MoveAnimUpdate);
}

// NEOPIXELBUS END

void setLED(uint8_t R, uint8_t G, uint8_t B) {
  RgbColor RGB(R,G,B);
  for(uint8_t i=0; i<configManager.data.numofpixels; i++) { // For each pixel...
    strip.SetPixelColor(i, RGB);
    strip.Show();
  }
}

void colorWipe(uint8_t R, uint8_t G, uint8_t B) {
  RgbColor RGB(R,G,B);
  for(uint8_t i=0; i<configManager.data.numofpixels; i++) { // For each pixel...
    strip.SetPixelColor(i, RGB);
    strip.Show();
    delay(20);
  }
}

void saveCallback() {
    intensity = configManager.data.matrixIntensity;
    strip.SetBrightness(intensity);
    RgbColor colorGreen(configManager.data.colorGreen[0],configManager.data.colorGreen[1],configManager.data.colorGreen[2]);
    RgbColor colorYellow(configManager.data.colorYellow[0],configManager.data.colorYellow[1],configManager.data.colorYellow[2]);
    RgbColor colorOrange(configManager.data.colorOrange[0],configManager.data.colorOrange[1],configManager.data.colorOrange[2]);
    RgbColor colorRed(configManager.data.colorRed[0],configManager.data.colorRed[1],configManager.data.colorRed[2]);    
}

void MqttCallback(char* topic, byte* payload, unsigned int length) {
  Serial.print(F("[INFO] Message arrived ["));
  Serial.print(topic);
  Serial.print(F("] "));
  for (int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
  }
  Serial.println();

    // convert payload to intuff
    payload[length] = '\0';
    String s = String((char*)payload);
    intensity = s.toInt();
    configManager.data.matrixIntensity = intensity;
  
    Serial.print(F("[INFO] Intensity: "));
    Serial.println(intensity);
    strip.SetBrightness(intensity);
}

// function to crate HTML Colour
void array_to_string(byte array[], unsigned int len, char buffer[])
{
    for (unsigned int i = 0; i < len; i++)
    {
        byte nib1 = (array[i] >> 4) & 0x0F;
        byte nib2 = (array[i] >> 0) & 0x0F;
        buffer[i*2+0] = nib1  < 0xA ? '0' + nib1  : 'A' + nib1  - 0xA;
        buffer[i*2+1] = nib2  < 0xA ? '0' + nib2  : 'A' + nib2  - 0xA;
    }
    buffer[len*2] = '\0';
}

void setLedColours(void) {
  char str[32] = "";

    if (dash.data.CO2 <= configManager.data.boarderGreen ) {                              // green
    colorWipe(configManager.data.colorGreen[0],configManager.data.colorGreen[1],configManager.data.colorGreen[2]);
    blinking = false;

    // convert to HTML colour
    array_to_string(configManager.data.colorGreen, 3, str);
    co2Colour = "#";
    co2Colour += String(str);
    //Serial.println(co2Colour);

    } 
  else if (( dash.data.CO2 > configManager.data.boarderGreen ) && ( dash.data.CO2 < configManager.data.boarderYellow)) {       // yellow
    colorWipe(configManager.data.colorYellow[0],configManager.data.colorYellow[1],configManager.data.colorYellow[2]);
    blinking = false;

    // convert to HTML colour
    array_to_string(configManager.data.colorYellow, 3, str);
    co2Colour = "#";
    co2Colour += String(str);
    //Serial.println(co2Colour);

  }
  else if (( dash.data.CO2 >= configManager.data.boarderYellow) && ( dash.data.CO2 < configManager.data.boarderOrange)) {      // orange
    colorWipe(configManager.data.colorOrange[0],configManager.data.colorOrange[1],configManager.data.colorOrange[2]);
    blinking = false;
  
    // convert to HTML colour
    array_to_string(configManager.data.colorOrange, 3, str);
    co2Colour = "#";
    co2Colour += String(str);
    //Serial.println(co2Colour);

  }
  else if (( dash.data.CO2 >= configManager.data.boarderOrange) && ( dash.data.CO2 < configManager.data.boarderRed)) {      // orange
    colorWipe(configManager.data.colorRed[0],configManager.data.colorRed[1],configManager.data.colorRed[2]);
    blinking = false;

    // convert to HTML colour
    array_to_string(configManager.data.colorRed, 3, str);
    co2Colour = "#";
    co2Colour += String(str);
    //Serial.println(co2Colour);

  }
  else {
    setLED(configManager.data.colorRed[0],configManager.data.colorRed[1],configManager.data.colorRed[2]);                              // red
    blinking = true;

    // convert to HTML colour
    array_to_string(configManager.data.colorRed, 3, str);
    co2Colour = "#";
    co2Colour += String(str);
    //Serial.println(co2Colour);
  }

}


void PublishMQTT(void) {                     //MQTTclient.publish
  
    // Publish CO2 value
    String topic = "CO2Light/";
           topic = topic + configManager.data.place;
           topic = topic +"/CO2";
           dtostrf(dash.data.CO2, 5, 0, result);
           MQTTclient.publish(topic.c_str(), result);

    // Publish Intensity       
           topic = "CO2Light/";
           topic = topic + configManager.data.place;
           topic = topic +"/Brightness";
           dtostrf(configManager.data.matrixIntensity, 5, 0, result);
           MQTTclient.publish(topic.c_str(), result);
          
    // Publish CO2 Colour
           topic = "CO2Light/";
           topic = topic + configManager.data.place;
           topic = topic +"/CO2Colour";
           MQTTclient.publish( topic.c_str(), co2Colour.c_str() );    
    
    // Publish Temperature
          topic = "CO2Light/";
          topic = topic + configManager.data.place;
          topic = topic +"/Temperature";
          MQTTclient.publish(topic.c_str(), dash.data.Temperature); 

    // Publish Pressure
    if (configManager.data.sensorType == 1) {
          topic = "CO2Light/";
          topic = topic + configManager.data.place;
          topic = topic +"/Pressure";
          MQTTclient.publish(topic.c_str(), dash.data.Pressure); 
    }

    // Publish MQ-Sensor
          topic = "CO2Light/";
          topic = topic + configManager.data.place;
          topic = topic +"/MQ-Sensor";
          dtostrf(dash.data.MQ_Sensor, 4, 0, result);
          MQTTclient.publish(topic.c_str(), result); 

}

void reconnect(void) {
  // reconnect to MQTT Server
  if (!MQTTclient.connected()) {
    dash.data.MQTT_Connected = false;
    Serial.println("Attempting MQTT connection...");
    // Create a random client ID
    String clientId = "CO2Light-";
    clientId += String(WiFi.macAddress());
    //clientId += String(random(0xffff), HEX);
    
    // Attempt to connect
    if (MQTTclient.connect(clientId.c_str(),configManager.data.mqtt_user,configManager.data.mqtt_password)) {
      Serial.println("connected");
      dash.data.MQTT_Connected = true;
      // Once connected, publish an announcement...
      PublishMQTT();
      // ... and resubscribe
      String topic = "CO2Light/";
           topic = topic + configManager.data.place;
           topic = topic + "/Brightness";
      MQTTclient.subscribe(topic.c_str());
      //MQTTclient.subscribe((char*) topic.c_str());

    } else {
      Serial.print("failed, rc=");
      Serial.print(MQTTclient.state());
      Serial.println(" try again in one minute");
     }
    }
}

void verifyRange(int range);    // Need for calibration

void calibrationActive() {      // Calibration in progress
    if (dash.data.CO2_Calibration) {
      dash.data.CO2_Calibration = false;
      seconds = 0;
      calibrationStarted = true;          // starting calibration mode
      Serial.print(F("[INFO] calibrationStarted "));
      Serial.println(calibrationStarted);
      myMHZ19.autoCalibration(false);     // make sure auto calibration is off
      Serial.print("[DEBUG] ABC Status: "); myMHZ19.getABC() ? Serial.println("ON") :  Serial.println("OFF");  // now print it's status
      Serial.println("[INFO] Waiting 20 minutes to stabalise...");
      // Info on display
      display.clearDisplay();
      display.setCursor(0,0);
      display.setTextSize(2);
      display.print("Calibrate");
      display.setCursor(0,24);
      display.setTextSize(2);
      display.print("Started");
      display.setCursor(0,48);
      display.setTextSize(1);
      display.print("Please wait!");
      display.display();
      delay(4000);
    }
    if (seconds < 1200) {       // 20 minutes = 1200 seconds
      if (seconds > prevseconds) {
        prevseconds = seconds;
        display.clearDisplay();
        display.setCursor(0,0);
        display.setTextSize(2);
        display.print("Calibrate");
        display.setCursor(0,30);
        display.setTextSize(2);
        int remaining = 1200 - seconds;
        display.print(remaining);
        display.setCursor(0,48);
        display.setTextSize(1);
        display.print("seconds ramaining");
        display.display();
        Serial.print(F("[DEBUG] Calibration"));
        Serial.print(remaining);
        Serial.println(F(" seconds remaining"));
    } 
  } else {
        myMHZ19.calibrate();    // Take a reading which be used as the zero point for 400 ppm
        myMHZ19.autoCalibration(true);     // make sure auto calibration is on
        calibrationStarted = false;
        seconds = 0;
    }
}

void printCO2() {
  // clear display
  display.clearDisplay();
  // display co2
  display.setCursor(0,0);
  display.setTextSize(2);
  display.print("CO2");
  display.setCursor(0, 36);
  display.setTextSize(4);
  if (dash.data.CO2 == 0 ) {
    display.print("----");
  } else {
  display.print(dash.data.CO2);
  }
  display.setTextSize(1);
  display.print(" ppm"); 
  
  display.display(); 
}

void printTemperature() {
    // clear display
  display.clearDisplay();
  display.setCursor(0,0);
  display.setTextSize(2);
  display.print("Temperatur");
  display.setCursor(0, 36);
  display.setTextSize(3);
  display.print(dash.data.Temperature);
  display.setTextSize(1);
  display.cp437(true);
  display.write(521);
  display.setTextSize(2);
  display.print("C");
  display.display();
}

void printPressure() {
  // clear display
  display.clearDisplay();
  display.setCursor(0,0);
  display.setTextSize(2);
  display.print("Luftdruck");
  display.setCursor(0, 36);
  display.setTextSize(3);
  if (dash.data.Pressure == 0 ) {
    display.print("----");
  } else {
  display.print(dash.data.Pressure);
  }
  display.setTextSize(1);
  display.print(" hPa"); 
  
  display.display();

}

void printWifimode() {
    display.clearDisplay();
      if (isCaptive) {
        display.setCursor(0,0);
        display.setTextSize(2);
        display.print("Captive");
        display.setCursor(0,16);
        display.print("Portal");
        display.setCursor(0,40);
        display.setTextSize(1);
        display.print("IP Adresse");
        display.setCursor(0,56);
        display.setTextSize(1);
        display.print("192.168.4.1");
        display.display();
      } else {
        display.setCursor(0,0);
        display.setTextSize(2);
        display.print("Client");
        display.setCursor(0,16);
        display.print("Mode");
        display.setCursor(0,40);
        display.setTextSize(1);
        display.print("IP Adresse");
        display.setCursor(0,56);
        display.setTextSize(1);
        display.print(WiFi.localIP());
        display.display();
      }
}

void DisplayValues (void) { 
  played = false;
  seconds = 0;
  
    if (displayPtr == 1) {
          Serial.println(F("[DEBUG] show CO2"));
          printCO2();
          played = true;
        } else if (displayPtr == 2) {
           Serial.println(F("[DEBUG] show Temperature"));
           printTemperature();
           played = true;
        } else if ((displayPtr == 3 ) && (configManager.data.sensorType == 1)) {
          Serial.println(F("[DEBUG] show Pressure"));
           printPressure();
          played = true;
        } else if (displayPtr == 4) {
           Serial.println(F("[DEBUG] show Wifi Mode"));       
           printWifimode();
           played = true;
        } else {
          displayPtr++;
        }             
        
    if (displayPtr >=5) {
      displayPtr = 1;
    }

    if (played) {
      displayPtr++;
    }

  }
 
void getCO2() {
  // MH-Z19B reading
        // note: getCO2() default is command "CO2 Unlimited". This returns the correct CO2 reading even 
        // if below background CO2 levels or above range (useful to validate sensor). You can use the 
        // usual documented command with getCO2(false) 
        Serial.println(F("[DEBUG] read CO2 value"));
  
        dash.data.CO2 = myMHZ19.getCO2();                             // Request CO2 (as ppm)
        if (configManager.data.sensorType == 0) {
            dash.data.Temperature = myMHZ19.getTemperature();         // Request Temperature (as Celsius)
        }
        
               if(myMHZ19.errorCode == RESULT_OK)              // RESULT_OK is an alis for 1. Either can be used to confirm the response was OK.
        {
            Serial.print(F("[DEBUG] co2 Value successfully Recieved: "));
            Serial.println(dash.data.CO2);
            Serial.print(F("[DEBUG] Response Code: "));
            Serial.println(myMHZ19.errorCode);          // Get the Error Code value

            
        } else {
            Serial.println(F("[DEBUG] Failed to recieve CO2 value - Error"));
            Serial.print(F("[DEBUG] Response Code: "));
            Serial.println(myMHZ19.errorCode);          // Get the Error Code value
            dash.data.CO2 = 0;     
        }  
}

void getSensor() {
     // Read Sensors
      if (configManager.data.sensorType == 1) {
        sensors_event_t temp_event, pressure_event;
        bmp_temp->getEvent(&temp_event);
        bmp_pressure->getEvent(&pressure_event);
  
        Serial.print(F("[INFO] Temperature = "));
        Serial.print(temp_event.temperature);
        Serial.println(" *C");

        Serial.print(F("[INFO] Pressure = "));
        Serial.print(pressure_event.pressure);
        Serial.println(" hPa");
  
        dtostrf(temp_event.temperature, 2, 1, dash.data.Temperature);   // float to char
        dtostrf(pressure_event.pressure, 5, 1, dash.data.Pressure);   // float to char

      } 
}

void update_started() {
  display.clearDisplay();
  display.setCursor(0,0);
  display.setTextSize(2);
  display.print("Update");
  display.setCursor(0, 36);
  display.setTextSize(2);
  display.print("started");
  display.display();
  Serial.println(F("[INFO] HTTP update process started"));
}

void update_finished() {
  display.clearDisplay();
  display.setCursor(0,0);
  display.setTextSize(2);
  display.print("Update");
  display.setCursor(0, 36);
  display.setTextSize(2);
  display.print("done");
  display.display();
  Serial.println(F("[INFO] HTTP update process finished"));
}

void update_progress(int cur, int total) {
  char progressString[10];
  float percent = ((float)cur   / (float)total )  * 100;
  //sprintf(progressString, " %s",  String(percent).c_str()  );
  sprintf(progressString, " %s",  String(percent,0).c_str()  );
  display.clearDisplay();
  display.setCursor(0,0);
  display.setTextSize(2);
  display.print("Update");
  display.setCursor(0, 36);
  display.setTextSize(3);
  display.print(progressString);
  display.print(" %");
  display.display();  
  Serial.printf("[INFO] HTTP update process at %d of %d bytes...\n", cur, total);
}

void update_error(int err) {
  char errorString[8];
  sprintf(errorString, "Err %d", err);
  display.clearDisplay();
  display.setCursor(0,0);
  display.setTextSize(2);
  display.print("Update");
  display.setCursor(0, 36);
  display.setTextSize(2);
  display.print(errorString);
  display.display();  
  Serial.printf("[INFO] HTTP update fatal error code %d\n", err);
}

void updateFirmware() {
    ESPhttpUpdate.setLedPin(LED_BUILTIN, LOW);
    // Add optional callback notifiers
    ESPhttpUpdate.onStart(update_started);
    ESPhttpUpdate.onEnd(update_finished);
    ESPhttpUpdate.onProgress(update_progress);
    ESPhttpUpdate.onError(update_error);

    t_httpUpdate_return ret = ESPhttpUpdate.update(updateclient, "http://firmware.kidbuild.de/CO2light/firmware.bin");

    switch (ret) {
      case HTTP_UPDATE_FAILED:
        Serial.printf("[ERROR] HTTP_UPDATE_FAILD Error (%d): %s\n", ESPhttpUpdate.getLastError(), ESPhttpUpdate.getLastErrorString().c_str());
        break;

      case HTTP_UPDATE_NO_UPDATES:
        Serial.println(F("[INFO] HTTP_UPDATE_NO_UPDATES"));
        break;

      case HTTP_UPDATE_OK:
        Serial.println(F("[INFO] HTTP_UPDATE_OK"));
        break;
    }  
}

void setup() {
  Serial.begin(115200);

  LittleFS.begin();
  GUI.begin();
  configManager.begin();
  WiFiManager.begin(configManager.data.projectName);
  configManager.setConfigSaveCallback(saveCallback);
  dash.begin(500);

  // WiFi
  WiFi.hostname(configManager.data.wifi_hostname);
  WiFi.begin();

  // Timesync
  timeSync.begin(configManager.data.Time_Zone);

  //Onboard LED & analog port, etc
  pinMode(BUILTIN_LED,OUTPUT);                 // LED
  pinMode( A0 , INPUT);                        // Analog A0
  pinMode(buzzer, OUTPUT);                     // Buzzer
  digitalWrite(BUILTIN_LED,1);                 // LED off

  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println(F("[DEBUG] SSD1306 allocation failed"));
    for(;;);
  }
  delay(2000);
  display.clearDisplay();
  display.setTextColor(WHITE);

  // Say hello to the world
  display.clearDisplay();
  display.setCursor(0,0);
  display.setTextSize(3);
  display.print("booting");
  display.setCursor(0,30);
  display.setTextSize(1);
  display.print("Wifi Manager");
  display.setCursor(0,40);
  display.setTextSize(1);
  display.print("Please wait!");
  display.display(); 

  // NeoPixelBus SETUP
  // this resets all the neopixels to an off state
    strip.Begin();
    intensity = configManager.data.matrixIntensity;
    strip.SetBrightness(intensity);
    SetupAnimations();
    colorWipe(0,0,255);
    strip.Show();


  // MQTT
  MQTTclient.setServer(configManager.data.mqtt_server, configManager.data.mqtt_port);
  MQTTclient.setCallback(MqttCallback);

  // MH-Z19B construct
  mySerial.begin(BAUDRATE);                               // (Uno example) device to MH-Z19 serial start   
  myMHZ19.begin(mySerial);                                // *Serial(Stream) refence must be passed to library begin(). 
  myMHZ19.autoCalibration(true);                              // Turn auto calibration ON (OFF autoCalibration(false))

  char myVersion[4];          
  myMHZ19.getVersion(myVersion);
  Serial.print(F("[DEBUG] Firmware Version: "));
  for(byte i = 0; i < 4; i++) {
    Serial.print(myVersion[i]);
    if(i == 1)
      Serial.print(".");    
  }
   Serial.println("");
   Serial.print("Range: ");
   Serial.println(myMHZ19.getRange());   
   Serial.print("Background CO2: ");
   Serial.println(myMHZ19.getBackgroundCO2());
   Serial.print("Temperature Cal: ");
   Serial.println(myMHZ19.getTempAdjustment());
   Serial.print("ABC Status: "); myMHZ19.getABC() ? Serial.println("ON") :  Serial.println("OFF");

   // I2C Portscanner
   Serial.println("[DEBUG] I2C Scanner to scan for devices on each port pair D1 to D2");
    display.clearDisplay();
    CursorY = 0;
    display.setCursor(0,CursorY);
    display.setTextSize(2);
    display.print("I2C Addr.");
    CursorY += 24;
    display.setCursor(0, CursorY);
    scanPorts();
   
    // BME280 SETUP
    
    // BMP280 SETUP
    if (configManager.data.sensorType == 1) {
      Serial.println(F("[DEBUG] BMP280 Sensor event test"));
      unsigned status;
      //status = bmp.begin(BMP280_ADDRESS_ALT, BMP280_CHIPID);
      status = bmp.begin(configManager.data.sensorAddress);
      if (!status) {
        Serial.println(F("[DEBUG] Could not find a valid BMP280 sensor, check wiring or "
                        "try a different address!"));
        Serial.print("SensorID was: 0x"); Serial.println(bmp.sensorID(),16);
        Serial.print("        ID of 0xFF probably means a bad address, a BMP 180 or BMP 085\n");
        Serial.print("   ID of 0x56-0x58 represents a BMP 280,\n");
        Serial.print("        ID of 0x60 represents a BME 280.\n");
        Serial.print("        ID of 0x61 represents a BME 680.\n");
      } else {
        /* Default settings from datasheet. */
        bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                   Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                    Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                    Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                    Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */
        bmp_temp->printSensorDetails();
      }
    }

}

void loop() {

 // framework things
  yield();
  WiFiManager.loop();
  updater.loop();
  configManager.loop();
  dash.loop();
  if (configManager.data.mqttEnable) {              
        MQTTclient.loop();
      }

  // NeoPixelBus
  if (calibrationStarted == true) {
    animations.UpdateAnimations();
    strip.Show();
  }


//tasks
    if (taskA.previous == 0 || (millis() - taskA.previous > taskA.rate)) {
        taskA.previous = millis();

      seconds++;

      // Read Sensor Data
      smoothAnalog = analogRead(A0);
      dash.data.MQ_Graph = smoothAnalog;
      dash.data.MQ_Sensor = smoothAnalog;

      // Check if reboot flag is set
      isCaptive = WiFiManager.isCaptivePortal();

        int rssi = 0;
        rssi = WiFi.RSSI();
        sprintf(dash.data.Wifi_RSSI, "%ld", rssi) ;
        dash.data.WLAN_RSSI = WiFi.RSSI();
      
      if (calibrationStarted == false) {

        // set colors of LED
      if((myMHZ19.errorCode == RESULT_OK) && (dash.data.CO2 != 0)) {
            // Change the colours regarding the CO2 value
            setLedColours();
          } 
      
        // red blinking
      if (blinking) {
        if (intensity > 0) {
          intensity = 0;
          strip.SetBrightness(intensity);
        } else {
          intensity = configManager.data.matrixIntensity;
          strip.SetBrightness(intensity);
        }
        // Buzzer
        if (!piep) {
          piep = true;
          tone(buzzer, 1000); // play tone
        } else {
          noTone(buzzer);     // silent
        }

      } else {
          intensity = configManager.data.matrixIntensity;
          strip.SetBrightness(intensity);
          piep = false;   // play tone again
        }
      
      if (mqttPublishTime <= configManager.data.mqtt_interval) {
        mqttPublishTime++;
      } else {
        PublishMQTT();
        mqttPublishTime = 0;
      }

      if (seconds >= 5) {
        DisplayValues();
      }

      } else {
    calibrationActive();
    } 

       // update pending
        if (updatePending) {
          updatePending = false;
          updateFirmware();
        }

      // reboot pending
        if (reboot) {
          reboot = false;
          ESP.reset();
        }
    
}

    if (taskB.previous == 0 || (millis() - taskB.previous > taskB.rate)) {
        taskB.previous = millis();
      
      // MQTT Stuff
      if (configManager.data.mqttEnable) {              
        MQTTclient.loop();
        reconnect();
      }
    }

    if (taskC.previous == 0 || (millis() - taskC.previous > taskC.rate)) {
        taskC.previous = millis();

        getSensor();
        getCO2();
    }

    // Update from dashboard 
    if (dash.data.Online_update) {
        dash.data.Online_update = false;
        dash.loop();
        updatePending = true;
    }

       // Reboot from dashboard 
    if (dash.data.Reboot) {
        dash.data.Reboot = false;
        dash.loop();
        reboot = true;
    }

  if (dash.data.CO2_Calibration) {
    calibrationActive();
  }
}

