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

boolean blinking = false;
int intensity = 30;     // working var for blinking etc.
String co2Colour;

// MH-Z19B definitions
#define RX_PIN D7                                          // Pin13 D7 Rx pin which the MHZ19 Tx pin is attached to
#define TX_PIN D8                                          // Pin15 D8 Tx pin which the MHZ19 Rx pin is attached to
#define BAUDRATE 9600                                      // Device to MH-Z19 Serial baudrate (should not be changed)

MHZ19 myMHZ19;                                             // Constructor for library
SoftwareSerial mySerial(RX_PIN, TX_PIN);                   // (Uno example) create device to MH-Z19 serial

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
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


// Tasks
struct task
{    
    unsigned long rate;
    unsigned long previous;
};

task taskA = { .rate = 1000, .previous = 0 };     // 1 second
task taskB = { .rate = 60000, .previous = 0 };     // 1 minute

// Global definitions
#define BUILTIN_LED 2  //On board LED
bool reboot = false;            // flag to reboot device
int seconds = 0; 
bool isCaptive = false; 	      // Captive portal active
unsigned long timeElapse = 0;
bool calibrationStarted = false;  
int prevseconds;

// SUBROUTINES
//*************************************************************************************

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
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  for (int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
  }
  Serial.println();

    // convert payload to intuff
    payload[length] = '\0';
    String s = String((char*)payload);
    intensity = s.toInt();
    configManager.data.matrixIntensity = intensity;
  
    Serial.print(F("Intensity: "));
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

void printDisplay(void) {
  // clear display
  display.clearDisplay();
  // display temperature
  display.setCursor(0,0);
  display.setTextSize(4);
  display.print(dash.data.Temperature,1);
  display.setTextSize(3);
  display.cp437(true);
  display.write(521);
  display.setTextSize(4);
  display.print("C");
  // display co2
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
}

void reconnect(void) {
  // reconnect to MQTT Server
  if (!MQTTclient.connected()) {
    dash.data.MQTT_Connected = false;
    Serial.println("Attempting MQTT connection...");
    // Create a random client ID
    String clientId = "CO2Light-";
    clientId += String(random(0xffff), HEX);
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
    if (dash.data.Calibration) {
      dash.data.Calibration = false;
      seconds = 0;
      calibrationStarted = true;          // starting calibration mode
      Serial.print(F("[DEBUG] calibrationStarted "));
      Serial.println(calibrationStarted);
      myMHZ19.autoCalibration(false);     // make sure auto calibration is off
      Serial.print("ABC Status: "); myMHZ19.getABC() ? Serial.println("ON") :  Serial.println("OFF");  // now print it's status
      Serial.println("Waiting 20 minutes to stabalise...");
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

  //Onboard LED port Direction output
  pinMode(BUILTIN_LED,OUTPUT); 

  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println(F("SSD1306 allocation failed"));
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

  // NeoPixelBus TEST
  //animations.UpdateAnimations();
  if (calibrationStarted == true) {
    animations.UpdateAnimations();
    strip.Show();
  }
  
  

//tasks
    if (taskA.previous == 0 || (millis() - taskA.previous > taskA.rate)) {
        taskA.previous = millis();

      seconds++;
      Serial.print(F("[INFO] seconds "));
      Serial.println(seconds);

      Serial.print(F("[DEBUG] calibrationStarted = "));
      Serial.println(calibrationStarted);

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
      } else {
          intensity = configManager.data.matrixIntensity;
          strip.SetBrightness(intensity);
        }
      
        if (mqttPublishTime <= configManager.data.mqtt_interval) {
          mqttPublishTime++;
        } else {
          PublishMQTT();
          mqttPublishTime = 0;
        }
      } else {
    calibrationActive();
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

        // reboot pending
        if (reboot) {
          reboot = false;
          ESP.reset();
        }

       // Reboot from dashboard 
    if (dash.data.Reboot) {
        dash.data.Reboot = false;
        dash.loop();
        reboot = true;
    }

  // Display
  // only if Calibraion is not running
  if (calibrationStarted == false) {
    if ( (seconds > 11) && (seconds <= 14) ) {
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
    } else {
      printDisplay(); 
    }

   if ( seconds == 15 ) {
    seconds = 0;     
  // MH-Z19B reading
        // note: getCO2() default is command "CO2 Unlimited". This returns the correct CO2 reading even 
        // if below background CO2 levels or above range (useful to validate sensor). You can use the 
        // usual documented command with getCO2(false) 
        Serial.println(F("read CO2 value"));
  
        dash.data.CO2 = myMHZ19.getCO2();                             // Request CO2 (as ppm)
        dash.data.Temperature = myMHZ19.getTemperature();             // Request Temperature (as Celsius)
        
               if(myMHZ19.errorCode == RESULT_OK)              // RESULT_OK is an alis for 1. Either can be used to confirm the response was OK.
        {
            Serial.print(F("co2 Value successfully Recieved: "));
            Serial.println(dash.data.CO2);
            Serial.print(F("Response Code: "));
            Serial.println(myMHZ19.errorCode);          // Get the Error Code value

            
        } else {
            Serial.println(F("Failed to recieve CO2 value - Error"));
            Serial.print(F("Response Code: "));
            Serial.println(myMHZ19.errorCode);          // Get the Error Code value
            dash.data.CO2 = 0;     
        }  
    }     

  }

  if (dash.data.Calibration) {
    calibrationActive();
  }
}

