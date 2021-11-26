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
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_NeoPixel.h>
#include "MHZ19.h"                                        
#include <SoftwareSerial.h>

// WS2812 definitions
#define PIN D6 // Data Pin
#define NUMPIXELS 27 // Number of LED´s
Adafruit_NeoPixel pixels(NUMPIXELS, PIN, NEO_GRB + NEO_KHZ800);
boolean blinking = false;
int intensity = 30;

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
int seconds = 0;                 // var for counting seconds

//--- BEGIN MQTT ------------------------------------
#include <PubSubClient.h>
#define MQTT_KEEPALIVE 60;
boolean mqttconnected = false;
int mqttReconnect;            // timeout for reconnecting MQTT Server
int mqttPublishTime;          // last publish time in seconds

// MQTT definitions
//void MqttCallback(char *topic, byte *payload, unsigned int length);
WiFiClient espclient;
PubSubClient MQTTclient(espclient);
#define MQTT_ID "CO2light"
char buf[40];
#define MSG_BUFFER_SIZE	(20)
char result[MSG_BUFFER_SIZE];


// Tasks
struct task
{    
    unsigned long rate;
    unsigned long previous;
};

task taskA = { .rate = 1000, .previous = 0 };
task taskB = { .rate = 200, .previous = 0 };

// Global definitions
#define LED 2  //On board LED
bool reboot = false;            // flag to reboot device


//Define subs
void reconnect(void);
void PublishMQTT(void);
void setLED(void);
void printDisplay(void);

void setLED(byte R, byte G, byte B) {
  for(int i=0; i<NUMPIXELS; i++) { // For each pixel...
    pixels.setPixelColor(i, pixels.Color(R, G, B));
    //intensity = atoi(configManager.data.matrixIntensity);
    //pixels.setBrightness(intensity);
    pixels.show(); 
  }
}

void saveCallback() {
    intensity = atoi(configManager.data.matrixIntensity);
    pixels.setBrightness(intensity); 
}

void setup() {
  Serial.begin(115200);

  LittleFS.begin();
  GUI.begin();
  configManager.begin();
  WiFiManager.begin(configManager.data.projectName);
  configManager.setConfigSaveCallback(saveCallback);
  timeSync.begin();
  dash.begin(500);

  // WiFi
  WiFi.hostname(configManager.data.wifi_hostname);
  WiFi.begin();

  //Onboard LED port Direction output
  pinMode(LED,OUTPUT); 

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

  pixels.begin(); // INITIALIZE NeoPixel strip object (REQUIRED)

  String VERSION = F("v.1.0");
    int str_len = VERSION.length() + 1;
    VERSION.toCharArray(dash.data.Version,str_len);

    MQTTclient.setServer(configManager.data.mqtt_server, configManager.data.mqtt_port);
    //MQTTclient.setCallback(MqttCallback);

  // MH-Z19B construct
  mySerial.begin(BAUDRATE);                               // (Uno example) device to MH-Z19 serial start   
  myMHZ19.begin(mySerial);                                // *Serial(Stream) refence must be passed to library begin(). 
  myMHZ19.autoCalibration();                              // Turn auto calibration ON (OFF autoCalibration(false))

}

void loop() {
  // framework things
  WiFiManager.loop();
  updater.loop();
  configManager.loop();
  dash.loop();
  MQTTclient.loop();

//tasks
    if (taskA.previous == 0 || (millis() - taskA.previous > taskA.rate))
    {
        taskA.previous = millis();
        int rssi = 0;
        rssi = WiFi.RSSI();
        sprintf(dash.data.Wifi_RSSI, "%ld", rssi) ;
        dash.data.WLAN_RSSI = WiFi.RSSI();

                 
        reconnect();
        mqttReconnect++;      

        if (mqttPublishTime <= configManager.data.mqtt_interval) {
          mqttPublishTime++;
        } else {
          PublishMQTT();
          mqttPublishTime = 0;

        }

        // reboot pending
        if (reboot) {
          reboot = false;
          ESP.reset();
        }

        seconds++;

  if (dash.data.CO2 <= configManager.data.boarderGreen ) {                              // green
    setLED(0,255,0);
    blinking = false;
  } 
  else if (( dash.data.CO2 > configManager.data.boarderGreen ) && ( dash.data.CO2 < configManager.data.boarderYellow)) {       // yellow
    setLED(255,255,0);
    blinking = false;
  }
  else if (( dash.data.CO2 >= configManager.data.boarderYellow) && ( dash.data.CO2 < configManager.data.boarderOrange)) {      // orange
    setLED(255,128,0);
    blinking = false;
  }
  else if (( dash.data.CO2 >= configManager.data.boarderOrange) && ( dash.data.CO2 < configManager.data.boarderRed)) {      // orange
    setLED(255,0,0);
    blinking = false;
  }
  else {
    setLED(255,0,0);                              // red
    blinking = true;
  }

    // red blinking
      if (blinking) {
        if (intensity > 0) {
          intensity = 0;
          pixels.setBrightness(intensity);
        } else {
          intensity = atoi(configManager.data.matrixIntensity);
          pixels.setBrightness(intensity);
        }
      } else {
          intensity = atoi(configManager.data.matrixIntensity);
          pixels.setBrightness(intensity);
        }

       // Reboot from dashboard 
    if (dash.data.Reboot) {
        dash.data.Reboot = false;
        dash.loop();
        reboot = true;
    }

  if ( (seconds > 11) && (seconds <= 14) ) {
    display.clearDisplay();
    display.setCursor(0,0);
    display.setTextSize(3);
    display.print("IP:");
    display.setCursor(0,30);
    display.setTextSize(1);
    display.print(WiFi.localIP());
    display.display();
  } else {
    printDisplay();
  }

   if ( seconds == 15 ) {
    seconds = 0;     
  // MH-Z19B reading
        /* note: getCO2() default is command "CO2 Unlimited". This returns the correct CO2 reading even 
        if below background CO2 levels or above range (useful to validate sensor). You can use the 
        usual documented command with getCO2(false) */
        Serial.println(F("read CO2 value"));

        dash.data.CO2 = myMHZ19.getCO2();                             // Request CO2 (as ppm)
        dash.data.Temperature = myMHZ19.getTemperature();             // Request Temperature (as Celsius)
        
               if(myMHZ19.errorCode == RESULT_OK)              // RESULT_OK is an alis for 1. Either can be used to confirm the response was OK.
        {
            Serial.print(F("co2 Value successfully Recieved: "));
            Serial.println(dash.data.CO2);
            Serial.print(F("Response Code: "));
            Serial.println(myMHZ19.errorCode);          // Get the Error Code value
        }

        else 
        {
            Serial.println(F("Failed to recieve CO2 value - Error"));
            Serial.print(F("Response Code: "));
            Serial.println(myMHZ19.errorCode);          // Get the Error Code value
            dash.data.CO2 = 0;     
        }  
 }

}
}

// SUBS

void printDisplay(void) {
  // clear display
  display.clearDisplay();
  // display temperature
  display.setCursor(0,0);
  display.setTextSize(4);
  display.print(dash.data.Temperature,1);
  display.print(" ");
  display.setTextSize(3);
  display.cp437(true);
  display.write(167);
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

    String topic = "CO2Light/";
           topic = topic + configManager.data.place;
           topic = topic +"/CO2";
      dtostrf(dash.data.CO2, 5, 0, result);
    MQTTclient.publish(topic.c_str(), result);

}

void reconnect(void) {
  if (mqttReconnect > 60) {

    mqttReconnect = 0;    // reset reconnect timeout
  // reconnect to MQTT Server
  if (!MQTTclient.connected()) {
    dash.data.MQTT_Connected = false;
    Serial.println("Attempting MQTT connection...");
    // Create a random client ID
    String clientId = "CO2light-";
    clientId += String(random(0xffff), HEX);
    //clientId += String(configManager.data.messure_place);
    // Attempt to connect
    if (MQTTclient.connect(clientId.c_str(),configManager.data.mqtt_user,configManager.data.mqtt_password)) {
      Serial.println("connected");
      dash.data.MQTT_Connected = true;
      // Once connected, publish an announcement...
      PublishMQTT();
      // ... and resubscribe
      // MQTTclient.subscribe("inTopic");
    } else {
      Serial.print("failed, rc=");
      Serial.print(MQTTclient.state());
      Serial.println(" try again in one minute");
     }
    }
   }
 
}