/************************************************************************************************************************
 * CO2light project
 * You will find all infos at www.kidbuild.de
 * 
 * This code provides a CO2 traffic light
 * It should support shools, DIY people or other groups to get informations about the air quality in rooms.
 * The traffic light shows you the time to get fresh air into the rooms.
 * There are four colours from the RGB LED´s who tell you the status of the air.
 * 
 * Changelog
 * 
 * ### Verison 0.4
 * (Eisbaeeer 20201222)   
 * - Only send MQTT if request is valid
 * 
 * ### Version 0.3
 * (Eisbaeeer 20201220)   
 * - Added temperature, humidity and ppm on webpage
 * 
 * ### Version 0.2 
 * (Eisbaeeer 20201220)   
 * - added WifiManager
 * - added Website for config brightness
 * - added HttpUpdateServer
 * - added MQTT
 * 
 * ### Version 0.1 
 * (Eisbaeeer 20201216)   
 * - initial version
 * 
 * Used 3rd party code or parts of that:
 * 
 * Rui Santos
 * https://randomnerdtutorials.com  
 * 
 * MH-Z19B library
 * https://github.com/WifWaf/MH-Z19
 *************************************************************************************************************************/

#include <FS.h>                    //this needs to be first, or it all crashes and burns...
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <Adafruit_NeoPixel.h>
#include "MHZ19.h"                                        
#include <SoftwareSerial.h>
#include <WiFiClient.h>
#include <ESP8266WebServer.h>
#include <ESP8266HTTPUpdateServer.h>
#include <WiFiManager.h>          // WifiManager 
#include "JsonStreamingParser.h"   // Json Streaming Parser  
#include <ArduinoJson.h>          // ArduinoJSON                 https://github.com/bblanchon/ArduinoJson
#include "index.h"
#include <ESPStringTemplate.h>

char version[10] = "v0.4";

char matrixIntensity[5] = "100";
char htmlBuffer[4096];

const char* update_path = "/firmware";
const char* update_username = "admin";
const char* update_password = "admin";

ESP8266WebServer httpServer(80);
ESP8266HTTPUpdateServer httpUpdater;


// WS2812 definitions
#define PIN D6 // Data Pin
#define NUMPIXELS 27 // Number of LED´s
Adafruit_NeoPixel pixels(NUMPIXELS, PIN, NEO_GRB + NEO_KHZ800);

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

#define DHTPIN D5     // Pin14 Digital pin connected to the DHT sensor

// Uncomment the type of sensor in use:
//#define DHTTYPE    DHT11     // DHT 11
#define DHTTYPE    DHT22     // DHT 22 (AM2302)
//#define DHTTYPE    DHT21     // DHT 21 (AM2301)

float t = 0;    // Temperature value
float h = 0;    // Humidity value
int co2 = 400;  // Co2 value

// no more delay with interval 
unsigned long previousMillis = 0;
const long interval = 100;   // 
byte periode = 0;
int seconds = 5;                 // var for counting seconds

DHT dht(DHTPIN, DHTTYPE);

//flag for saving data
bool shouldSaveConfig = false;

//--- BEGIN MQTT ------------------------------------
#include <PubSubClient.h>
#define MQTT_KEEPALIVE 60;
boolean mqttconnected = false;

// MQTT definitions
void MqttCallback(char *topic, byte *payload, unsigned int length);
WiFiClient espclient;
PubSubClient mqttclient(espclient);
#define MQTT_ID "CO2light"
char buf[40];

// IP Adresse und Port des MQTT Servers
IPAddress mqttserver(192, 168, 1, 200);
const int mqttport = 1883;
// If user auth is req. please add this parameters!
#define mqttauth
char mqttuser[20] = "mqtt";
char mqttpass[20] = "mqtt"; 

// Declare subs
void Mqttpublish();
//--- END MQTT -------------------------------------

void setup() {
  Serial.begin(115200);

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

  //--- Begin Wifimanager ----------------------------------------------
  if (SPIFFS.begin()) {
    if (SPIFFS.exists("/config.json")) {
      //file exists, reading and loading
      File configFile = SPIFFS.open("/config.json", "r");
      if (configFile) {
        Serial.println("opened config file");
        size_t size = configFile.size();
        // Allocate a buffer to store contents of the file.
        std::unique_ptr<char[]> buf(new char[size]);

        //ACHTUNG FORMAT
        //infoReset();
        //
        //ACHTUNG FORMAT

        configFile.readBytes(buf.get(), size);
        DynamicJsonDocument json(4096);
        deserializeJson(json, buf.get());
        serializeJson(json,Serial);
       
        strcpy(matrixIntensity, json["matrixIntensity"]);
                
        JsonVariant jsonMatrixIntensity = json["matrixIntensity"];
        if (!jsonMatrixIntensity.isNull()) {
            strcpy(matrixIntensity, json["matrixIntensity"]);
        } 
                
      }
    } else {
      
    }
  } else {
    Serial.println("failed to mount FS");
  }
  
  WiFiManager wifiManager;
  

  // Requesting Instagram and Intensity for Display
  WiFiManagerParameter custom_intensity("Helligkeit", "Helligkeit 0-15", matrixIntensity, 5);
  
  // Add params to wifiManager
  WiFi.hostname("CO2light");
  wifiManager.setConnectTimeout(60);
  wifiManager.addParameter(&custom_intensity);
  
   //set config save notify callback
  wifiManager.setSaveConfigCallback(saveConfigCallback);
  wifiManager.autoConnect("CO2light");

  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());

  display.clearDisplay();
  display.setCursor(0,0);
  display.setTextSize(3);
  display.print("IP:");
  display.setCursor(0,30);
  display.setTextSize(1);
  display.print(WiFi.localIP());
  display.display(); 
    //--- END Wifimanager -----------------------------------------

  httpUpdater.setup(&httpServer, update_path, update_username, update_password);
  httpServer.begin();
  Serial.println("");
  Serial.printf("HTTPUpdateServer ready! Open http://%s.local%s in your browser and login with username '%s' and password '%s'\n", "CO2light", update_path, update_username, update_password);

  httpServer.on("/", handleRoot);
  httpServer.on("/reset", getReset);
  httpServer.on("/config", getConfig);

  //save the custom parameters to FS
  if (shouldSaveConfig) {
      saveConfig();
    //end save
  }

  dht.begin();

  // MH-Z19B construct
  mySerial.begin(BAUDRATE);                               // (Uno example) device to MH-Z19 serial start   
  myMHZ19.begin(mySerial);                                // *Serial(Stream) refence must be passed to library begin(). 
  myMHZ19.autoCalibration();                              // Turn auto calibration ON (OFF autoCalibration(false))

  // start MQTT client
  MqttConnect(mqttuser, mqttpass);  
}

void loop() {

    httpServer.handleClient();
    
    unsigned long currentMillis = millis();
    if (currentMillis - previousMillis >= interval) {
      previousMillis = currentMillis;
      periode++;
       
      if ( periode >= 10 ) {
      seconds++;
      periode = 0;  
      }    
    }
     
  if (co2 <= 650 ) {                              // green
    setLED(0,255,0);
  } 
  else if (( co2 > 650 ) && ( co2 < 850)) {       // yellow
    setLED(255,255,0);
  }
  else if (( co2 >= 850) && ( co2 < 1000)) {      // orange
    setLED(255,128,0);
  }
  else{
    setLED(255,0,0);                              // red
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
  }
   if ( seconds == 15 ) {
    seconds = 0;     
  // MH-Z19B reading
        /* note: getCO2() default is command "CO2 Unlimited". This returns the correct CO2 reading even 
        if below background CO2 levels or above range (useful to validate sensor). You can use the 
        usual documented command with getCO2(false) */

        co2 = myMHZ19.getCO2();                             // Request CO2 (as ppm)
        
               if(myMHZ19.errorCode == RESULT_OK)              // RESULT_OK is an alis for 1. Either can be used to confirm the response was OK.
        {
            Serial.print("co2 Value successfully Recieved: ");
            Serial.println(co2);
            Serial.print("Response Code: ");
            Serial.println(myMHZ19.errorCode);          // Get the Error Code value

            // MQTT publish
            Mqttpublish();
        }

        else 
        {
            Serial.println("Failed to recieve CO2 value - Error");
            Serial.print("Response Code: ");
            Serial.println(myMHZ19.errorCode);          // Get the Error Code value
            co2 = 0;     
        }  

  //read temperature and humidity
  t = dht.readTemperature();
  h = dht.readHumidity();
  
  if (isnan(h) || isnan(t)) {
    Serial.println("Failed to read from DHT sensor!");
  
  } else {
    Serial.print("Temperature: ");
    Serial.println(t);
    Serial.print("Humidity: ");
    Serial.println(h);
         
  }
    if ((!isnan(h)) || (!isnan(t)) || (!myMHZ19.errorCode == RESULT_OK )) {     
        // Printout to display
        printDisplay();    
    }
   
  }
}

void setLED(byte R, byte G, byte B) {
  for(int i=0; i<NUMPIXELS; i++) { // For each pixel...
    pixels.setPixelColor(i, pixels.Color(R, G, B));
    int intensity = atoi(matrixIntensity);
    pixels.setBrightness(intensity);
    pixels.show(); 
  }
}

void printDisplay(void) {
  // clear display
  display.clearDisplay();
  
  // display temperature
  display.setCursor(0,0);
  display.setTextSize(2);
  display.print(t,1);
  display.print(" ");
  display.setTextSize(1);
  display.cp437(true);
  display.write(167);
  display.setTextSize(2);
  display.print("C");
  
  // display humidity
  display.setCursor(0, 18);
  display.print(h,1);
  display.print(" %"); 

  // display co2
  display.setCursor(0, 36);
  display.setTextSize(4);
  if (co2 == 0 ) {
    display.print("----");
  } else {
  display.print(co2);
  }
  display.setTextSize(1);
  display.print(" ppm"); 
  
  display.display(); 
}

void saveConfig() {
    DynamicJsonDocument json(4096);
    json["matrixIntensity"] = matrixIntensity;
        
    File configFile = SPIFFS.open("/config.json", "w");
    
    if (!configFile) {
      Serial.println("failed to open config file for writing");
    }

    serializeJson(json, Serial);
    serializeJson(json, configFile); 
}

//callback notifying us of the need to save config
void saveConfigCallback () {
  Serial.println("Should save config");
  shouldSaveConfig = true;
}

void handleRoot() {
  //Serial.print("HTML Buffer:");
  //Serial.println(htmlBuffer);
  ESPStringTemplate webpage(htmlBuffer, sizeof(htmlBuffer));
    
  webpage.add_P(_PAGE_HEAD);
  webpage.add_P(_PAGE_START);
  
  TokenStringPair intensityPair[1]; 
  intensityPair[0].setPair("%INTENSITY%",matrixIntensity );

  dtostrf(t, 4, 1, buf); // Leave room for too large numbers!
  TokenStringPair tempPair[1];
  tempPair[0].setPair("%TEMPERATURE%", buf);
  webpage.add_P(_PAGE_CONFIG_TEMPERATURE, tempPair, 1);
  
  dtostrf(h, 6, 1, buf); 
  TokenStringPair humiPair[1];
  humiPair[0].setPair("%HUMIDITY%", buf);
  webpage.add_P(_PAGE_CONFIG_HUMIDITY, humiPair, 1);

  String str;
  str=String(co2);
  str.toCharArray(buf,5);
  TokenStringPair ppmPair[1];
  ppmPair[0].setPair("%CO2%", buf);
  webpage.add_P(_PAGE_CONFIG_CO2, ppmPair, 1);
  
  webpage.add_P(_PAGE_CONFIG_INTENSITY, intensityPair, 1);
  webpage.add_P(_PAGE_FOOTER);
  webpage.add_P(_PAGE_ACTIONS);

  TokenStringPair versionPair[1];
  versionPair[0].setPair("%VERSION%", version);
  webpage.add_P(_PAGE_SOURCE, versionPair, 1);

  httpServer.send(200, "text/html", htmlBuffer);
}

void redirectBack() {
  httpServer.sendHeader("Location", String("/"), true);
  httpServer.send ( 302, "text/plain", "");
}

void getConfig() {
  // Intensity
  String intensityString = httpServer.arg("intensity");
  String matrixIntensityString = intensityString;
  matrixIntensityString.toCharArray(matrixIntensity,40);

      saveConfig();
      redirectBack();      
  }

void getReset() {
  redirectBack();
  restartX();
}

void restartX() {
  display.clearDisplay();
  display.setCursor(0,0);
  display.setTextSize(3);
  display.print("reboot");
  display.display(); 
  delay(1000);
  ESP.reset();
}

void MqttConnect(char *user, char* pass) {

    mqttclient.setClient(espclient);
    mqttclient.setServer(mqttserver, mqttport);
    mqttclient.setCallback(MqttCallback);
    
    mqttconnected = mqttclient.connect(MQTT_ID, user, pass);
    if (mqttconnected) {
     //   mqttclient.subscribe("CO2light/Brightness");
    }
}

void MqttCallback(char *topic, byte *payload, unsigned int length) {
    char *payloadvalue;
    char *payloadkey;

    payload[length] = '\0';
    payloadkey = (char *)&payload[0];
}

void Mqttpublish(void) {
    if (mqttclient.connected()) {
        dtostrf(t, 5, 2, buf);
        mqttclient.publish("CO2light/Temperatur", buf);
        dtostrf(h, 5, 0, buf);
        mqttclient.publish("CO2light/Feuchtigkeit", buf);
        dtostrf(co2, 5, 0, buf);
        mqttclient.publish("CO2light/CO2", buf);
    } else {
        MqttConnect(mqttuser, mqttpass);  
    }
}
