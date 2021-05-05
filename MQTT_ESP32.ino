/* 
 *******************************************
 *           Introduction                  *
 *******************************************
 *
 *  In this code you will see the use of MQTT protocol being used as a means of data transfer for temperature readings.
 *  The ESP32 will connect to the WiFi, then to the MQTT broker that is on the same WiFi. Once connection is 
 *  established it will then Publish the temperature readings with the topic attached to it so the Broker can sort out
 *  its data from other ESP32 microcontrollers.  
 *  
 *  Author(s): Sergio Castro, Steven Pursak, Amadou Diop, Yousef Gholom, Anthony Paskert
 *  
 *  Comments: There are a few options that need to be changed for the users own personal equipment. Those are the following:
 *            - In the Macros section the WiFi SSID needs to match the WiFi's name, same for the WiFi Password. 
 *            
 *            - In the "void connectToWifi()" under Function Protoypes, if you have a WiFi that has no password, then comment out
 *              the instruction that has WIFI_PASSWORD and uncomment the one below it that has Null in its place. This is for 
 *              that exact purpose. 
 *              
 *            - The IP Address needs to match that of the MQTT IP Address (In this case the Rasp Pi's IP address). 
 *            
 *            - The topic of the MQTT data needs to be update for each model, in this case the topic is "esp32/model-a/Temperature" 
 *              for each model we change the model-a to model-b,c,d... however the entire topic can be changed so long as it matches
 *              what is needed on the broker side, and it is also case sensitive.   
 *              
 * Contributions & Acknowledgements:              
 * This project takes insperation from last years Safran Rotor Temperature group as well as the following link from a similar concept 
 * published by Rui Santos
 * https://RandomNerdTutorials.com/esp32-mqtt-publish-ds18b20-temperature-arduino/
 *              
 */

/*
 *******************************************
 *       Header File Inclusions            *
 *******************************************
 */
#include <WiFi.h>
#include <AsyncMqttClient.h>
#include <driver/adc.h>
#include "esp32-hal-cpu.h"
extern "C" 
{
  #include "freertos/FreeRTOS.h"
  #include "freertos/timers.h"
}
/*
  *******************************************
  *              Macros                     *
  *******************************************
 */
#define WIFI_SSID "WiFi" // Place your Wifi name in between the quotations
#define WIFI_PASSWORD "Password" // Place your Wifi Password in between the quotations 
#define MQTT_HOST IPAddress(192, 168, 39, 183) // Raspberry Pi Mosquitto MQTT Broker. Need to adjust per different wifi networks
//#define MQTT_HOST "example.com" // For a cloud MQTT broker, type the domain name
#define MQTT_PORT 1883
#define MQTT_PUB_TEMP "esp32/model-a/Temperature" // Temperature MQTT Topic. Model a is for the first test model, will change to b and c for the remaining two models

/*
  *******************************************
  *        Variable declarations            *
  *******************************************
 */
const int analogPin = 34; // GPIO where the DS18B20 is connected to                    
static float Temp; // Temperature, everything after Temp is added in from previous codes
float Vout; // Voltage after adc
float SenVal; // Analog Sensor value
unsigned long previousMillis = 0; // Stores last time Temperature was published
const long interval = 1000; // Interval at which to publish sensor readings (Currently set to 1 second intervals)

/*
  *******************************************
  *        Function Prototypes              *
  *******************************************
 */
float getTemp(); // ADC & Temp logic
void connectToWifi(); // 

AsyncMqttClient mqttClient;
TimerHandle_t mqttReconnectTimer;
TimerHandle_t wifiReconnectTimer;

void setup() 
{
  Serial.begin(115200);
  Serial.println();
  Serial.println();

  mqttReconnectTimer = xTimerCreate("mqttTimer", pdMS_TO_TICKS(2000), pdFALSE, (void*)0, reinterpret_cast<TimerCallbackFunction_t>(connectToMqtt));
  wifiReconnectTimer = xTimerCreate("wifiTimer", pdMS_TO_TICKS(2000), pdFALSE, (void*)0, reinterpret_cast<TimerCallbackFunction_t>(connectToWifi));

  WiFi.onEvent(WiFiEvent);

  mqttClient.onConnect(onMqttConnect);
  mqttClient.onDisconnect(onMqttDisconnect);
  mqttClient.onPublish(onMqttPublish);
  mqttClient.setServer(MQTT_HOST, MQTT_PORT);
  // If your broker requires authentication (username and password), set them below
  //mqttClient.setCredentials("REPlACE_WITH_YOUR_USER", "REPLACE_WITH_YOUR_PASSWORD");
  connectToWifi();
}

void loop() 
{
  unsigned long currentMillis = millis();
  // Every X number of seconds (interval = 10 seconds) it publishes a new MQTT message
  if (currentMillis - previousMillis >= interval)
  {
    previousMillis = currentMillis; // Save the last time a new reading was published
    // New Temperature readings
    //sensors.requestTemperatures(); // I believe that in this we can add the getting Temperature process in order to get the Temperature. 
    getTemp(); //proposed solution to line above.
    // Temp1 = getTemp(); //sensors.getTempFByIndex(0);
    
    // Publish an MQTT message on topic esp32/model-a/Temperature
    uint16_t packetIdPub1 = mqttClient.publish(MQTT_PUB_TEMP, 1, true, String(Temp).c_str());                            
    Serial.printf("Publishing on topic %s at QoS 1, packetId: ", MQTT_PUB_TEMP);
    Serial.println(packetIdPub1);
    Serial.printf("Message: %.2f /n", getTemp()); 
  }
}

void connectToWifi() 
{
  Serial.println("Connecting to Wi-Fi...");
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD); // Use when there is a WiFi Password i.e home WiFi
  //WiFi.begin(WIFI_SSID, NULL); // Use when there is no password i.e csuguest
}

void connectToMqtt() 
{
  Serial.println("Connecting to MQTT...");
  mqttClient.connect();
}

void WiFiEvent(WiFiEvent_t event) 
{
  Serial.printf("[WiFi-event] event: %d\n", event);
  switch(event) 
  {
    case SYSTEM_EVENT_STA_GOT_IP:
      Serial.println("WiFi connected");
      Serial.println("IP address: ");
      Serial.println(WiFi.localIP());
      connectToMqtt();
      break;
    case SYSTEM_EVENT_STA_DISCONNECTED:
      Serial.println("WiFi lost connection");
      xTimerStop(mqttReconnectTimer, 0); // Ensure we don't reconnect to MQTT while reconnecting to Wi-Fi
      xTimerStart(wifiReconnectTimer, 0);
      break;
  }
}

void onMqttConnect(bool sessionPresent) 
{
  Serial.println("Connected to MQTT.");
  Serial.print("Session present: ");
  Serial.println(sessionPresent);
}

void onMqttDisconnect(AsyncMqttClientDisconnectReason reason) 
{
  Serial.println("Disconnected from MQTT.");
  if (WiFi.isConnected()) 
  {
    xTimerStart(mqttReconnectTimer, 0);
  }
}

void onMqttPublish(uint16_t packetId) 
{
  Serial.println("Publish acknowledged.");
  Serial.print("  packetId: ");
  Serial.println(packetId);
}

float getTemp()
{
  float SenVal, Vout;
  SenVal = analogRead(analogPin); // Analog value from AD8495 output pin
  Vout = (SenVal *3.33)/4095; // Conversion analog to digital for the Temperature read voltage 
  Temp = (209.52*Vout)-228.799;// Calibrated conversion of digital voltage level to Temperature 
  Serial.println (Vout);
  Serial.println (Temp);
  return Temp;
}
