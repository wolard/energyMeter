#include <Arduino.h>

#define pushButton_pin   33
#define LED_pin   32
volatile int pulsesPerMin;
int pulsesPerHour;
unsigned long startMillis;  //some global variables available anywhere in the program
unsigned long currentMillis;
unsigned long pulseMillis;
const unsigned long period = 30000;
char str[8];
int pulsesPerHourArr[60];
#include <WiFi.h>
extern "C" {
	#include "freertos/FreeRTOS.h"
	#include "freertos/timers.h"
}
#include <AsyncMqttClient.h>

#define WIFI_SSID "koti"
#define WIFI_PASSWORD "kopo2008"

#define MQTT_HOST IPAddress(192, 168, 1, 201)
#define MQTT_PORT 1883

AsyncMqttClient mqttClient;
TimerHandle_t mqttReconnectTimer;
TimerHandle_t wifiReconnectTimer;

void connectToWifi() {
  Serial.println("Connecting to Wi-Fi...");
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
}

void connectToMqtt() {
  Serial.println("Connecting to MQTT...");
  mqttClient.connect();
}

void WiFiEvent(WiFiEvent_t event) {
    Serial.printf("[WiFi-event] event: %d\n", event);
    switch(event) {
    case SYSTEM_EVENT_STA_GOT_IP:
        Serial.println("WiFi connected");
        Serial.println("IP address: ");
        Serial.println(WiFi.localIP());
        connectToMqtt();
        break;
    case SYSTEM_EVENT_STA_DISCONNECTED:
        Serial.println("WiFi lost connection");
        xTimerStop(mqttReconnectTimer, 0); // ensure we don't reconnect to MQTT while reconnecting to Wi-Fi
        xTimerStart(wifiReconnectTimer, 0);
        break;
    }
}

void onMqttConnect(bool sessionPresent) {
  Serial.println("Connected to MQTT.");
  Serial.print("Session present: ");
  Serial.println(sessionPresent);
  uint16_t packetIdSub = mqttClient.subscribe("test/lol", 2);
  Serial.print("Subscribing at QoS 2, packetId: ");
  Serial.println(packetIdSub);
  mqttClient.publish("test/lol", 0, true, "test 1");
  Serial.println("Publishing at QoS 0");
  uint16_t packetIdPub1 = mqttClient.publish("test/lol", 1, true, "test 2");
  Serial.print("Publishing at QoS 1, packetId: ");
  Serial.println(packetIdPub1);
  uint16_t packetIdPub2 = mqttClient.publish("test/lol", 2, true, "test 3");
  Serial.print("Publishing at QoS 2, packetId: ");
  Serial.println(packetIdPub2);
}

void onMqttDisconnect(AsyncMqttClientDisconnectReason reason) {
  Serial.println("Disconnected from MQTT.");

  if (WiFi.isConnected()) {
    xTimerStart(mqttReconnectTimer, 0);
  }
}

void onMqttSubscribe(uint16_t packetId, uint8_t qos) {
  Serial.println("Subscribe acknowledged.");
  Serial.print("  packetId: ");
  Serial.println(packetId);
  Serial.print("  qos: ");
  Serial.println(qos);
}

void onMqttUnsubscribe(uint16_t packetId) {
  Serial.println("Unsubscribe acknowledged.");
  Serial.print("  packetId: ");
  Serial.println(packetId);
}

void onMqttMessage(char* topic, char* payload, AsyncMqttClientMessageProperties properties, size_t len, size_t index, size_t total) {
  Serial.println("Publish received.");
  Serial.print("  topic: ");
  Serial.println(topic);
  Serial.print("  qos: ");
  Serial.println(properties.qos);
  Serial.print("  dup: ");
  Serial.println(properties.dup);
  Serial.print("  retain: ");
  Serial.println(properties.retain);
  Serial.print("  len: ");
  Serial.println(len);
  Serial.print("  index: ");
  Serial.println(index);
  Serial.print("  total: ");
  Serial.println(total);
}

void onMqttPublish(uint16_t packetId) {
  Serial.println("Publish acknowledged.");
  Serial.print("  packetId: ");
  Serial.println(packetId);
}

void IRAM_ATTR toggleLED()
{
  if(millis()-pulseMillis>200)
{
 pulseMillis=millis();
  pulsesPerMin++;
} 
}
void setup()
{
  pulseMillis=millis();
  startMillis = millis(); 
  pulsesPerHour=0;
  pulsesPerMin=0;
  Serial.begin(9600);  
  pinMode(pushButton_pin, INPUT);
  attachInterrupt(pushButton_pin, toggleLED, RISING);
   mqttReconnectTimer = xTimerCreate("mqttTimer", pdMS_TO_TICKS(2000), pdFALSE, (void*)0, reinterpret_cast<TimerCallbackFunction_t>(connectToMqtt));
  wifiReconnectTimer = xTimerCreate("wifiTimer", pdMS_TO_TICKS(2000), pdFALSE, (void*)0, reinterpret_cast<TimerCallbackFunction_t>(connectToWifi));

  WiFi.onEvent(WiFiEvent);

  mqttClient.onConnect(onMqttConnect);
  mqttClient.onDisconnect(onMqttDisconnect);
  mqttClient.onSubscribe(onMqttSubscribe);
  mqttClient.onUnsubscribe(onMqttUnsubscribe);
  mqttClient.onMessage(onMqttMessage);
  mqttClient.onPublish(onMqttPublish);
  mqttClient.setServer(MQTT_HOST, MQTT_PORT);

  connectToWifi();
   for (int i = 0; i < 59; i++) {
    
      pulsesPerHourArr[i]=0;
    
} 
}
void loop()
{
    currentMillis = millis();  //get the current "time" (actually the number of milliseconds since the program started)
  if (currentMillis - startMillis >= 60000)  //test whether the period has elapsed
  {
  


  for (int i = 59; i > 0; i--) {
   // Serial.print(pulsesPerHourArr[i]);
    
        pulsesPerHour+=pulsesPerHourArr[i];
     pulsesPerHourArr[i] = pulsesPerHourArr[i - 1];
  
    }
    pulsesPerHourArr[0]=pulsesPerMin;
    pulsesPerHour+=pulsesPerMin;
  
  int tempulsespermin=pulsesPerMin;
  int tempulsesperhour=pulsesPerHour;
  pulsesPerMin=0;
pulsesPerHour=0;
itoa( tempulsespermin, str, 10 );
    mqttClient.publish("/energymeter/min", 0, true, str);
    itoa( tempulsesperhour, str, 10 );
    mqttClient.publish("/energymeter/hour", 0, true, str);
 /*
 Serial.println();
 Serial.println("pulsesPerHour");
 Serial.println(pulsesPerHour);
 Serial.println("pulsesPerMin");
Serial.println(pulsesPerMin);
*/
tempulsespermin=0;
tempulsesperhour=0;

    startMillis = currentMillis;  //IMPORTANT to save the start time of the current LED state.
  }

  
}