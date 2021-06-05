#include "DHTesp.h" 
#include "PirSensor.h"

// PIR Initilization
#define PUBLISH_INTERVAL 5
#define MOTION_SENSOR_PIN 33
PirSensor motion = PirSensor(MOTION_SENSOR_PIN, 2, false, false);
long lastRun = millis();

///////

/////// MQ Inilization //
#define MQ_SENSOR_PIN 34
 /////


// MQTT Inililization //

#include <WiFi.h>
#include <MQTT.h>

const char ssid[] = "ssid";
const char pass[] = "pass";

WiFiClient netClient;
MQTTClient client;

unsigned long lastMillis = 0;

const char* mqtt_server_ip = "100.64.00.00" ;


/////////////////

// DHT Inililization ///
DHTesp dht;

void tempTask(void *pvParameters);
bool getTemperature();
void triggerGetTemp();

/** Task handle for the light value read task */
TaskHandle_t tempTaskHandle = NULL;
/** Comfort profile */
ComfortState cf;
/** Flag if task should run */
bool tasksEnabled = false;
/** Pin number for DHT11 data pin */
int dhtPin = 32;

///////

//
bool initDhtSensor() {
  byte resultValue = 0;
  // Initialize temperature sensor
  dht.setup(dhtPin, DHTesp::DHT11);
  Serial.println("DHT initiated");

  // Start task to get temperature
  xTaskCreatePinnedToCore(
      tempTask,                       /* Function to implement the task */
      "tempTask ",                    /* Name of the task */
      4000,                           /* Stack size in words */
      NULL,                           /* Task input parameter */
      5,                              /* Priority of the task */
      &tempTaskHandle,                /* Task handle. */
      1);                             /* Core where the task should run */

  if (tempTaskHandle == NULL) {
    Serial.println("Failed to start task for temperature update");
    return false;
  } 
  return true;
}



/**
 * Task to reads temperature from DHT11 sensor
 * @param pvParameters
 *    pointer to task parameters
 */
void tempTask(void *pvParameters) {
  Serial.println("tempTask loop started");

  const TickType_t xDelay = 10000 / portTICK_PERIOD_MS;

  while (1) // tempTask loop
  {
      // Get temperature values
    getTemperature();
    vTaskDelay( xDelay );
  }
}

/**
 * getTemperature
 * Reads temperature from DHT11 sensor
 * @return bool
 *    true if temperature could be aquired
 *    false if aquisition failed
*/
bool getTemperature() {
  // Reading temperature and humidity 
  TempAndHumidity newValues = dht.getTempAndHumidity();
  // Check if it is ready to read 
  if (dht.getStatus() != 0) {
    Serial.println("DHT11 error status: " + String(dht.getStatusString()));
    return false;
  }

  float heatIndex = dht.computeHeatIndex(newValues.temperature, newValues.humidity);
  float dewPoint = dht.computeDewPoint(newValues.temperature, newValues.humidity);
  float cr = dht.getComfortRatio(cf, newValues.temperature, newValues.humidity);

  String comfortStatus;
  switch(cf) {
    case Comfort_OK:
      comfortStatus = "Comfort_OK";
      break;
    case Comfort_TooHot:
      comfortStatus = "Comfort_TooHot";
      break;
    case Comfort_TooCold:
      comfortStatus = "Comfort_TooCold";
      break;
    case Comfort_TooDry:
      comfortStatus = "Comfort_TooDry";
      break;
    case Comfort_TooHumid:
      comfortStatus = "Comfort_TooHumid";
      break;
    case Comfort_HotAndHumid:
      comfortStatus = "Comfort_HotAndHumid";
      break;
    case Comfort_HotAndDry:
      comfortStatus = "Comfort_HotAndDry";
      break;
    case Comfort_ColdAndHumid:
      comfortStatus = "Comfort_ColdAndHumid";
      break;
    case Comfort_ColdAndDry:
      comfortStatus = "Comfort_ColdAndDry";
      break;
    default:
      comfortStatus = "Unknown:";
      break;
  };

  Serial.println(" T:" + String(newValues.temperature) + " H:" + String(newValues.humidity) + " I:" + String(heatIndex) + " D:" + String(dewPoint) + " " + comfortStatus);
  return true;
}

////// MQTT ///
void connect() {
  Serial.print("checking wifi...");
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(1000);
  }

  Serial.print("\nconnecting...");
  while (!client.connect("sciot", "public", "public")) {
    Serial.print(".");
    delay(1000);
  }

  Serial.println("\nconnected!");

  client.subscribe("/hello");
  // client.unsubscribe("/hello");
}

void messageReceived(String &topic, String &payload) {
  Serial.println("incoming: " + topic + " - " + payload);

  // Note: Do not use the client in the callback to publish, subscribe or
  // unsubscribe as it may cause deadlocks when other things arrive while
  // sending and receiving acknowledgments. Instead, change a global variable,
  // or push to a queue and handle it in the loop after calling `client.loop()`.
}


//////////////

void setup()
{
  Serial.begin(9600);
  initWiFi();
  // MQTT
  init_mqtt();
  // DHT
  initDhtSensor();
  // PIR 
  motion.begin();
  // MQ
  pinMode(MQ_SENSOR_PIN, INPUT);

}

void loop() {
    detect_motion();
    delay(100);
    detect_smoke();
    delay(1000);
    start_mqtt();
}


void detect_motion(){
  int motionStateChange = motion.sampleValue();
  if (motionStateChange >= 0) {
      Serial.print("Motion Detection : ");
      Serial.println(motionStateChange);
  }
}

void detect_smoke(){
  int analogValue = analogRead(MQ_SENSOR_PIN);
  //int digital_value = analogRead(MQ_SENSOR_PIN);

  Serial.print("MQ value: ");
  Serial.println(analogValue);
}

void init_mqtt(){

  client.begin(mqtt_server_ip, netClient);
  client.onMessage(messageReceived);

  connect();
}


void start_mqtt(){
  client.loop();
  delay(10);  // <- fixes some issues with WiFi stability

  if (!client.connected()) {
    connect();
  }

  // publish a message roughly every second.
  if (millis() - lastMillis > 1000) {
    lastMillis = millis();
    client.publish("/test", "world");
  }
}

void initWiFi() {
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, pass);
  Serial.print("Connecting to WiFi ..");
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print('.');
    delay(1000);
  }
  Serial.println(WiFi.localIP());
}