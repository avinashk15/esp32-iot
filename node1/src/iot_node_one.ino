#include "DHTesp.h" 
#include "PirSensor.h"

#include <WiFi.h>
#include <MQTT.h>


#define LED_RED_PIN     22
#define LED_YELLOW_PIN  23

// PIR Initilization
#define PUBLISH_INTERVAL 5
#define MOTION_SENSOR_PIN 33
PirSensor motion = PirSensor(MOTION_SENSOR_PIN, 2, false, false);
long lastRun = millis();

///////

/////// MQ Inilization //
#define MQ_SENSOR_PIN 34
TaskHandle_t smokeDetectTaskHandle = NULL;
 /////


// MQTT Inililization //
const char ssid[] = "ssid";
const char pass[] = "pass";

WiFiClient netClient;
MQTTClient client;

unsigned long lastMillisMotionDetector = 0;

const char* mqtt_server_ip = "100.64.00.00" ;

// Topics for sensor data
const char* temperature_topic = "/building1/room1/temp";
const char* humidity_topic = "/building1/room1/humid";
const char* mq_topic = "/building1/room1/mq";
const char* motion_detect_topic = "/building1/room1/pir1";

// Topics for actuators
const char* smoke_alarm_topic = "/building1/room1/smokealarm";
const char* light_topic = "/building1/room1/light";
/////////////////

// DHT Inililization ///
DHTesp dht;

/** Task handle for DHT read task */
TaskHandle_t tempTaskHandle = NULL;
/** Comfort profile */
ComfortState cf;
/** DHT11 data pin */
int dhtPin = 32;

////////




void setup()
{
  Serial.begin(9600);
  // Start Wifi
  init_wifi();
  // MQTT
  init_mqtt();
  // Smoke (MQ) sensor
  init_smoke_detector();
  // DHT
  init_dht_sensor();
  // PIR 
  motion.begin();
  //
  pinMode(LED_RED_PIN,OUTPUT);
  pinMode(LED_YELLOW_PIN,OUTPUT);

}

void loop() {
    start_mqtt();
    detect_motion();
    delay(1);
}

bool init_dht_sensor() {
  // Initialize temperature sensor
  dht.setup(dhtPin, DHTesp::DHT11);
  Serial.println("DHT initiated");

  // Start task to get temperature
  xTaskCreatePinnedToCore(
      temperature_task,                       /* Function to implement the task */
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
 * Task to reads temperature and humidity from DHT11 sensor
 * @param pvParameters
 *    pointer to task parameters
 */
void temperature_task(void *pvParameters) {
  Serial.println("temperature_task loop started");

  const TickType_t xDelay = 10000 / portTICK_PERIOD_MS;

  while (1) // tempTask loop
  {
      // Get temperature values
    get_temperature();
    vTaskDelay( xDelay );
  }
}

/**
 * get_temperature
 * Reads temperature from DHT11 sensor
 * @return bool
 *    true if temperature could be aquired
 *    false if aquisition failed
*/
bool get_temperature() {
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
  int buffer_size = 100;
  char message[buffer_size];
  snprintf(message, buffer_size, "%f,%f,%f,%f,%s",newValues.temperature,newValues.humidity,heatIndex,dewPoint,comfortStatus);
  publish_data_mqtt(temperature_topic,message);
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

  client.subscribe(smoke_alarm_topic);
  client.subscribe(light_topic);

  // client.unsubscribe("/hello");
}

void messageReceived(String &topic, String &payload) {
  Serial.println("incoming: " + topic + " - " + payload);

  // Note: Do not use the client in the callback to publish, subscribe or
  // unsubscribe as it may cause deadlocks when other things arrive while
  // sending and receiving acknowledgments. Instead, change a global variable,
  // or push to a queue and handle it in the loop after calling `client.loop()`.
}


void detect_motion(){
  int motionStateChange = motion.sampleValue();
  if (motionStateChange >= 0) {
      Serial.print("Motion Detection : ");
      Serial.println(motionStateChange);
      digitalWrite(LED_YELLOW_PIN,motionStateChange);

      // Publish data at every 100 milisecond 
      if (millis() - lastMillisMotionDetector > 100) {
        lastMillisMotionDetector = millis();
        publish_data_mqtt(motion_detect_topic,motionStateChange) ;
      }
  }
}

bool init_smoke_detector() {
  pinMode(MQ_SENSOR_PIN, INPUT);

  // Start task to get temperature
  xTaskCreatePinnedToCore(
      detect_smoke,                       /* Function to implement the task */
      "detectSmokeTask ",                    /* Name of the task */
      2000,                           /* Stack size in words */
      NULL,                           /* Task input parameter */
      5,                              /* Priority of the task */
      &smokeDetectTaskHandle,                /* Task handle. */
      1);                             /* Core where the task should run */

  if (smokeDetectTaskHandle == NULL) {
    Serial.println("Failed to start task for detectSmoke");
    return false;
  } 
  return true;
}

// RTOS task to detect smoke..Read value at every 500 ms and publish data to broker.
void detect_smoke(void *pvParameters){

  const TickType_t xDelay = 500 / portTICK_PERIOD_MS;

  while (1)
  {
    int analogValue = analogRead(MQ_SENSOR_PIN);
    //int digital_value = analogRead(MQ_SENSOR_PIN);

    Serial.print("MQ value : ");
    Serial.println(analogValue);
    publish_data_mqtt(mq_topic,analogValue);
    vTaskDelay(xDelay);
  }
  
}

// Init mqtt client and connect to broker with IP address and TCP port 1883.
void init_mqtt(){

  client.begin(mqtt_server_ip, netClient);
  client.onMessage(messageReceived);

  connect();
}


void start_mqtt(){
  client.loop();
  delay(10); 

  if (!client.connected()) {
    connect();
  }

}

void publish_data_mqtt(const char topic[], int payload){
   // publish a message
    int buffer_size = 100;
    char message[buffer_size];
    snprintf(message, buffer_size, "%d" , payload);
    client.publish(topic, message);
}

void publish_data_mqtt(const char topic[], const char message[]){
   // publish a message
    client.publish(topic, message);
}


// Wifi initilization..Init in STA mode
void init_wifi() {
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, pass);
  Serial.print("Connecting to WiFi ..");
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print('.');
    delay(1000);
  }
  Serial.println(WiFi.localIP());
}