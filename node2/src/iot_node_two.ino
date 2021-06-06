#include <WiFi.h>
#include <MQTT.h>
#include <SPI.h>

#include "MFRC522.h"

/*
Vcc <-> 3V3 
RST (Reset) <-> D0
GND (Masse) <-> GND
MISO (Master Input Slave Output) <-> 19
MOSI (Master Output Slave Input) <-> 23
SCK (Serial Clock) <-> 18
SS/SDA (Slave select) <-> 5
*/
// RFID Initilization
//Constants
#define SS_PIN 5
#define RST_PIN 0

byte nuidPICC[4] = {0, 0, 0, 0};
MFRC522::MIFARE_Key key;
MFRC522 rfid = MFRC522(SS_PIN, RST_PIN);

//////////////////
// MQTT Inililization //
const char ssid[] = "ssid";
const char pass[] = "pass";

WiFiClient netClient;
MQTTClient client;

const char* mqtt_server_ip = "100.64.00.00" ;

// Topics for sensor data
const char* access_control_topic = "/building1/room1/rfid";

// Topics for actuators
const char* door_unlock_topic = "/building1/room1/door";
/////////////////

#define LED_PIN 22
void setup()
{
  Serial.begin(9600);
  // start Wifi
  init_wifi();
  // MQTT
  init_mqtt();
  // init mfrc522 
  init_rfid();

  //
  pinMode(LED_PIN,OUTPUT);
}

void loop() {
  start_mqtt();
  readRFID();
  delay(1);
}

void init_rfid(){
  //init rfid D8,D5,D6,D7
  SPI.begin();
  rfid.PCD_Init();

  Serial.print(F("Reader :"));
  rfid.PCD_DumpVersionToSerial();
}

void readRFID(void ) {
  ////Read RFID card

  for (byte i = 0; i < 6; i++) {
    key.keyByte[i] = 0xFF;
  }
  // Look for new 1 cards
  if ( ! rfid.PICC_IsNewCardPresent())
    return;

  // Verify if the NUID has been readed
  if (  !rfid.PICC_ReadCardSerial())
    return;

  // Store NUID into nuidPICC array
  for (byte i = 0; i < 4; i++) {
    nuidPICC[i] = rfid.uid.uidByte[i];
  }

  digitalWrite(LED_PIN,HIGH);
  char str[9] = "";

  array_to_string(nuidPICC, 4, str);

  publish_data_mqtt(access_control_topic,str);


  Serial.print(F("RFID In dec: "));
  printHex(rfid.uid.uidByte, rfid.uid.size);
  Serial.println();

  // Halt PICC
  rfid.PICC_HaltA();

  // Stop encryption on PCD
  rfid.PCD_StopCrypto1();

  digitalWrite(LED_PIN,LOW);

}


/**
   Helper routine to dump a byte array as hex values to Serial.
*/
void printHex(byte *buffer, byte bufferSize) {
  for (byte i = 0; i < bufferSize; i++) {
    Serial.print(buffer[i] < 0x10 ? " 0" : " ");
    Serial.print(buffer[i], HEX);
  }
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

  client.subscribe(door_unlock_topic);

}

void messageReceived(String &topic, String &payload) {
  Serial.println("incoming: " + topic + " - " + payload);

  // Note: Do not use the client in the callback to publish, subscribe or sending 
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
