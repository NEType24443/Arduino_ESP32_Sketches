#include "BLEDevice.h"
#include <WiFi.h>
#include <PubSubClient.h>
#include <Streaming.h>
static BLEAddress *pServerAddress = NULL;
#define LED 2
BLEScan* pBLEScan;
BLEClient*  pClient;
bool deviceFound = false;

String knownAddresses[] = { "73:c8:74:44:53:6b", "  :  :  :  :  :  "};

int RSSI_val = 90;

const char* ssid = "D-Link";
const char* password = "tp181100";
const char* mqtt_server = "192.168.1.11";  // change for your own MQTT broker address
#define TOPIC "metrics/exchange"  // Change for your own topic
#define PAYLOAD "1"    // change for your own payload

unsigned long entry;

WiFiClient espClient;
PubSubClient MQTTclient(espClient);

void MQTTcallback(char* topic, byte* payload, unsigned int length) {  //  Callback for response from a device listening
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  for (int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
  }
  Serial.println();
}

static void notifyCallback(
  BLERemoteCharacteristic* pBLERemoteCharacteristic,
  uint8_t* pData,
  size_t length,
  bool isNotify) {
  Serial.print("Notify callback for characteristic ");
  Serial.print(pBLERemoteCharacteristic->getUUID().toString().c_str());
  Serial.print(" of data length ");
  Serial.println(length);
}

class MyAdvertisedDeviceCallbacks: public BLEAdvertisedDeviceCallbacks {
    void onResult(BLEAdvertisedDevice advertisedDevice) {
      Serial.print("BLE Advertised Device found: ");
      Serial.println(advertisedDevice.toString().c_str());  //  Prints -> Name: AdvertisedDeviceName, Address: AdvertisedDeviceAddress
      pServerAddress = new BLEAddress(advertisedDevice.getAddress());

      bool known = false;
      for (int i = 0; i < (sizeof(knownAddresses) / sizeof(knownAddresses[0])); i++) {
        if (strcmp(pServerAddress->toString().c_str(), knownAddresses[i].c_str()) == 0) known = true;
      }
      if (known) {
        Serial.print("Device found: ");
        Serial.println(advertisedDevice.getRSSI());
        if (advertisedDevice.getRSSI() > -RSSI_val) deviceFound = true;
        else deviceFound = false;
        Serial.println(pServerAddress->toString().c_str());
        advertisedDevice.getScan()->stop();
      }
    }
}; // MyAdvertisedDeviceCallbacks

void sendMessage() {
  btStop();
  delay(10);
  // We start by connecting to a WiFi network
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  entry = millis();
  while (WiFi.status() != WL_CONNECTED) {
    if (millis() - entry >= 15000) esp_restart();
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.print("WiFi connected, IP address: ");
  Serial.println(WiFi.localIP());
  MQTTclient.setServer(mqtt_server, 1883);
  MQTTclient.setCallback(MQTTcallback);
  Serial.println("Connect to MQTT server...");
  while (!MQTTclient.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (MQTTclient.connect("ESP32Client", "admin", "admin")) {
      Serial.println("connected");
      MQTTclient.publish(TOPIC, PAYLOAD);
      //MQTTclient.subscribe(TOPIC);
    } else {
      Serial.print("failed, rc=");
      Serial.print(MQTTclient.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(1000);
    }
  }
  for (int i = 0; i > 50; i++) {  // Wait for 3 second for a response
    MQTTclient.loop();
    delay(100);
    Serial<<i<<endl;
  }
  MQTTclient.disconnect();
  delay(100);
  WiFi.mode(WIFI_OFF);
  btStart();
}


void setup() {
  Serial.begin(9600);
  Serial.println("Starting Arduino BLE Client application...");
  pinMode(LED, OUTPUT);
  digitalWrite(LED, HIGH);

  BLEDevice::init("ESP32");

  pClient  = BLEDevice::createClient();
  Serial.println(" - Created client");
  pBLEScan = BLEDevice::getScan();
  pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
  pBLEScan->setActiveScan(true);
}

void loop() {

  Serial.println();
  Serial.println("BLE Scan restarted.....");
  deviceFound = false;
  BLEScanResults scanResults = pBLEScan->start(30);
  if (deviceFound) {
    Serial.println("on");
    digitalWrite(LED, LOW);

    sendMessage();
    Serial.println("Waiting for 5 seconds");
    delay(5000);

  }
  else {
    Serial.println("off");
    digitalWrite(LED, HIGH);
  }
} // End of loop
