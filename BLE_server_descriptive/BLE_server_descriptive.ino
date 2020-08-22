/*
    Based on Neil Kolban example for IDF: https://github.com/nkolban/esp32-snippets/blob/master/cpp_utils/tests/BLE%20Tests/SampleServer.cpp
    Ported to Arduino ESP32 by Evandro Copercini
    updates by chegewara
*/
//#include <Arduino.h>
#include <Streaming.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>

// See the following for generating UUIDs:
// https://www.uuidgenerator.net/

#define TEXT_SERVICE_UUID           "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define TEXT_1_CHARACTERISTIC_UUID  "beb5483e-36e1-4688-b7f5-ea07361b26a8"
#define TEXT_2_CHARACTERISTIC_UUID  "beb5483e-36e1-4688-b7f5-ea07361b26a9"

#define LED_SERVICE_UUID            "6b1727c4-7ec7-42e8-8111-224dde473d36"
#define LED_1_CHARACTERISTIC_UUID   "99e8194b-edfc-4cf5-bf79-f78c97edd1a4"

#define LED_PIN 2

int RSSI_val = -80
class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
      deviceConnected = true;
      BLEDevice::startAdvertising();
    };

    void onDisconnect(BLEServer* pServer) {
      deviceConnected = false;
    }
};

class LEDServiceCallback: public BLECharacteristicCallbacks{

  void onWrite(BLECharacteristic* pCharacteristic){
    uint8_t state = String(pCharacteristic->getValue().c_str()).toInt();
    digitalWrite(LED_PIN, state);
    Serial<<"LED_state: "<<( (state) ? "HIGH":"LOW" )<< endl;
    pCharacteristic->notify(false);
  }

  void onRead(BLECharacteristic* pCharacteristic){
    //Serial.print(F("Read Data: "));
    //Serial.println(pCharacteristic->getValue().c_str());
    Serial<<F("Read Data: ")<<pCharacteristic->getValue().c_str()<<endl;
  }

  void onConnect(){
    Serial<<  <<endl;
  }
};

void setup() {
  Serial.begin(115200);
  Serial.println("Starting BLE work!");
  pinMode(LED_PIN, OUTPUT);
  BLEDevice::init("ESP32_DEVICE");
  
  BLEServer *pServer = BLEDevice::createServer();   // Create a Server

  setupTextService(pServer);
  setupLEDService(pServer);
  
  // BLEAdvertising *pAdvertising = pServer->getAdvertising();  // this still is working for backward compatibility
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  //BLEAdvertisementData advertisementData;
  //  advertisementData.addData("ESP32");
  pAdvertising->addServiceUUID(TEXT_SERVICE_UUID);
  pAdvertising->addServiceUUID(LED_SERVICE_UUID);
  pAdvertising->setScanResponse(true);
  pAdvertising->setMinPreferred(0x06);  // functions that help with iPhone connections issue
  pAdvertising->setMinPreferred(0x12);
  //pAdvertising->setAdvertisementData(advertisementData);
  BLEDevice::startAdvertising();
  Serial.println("Characteristic defined! Now you can read it in your phone!");
}

void setupTextService(BLEServer *pServer){
  BLEService *pTextService = pServer->createService(TEXT_SERVICE_UUID);  // Create a Service called Text
  
  BLECharacteristic *pText_1_Characteristic = pTextService->createCharacteristic
    ( //  Create a Characteristic for Service-Text,
      TEXT_1_CHARACTERISTIC_UUID,        //  UUID of the Characteristic,
      BLECharacteristic::PROPERTY_READ | //  Lets the Characteristic Value be read from,
      BLECharacteristic::PROPERTY_WRITE  //  Lets the Characteristic Value be written to.
    ),
    *pText_2_Characteristic = pTextService->createCharacteristic
    (
      TEXT_2_CHARACTERISTIC_UUID,
      BLECharacteristic::PROPERTY_READ|
      BLECharacteristic::PROPERTY_WRITE
    );
  
  pText_1_Characteristic->setValue("Characteristic 1");   //Set value of the characteristic
  pText_2_Characteristic->setValue("80");
  pTextService->start();
}

void setupLEDService(BLEServer *pServer){
  BLEService *pLEDService = pServer->createService(LED_SERVICE_UUID);   // Create a Service called LED

  BLECharacteristic *pLED_1_Characteristic =
    pLEDService->createCharacteristic
    ( //  Create a Characteristic for Service-LED,
      LED_1_CHARACTERISTIC_UUID,          //  UUID of the Characteristic,
      BLECharacteristic::PROPERTY_READ |  //  Lets the Characteristic Value be read from,
      BLECharacteristic::PROPERTY_WRITE   //  Lets the Characteristic Value be written to.
    );

  pLED_1_Characteristic->setValue("0"); //Set value of the characteristic
  pLED_1_Characteristic->setCallbacks(new LEDServiceCallback());
  pLEDService->start();
}



void loop() {
  // put your main code here, to run repeatedly:
  delay(2000);
}
