#include <BLEDevice.h>
#include <Streaming.h>

#define LED 2

static BLEAddress *pServerAddress = NULL;

BLEScan* pBLEScan;
BLEClient*  pClient;

bool deviceFound = false;

String knownAddresses[] = { "73:c8:74:44:53:6b", "  :  :  :  :  :  "};//{ "73:c8:74:44:53:6b"};  //

uint16_t RSSI_val = 90;

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

class AdvertisedDeviceCallbacks: public BLEAdvertisedDeviceCallbacks {
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
}; // AdvertisedDeviceCallbacks called when a device advertising itself was found

void doSomething() {
  //btStop();
  //delay(10);
  //  Modify the code Below
  Serial<<"Known device is close enough!"<<endl<<"Waiting for 5 seconds"<<endl;
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
  pBLEScan->setAdvertisedDeviceCallbacks(new AdvertisedDeviceCallbacks());
  pBLEScan->setActiveScan(true);
  //ledcSetup(1,1000 ,10)
}

void loop() {
  
  Serial.println();
  Serial.println("BLE Scan restarted.....");
  deviceFound = false;
  BLEScanResults scanResults = pBLEScan->start(30);
  
  if (deviceFound){
    Serial.println("on");
    digitalWrite(LED, HIGH);
    doSomething();
    delay(5000);
    //btStart();
    //delay(10);
  }
  
  else{
    Serial.println("off");
    digitalWrite(LED, LOW);
    delay(1000);
  }
  
} // End of loop
