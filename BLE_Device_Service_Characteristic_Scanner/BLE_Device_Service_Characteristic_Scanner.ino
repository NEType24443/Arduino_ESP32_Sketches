 #include <BLEDevice.h>
#include <Streaming.h>

#define LED 2

static BLEAddress *pServerAddress = NULL;

BLEScan* pBLEScan;
//BLEClient*  pClient;
static BLEAdvertisedDevice* myDevice;
static BLERemoteCharacteristic* pRemoteCharacteristic;

bool deviceFound = false;
static boolean doConnect = false;
static boolean connected = false;
static boolean doScan = false;

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
    Serial.print("data: ");
    Serial.println((char*)pData);
}

class AdvertisedDeviceCallbacks: public BLEAdvertisedDeviceCallbacks {
  void onResult(BLEAdvertisedDevice advertisedDevice) {
    Serial.print("BLE Advertised Device found: ");
    Serial.println(advertisedDevice.toString().c_str());  //  Prints -> Name: AdvertisedDeviceName, Address: AdvertisedDeviceAddress
    pServerAddress = new BLEAddress(advertisedDevice.getAddress());
    Serial.print("Device RSSI: ");
    Serial.println(advertisedDevice.getRSSI());
    //Serial.println(pServerAddress->toString().c_str());
    if (advertisedDevice.getRSSI() > -RSSI_val){
      deviceFound = true;
      if (advertisedDevice.haveServiceUUID()) {
        BLEDevice::getScan()->stop();
        Serial<<"Stopped Scaning"<<endl;
        myDevice = new BLEAdvertisedDevice(advertisedDevice);
        doConnect = true;
        doScan = true;
      } // Found our server
    }
    else deviceFound = false;
    // We have found a device, let us now see if it contains the service we are looking for.
    //advertisedDevice.getScan()->stop();
  }
}; // AdvertisedDeviceCallbacks called when a device advertising itself was found

bool connectToServer() {
    Serial.print("Forming a connection to ");
    Serial.println(myDevice->getAddress().toString().c_str());
    
    BLEClient*  pClient  = BLEDevice::createClient();
    Serial.println(" - Created client");

    //pClient->setClientCallbacks(new MyClientCallback());

    // Connect to the remove BLE Server.
    pClient->connect(myDevice);  // if you pass BLEAdvertisedDevice instead of address, it will be recognized type of peer device address (public or private)
    Serial.println(" - Connected to server");
    connected = true;
    return true;
}

void ReadServices() {
  //  Modify the code Below
  Serial<<"Known device is close enough!"<<endl<<"Waiting for 5 seconds"<<endl;
  if (doConnect == true) {
    if (connectToServer()) {
      Serial.println("We are now connected to the BLE Server.");
    } else {
      Serial.println("We have failed to connect to the server; there is nothing more we will do.");
    }
    doConnect = false;
  }
  // If we are connected to a peer BLE Server, update 
  // the characteristic each time we are reached
  // with the current time since boot.
  /*
  if (connected) {
    String newValue = "Time since boot: " + String(millis()/1000);
    Serial.println("Setting new characteristic value to \"" + newValue + "\"");
    // Set the characteristic's value to be the array of bytes that is actually a string.
    pRemoteCharacteristic->writeValue(newValue.c_str(), newValue.length());
  }
  else if(doScan){
    BLEDevice::getScan()->start(0);  // this is just eample to start scan after disconnect, most likely there is better way to do it in arduino
  }*/
  // Obtain a reference to the service we are after in the remote BLE server.
    /*BLERemoteService* pRemoteService = pClient->getService(serviceUUID);
    if (pRemoteService == nullptr) {
      Serial.print("Failed to find our service UUID: ");
      Serial.println(serviceUUID.toString().c_str());
      pClient->disconnect();
      return false;
    }
    Serial.println(" - Found our service");
    

    // Obtain a reference to the characteristic in the service of the remote BLE server.
    pRemoteCharacteristic = pRemoteService->getCharacteristic(charUUID);
    if (pRemoteCharacteristic == nullptr) {
      Serial.print("Failed to find our characteristic UUID: ");
      Serial.println(charUUID.toString().c_str());
      pClient->disconnect();
      return false;
    }
    Serial.println(" - Found our characteristic");

    // Read the value of the characteristic.
    if(pRemoteCharacteristic->canRead()) {
      std::string value = pRemoteCharacteristic->readValue();
      Serial.print("The characteristic value was: ");
      Serial.println(value.c_str());
    }

    if(pRemoteCharacteristic->canNotify())
      pRemoteCharacteristic->registerForNotify(notifyCallback);
    */
    //pClient->disconnect();
}

void setup() {
  Serial.begin(9600);
  Serial.println("Starting Arduino BLE Client application...");
  pinMode(LED, OUTPUT);
  digitalWrite(LED, LOW);

  BLEDevice::init("ESP32");

  pBLEScan = BLEDevice::getScan();
  pBLEScan->setAdvertisedDeviceCallbacks(new AdvertisedDeviceCallbacks());
  pBLEScan->setActiveScan(true);
  //ledcSetup(1,1000 ,10)
}

void loop() {
  
  Serial.println();
  Serial.println("BLE Scan restarted.....");
  deviceFound = false;
  BLEScanResults scanResults = pBLEScan->start(10);
  
  if (deviceFound){   // True if signal strength is high enough
    ReadServices();
   digitalWrite(LED, HIGH);
    delay(5000);
    //btStart();
    //delay(10);
  }
  
  else{
    digitalWrite(LED, LOW);
    delay(1000); //  Scan Interval
  }
  
} // End of loop
