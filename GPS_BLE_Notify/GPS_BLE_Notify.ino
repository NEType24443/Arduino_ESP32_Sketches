#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <HardwareSerial.h>
#include <Streaming.h>

#define LED_PIN 2

#define RX1 GPIO_NUM_32
#define TX1 GPIO_NUM_33

#define RX2 GPIO_NUM_25
#define TX2 GPIO_NUM_26

HardwareSerial GpsUart(1);

BLEServer* pServer = NULL;
BLECharacteristic * pLatCharacteristic = NULL, * pLngCharacteristic = NULL;
bool  deviceConnected = false,
      oldDeviceConnected = false,
      ledState = false;

// See the following for generating UUIDs:
// https://www.uuidgenerator.net/

#define GPS_SERVICE_UUID        "00001819-0000-1000-8000-00805f9b34f1"
#define GPS_LAT_CHARACTERISTIC_UUID "00002aae-0000-1000-8000-00805f9b34fb"
#define GPS_LNG_CHARACTERISTIC_UUID "00002aaf-0000-1000-8000-00805f9b34fb"

const unsigned char UBX_HEADER[] = { 0xB5, 0x62 };

struct NAV_POSLLH {
  unsigned char cls;
  unsigned char id;
  unsigned short len;
  unsigned long iTOW;
  long lon = 0.0;
  long lat = 0.0;
  long height = 0.0;
  long hMSL;
  unsigned long hAcc;
  unsigned long vAcc;
};

NAV_POSLLH posllh;

void calcChecksum(unsigned char* CK) {
  memset(CK, 0, 2);
  for (int i = 0; i < (int)sizeof(NAV_POSLLH); i++) {
    CK[0] += ((unsigned char*)(&posllh))[i];
    CK[1] += CK[0];
  }
}

bool processGPS() {
  static int fpos = 0;
  static unsigned char checksum[2];
  const int payloadSize = sizeof(NAV_POSLLH);

  while ( GpsUart.available() ) {
    byte c = GpsUart.read();
    if ( fpos < 2 ) {
      if ( c == UBX_HEADER[fpos] )
        fpos++;
      else
        fpos = 0;
    }
    else {
      if ( (fpos - 2) < payloadSize ) // All data is fed to data structure NAV_POSLLH instance posllh
        ((unsigned char*)(&posllh))[fpos - 2] = c;

      fpos++;

      if ( fpos == (payloadSize + 2) ) { // once Headers(+2) and Payload is done 
        calcChecksum(checksum);
      }
      else if ( fpos == (payloadSize + 3) ) { // 
        if ( c != checksum[0] )
          fpos = 0;
      }
      else if ( fpos == (payloadSize + 4) ) { // 
        fpos = 0;
        if ( c == checksum[1] ) {
          return true;
        }
      }
      else if ( fpos > (payloadSize + 4) ) {
        fpos = 0;
      }
    }
  }
  return false;
}

class MyServerCallbacks: public BLEServerCallbacks {
  void onConnect(BLEServer* pServer) {
    deviceConnected = true;
  }

  void onDisconnect(BLEServer* pServer) {
    deviceConnected = false;
  }
};

void setup() {
  Serial.begin(9600);
  GpsUart.begin(9600, SERIAL_8N1, RX1, TX1);
  pinMode(LED_PIN, OUTPUT);
  // Create the BLE Device
  BLEDevice::init("ESP32");
  // Create the BLE Server
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());
  
  // Create the BLE Service
  BLEService *pGpsService = pServer->createService( GPS_SERVICE_UUID );
  
  // Create a BLE Characteristic
  pLatCharacteristic = pGpsService->createCharacteristic(
                      GPS_LAT_CHARACTERISTIC_UUID,
                      BLECharacteristic::PROPERTY_READ   |
                      BLECharacteristic::PROPERTY_NOTIFY |
                      BLECharacteristic::PROPERTY_INDICATE
                    );
                    
  pLngCharacteristic = pGpsService->createCharacteristic(
                      GPS_LNG_CHARACTERISTIC_UUID,
                      BLECharacteristic::PROPERTY_READ   |
                      BLECharacteristic::PROPERTY_NOTIFY |
                      BLECharacteristic::PROPERTY_INDICATE
                    );     
                    
  pLatCharacteristic->addDescriptor(new BLE2902());
  pLngCharacteristic->addDescriptor(new BLE2902());
  
  // Start the service
  pGpsService->start();
  delay(10);
  
  // Start advertising
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(GPS_SERVICE_UUID);
  pAdvertising->setScanResponse(false);
  pAdvertising->setMinPreferred(0x0);  // set value to 0x00 to not advertise this parameter
  BLEDevice::startAdvertising();
  deviceConnected = false;
  delay(10);
  Serial<< "--> Waiting a client connection to notify them." << endl;
}

void loop() {
  // Connecting
  if (deviceConnected && !oldDeviceConnected) {
    // do stuff here on connecting
    oldDeviceConnected = deviceConnected; // oldDeviceConnected = true
    Serial<< endl << "--> Connecting";
  }
  // Disconnecting
  if (!deviceConnected && oldDeviceConnected) {
    delay(200); // give the bluetooth stack the chance to get things ready
    Serial<< ".";
    delay(200);
    Serial<< ".";
    delay(200);
    Serial<< "." << endl;
    pServer->startAdvertising(); // restart advertising
    Serial<< "--> Start Advertising Device Name" << endl;
    oldDeviceConnected = deviceConnected; // oldDeviceConnected = false
  }
  // Wait for connection
  if(!deviceConnected){
    Serial<< "--> Waiting for connection";
    while(!deviceConnected){
      if( GpsUart.available() ) GpsUart.flush();
      digitalWrite(LED_PIN, ledState^=1);
      if(ledState) delay(100);
      else{
        delay(900);
        Serial<<(".");
      }
//      delay((ledState) ? 100 : 2900);
    }
    digitalWrite(LED_PIN, LOW);
  }
  // Notify changed value
  if (deviceConnected && processGPS()) {
    digitalWrite(LED_PIN, HIGH);
    String Lat = String(posllh.lat/10000000) + "." + String(posllh.lat%10000000);
    String Lon = String(posllh.lon/10000000) + "." + String(posllh.lon%10000000);
    Serial<< "Lat: " << Lat <<"  "<< "Lon: " << Lon << endl;
    pLatCharacteristic->setValue(Lat.c_str());
    pLngCharacteristic->setValue(Lon.c_str());
    pLatCharacteristic->notify();
    pLngCharacteristic->notify();
    delay(100); // bluetooth stack will go into congestion, if too many packets are sent, in 6 hours test i was able to go as low as 3ms
    digitalWrite(LED_PIN, LOW);
    delay(400);
  }
}
