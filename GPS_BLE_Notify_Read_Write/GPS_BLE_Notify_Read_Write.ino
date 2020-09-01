#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <HardwareSerial.h>
#include <Streaming.h>

#define LED_PIN 2

#define RX1 GPIO_NUM_32   //  GPS TX
#define TX1 GPIO_NUM_33   //  GPS RX

#define RX2 GPIO_NUM_25   //  SIM800L TX 
#define TX2 GPIO_NUM_26   //  SIM800L RX

HardwareSerial GpsUart(1);

BLEServer* pServer = NULL;
BLECharacteristic * pLatCharacteristic = NULL, * pLngCharacteristic = NULL;
bool  deviceConnected = false,
      oldDeviceConnected = false,
      ledState = false;
uint32_t value = 0;

// See the following for generating UUIDs:
// https://www.uuidgenerator.net/

#define GPS_SERVICE_UUID            "00001819-0000-1000-8000-00805f9b34f1"
#define GPS_LAT_CHARACTERISTIC_UUID "00002aae-0000-1000-8000-00805f9b34fb"
#define GPS_LNG_CHARACTERISTIC_UUID "00002aaf-0000-1000-8000-00805f9b34fb"

const unsigned char UBX_HEADER[] = { 0xB5, 0x62 };

struct NAV_POSLLH {
  // Type         Name             Unit     Description (scaling)
  unsigned char   cls;
  unsigned char   id;
  unsigned short  len;
  unsigned long   iTOW;         // ms       GPS time of week of the navigation epoch. See the description of iTOW for details
  long            lon = 0.0;    // deg      Longitude (1e-7)
  long            lat = 0.0;    // deg      Latitude (1e-7)
  long            height = 0.0; // mm       Height above ellipsoid
  long            hMSL;         // mm       Height above mean sea level
  unsigned long   hAcc;         // mm       Horizontal accuracy estimate
  unsigned long   vAcc;         // mm       Vertical accuracy estimate

};

struct NAV_SOL {
  // Type         Name           Unit     Description (scaling)
  unsigned char   cls;
  unsigned char   id;
  unsigned short  len;
  unsigned long   iTOW;       // ms       GPS time of week of the navigation epoch. See the description of iTOW for details
  long            fTOW;       // ns       Fractional part of iTOW (range: +/-500000). The precise GPS time of week in
                              //          seconds is: (iTOW * 1e-3) + (fTOW * 1e-9)
  short           week;       // weeks    GPS week number of the navigation epoch
  unsigned char   gpsFix;     // -        GPSfix Type, range 0..5
  char            flags;      // -        Fix Status Flags (see graphic below)
  long            ecefX;      // cm       ECEF X coordinate
  long            ecefY;      // cm       ECEF Y coordinate
  long            ecefZ;      // cm       ECEF Z coordinate
  unsigned long   pAcc;       // cm       3D Position Accuracy Estimate
  long            ecefVX;     // cm/s     ECEF X velocity
  long            ecefVY;     // cm/s     ECEF Y velocity
  long            ecefVZ;     // cm/s     ECEF Z velocity
  unsigned long   sAcc;       // cm/s     Speed Accuracy Estimate
  unsigned short  pDOP;       // -        Position DOP (0.01)
  unsigned char   reserved1;  // -        Reserved
  unsigned char   numSV;      // -        Number of satellites used in Nav Solution
  unsigned long   reserved2;  // -        Reserved
};

NAV_SOL Sol;
NAV_POSLLH Posllh;

void calcChecksum(unsigned char* CK) {
  memset(CK, 0, 2);
  for (int i = 0; i < (int)sizeof(NAV_POSLLH); i++) {
    CK[0] += ((unsigned char*)(&Posllh))[i];
    CK[1] += CK[0];
  }
}

bool processGPS(uint8_t data_struct) {
  static int fpos = 0;
  static unsigned char checksum[2];
  switch(data_struct){
    case 0:
      const int payloadSize = sizeof(NAV_POSLLH);
      GpsUart<<_HEX(0x01)<<_HEX(0x02);
      break;
    case 1:
      const int payloadSize = sizeof(NAV_SOL);
      GpsUart<<_HEX(0x01)<<_HEX(0x02);
      break;
    
  }
  // sizeof(NAV_POSLLH); // 28 bytes
  // sizeof(NAV_SOL); // 52 bytes
  while (!GpsUart.available());
  while ( GpsUart.available()){
    byte c = GpsUart.read();
    if ( fpos < 2 ){
      if ( c == UBX_HEADER[fpos] )
        fpos++;
      else
        fpos = 0;
    }
    // Sync with header after success.
    else{
      // Put the byte read to a particular address of this object which depends on the carriage position.
      if ( (fpos - 2) < payloadSize )
        ((unsigned char*)(&Posllh))[fpos - 2] = c;
      // Move the carriage forward.  
      fpos++;

      // Carriage is at the first checksum byte, we can calculate our checksum, but not compare, because this byte is
      // not read.
      if ( fpos == (payloadSize + 2) ) {
        calcChecksum(checksum);
      }
      // Carriage is at the second checksum byte, but only the first byte of checksum read, check if it equals to  
      // ours.
      else if ( fpos == (payloadSize + 3) ) {
        // Reset if not correct.
        if ( c != checksum[0] )
          fpos = 0;
      }
      // Carriage is after the second checksum byte, which has been read, check if it equals to ours.
      else if ( fpos == (payloadSize + 4) ) {
        // Reset the carriage.
        fpos = 0;
        // The readings are correct and filled the object, return true.
        if ( c == checksum[1] ) {
          return true;
        }
      }
      // Reset the carriage if it is out of a packet.
      else if ( fpos > (payloadSize + 4) ) {
        fpos = 0;
      }
    }
  }
  return false;
}

class GpsLatCharacteristicCallbacks: public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic* pCharacteristic){
    //Serial<<"Lat Set to:"<< pCharacteristic->getValue() << endl;
    std::string Lat = pCharacteristic->getValue();
    //ESP_LOGI(Lat);
    Lat += " N";
    uint8_t i = 0;
    while(Lat.length()>i){
     Serial<< Lat[i];
     i++;
    }
    Serial<< endl;
  }
};

class GpsLonCharacteristicCallbacks: public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic* pCharacteristic){
    //Serial<<"Lon Set to:"<< pCharacteristic->getValue();
    std::string Lon = pCharacteristic->getValue();
    //ESP_LOGI(Lon);
    Lon += " E";
    uint8_t i = 0;
    while(Lon.length()>i){
     Serial<< Lon[i];
     i++;
    }
    Serial<< endl;
  }
};

class MyServerCallbacks: public BLEServerCallbacks {
  void onConnect(BLEServer* pServer) {
    deviceConnected = true;
  }

  void onDisconnect(BLEServer* pServer) {
    deviceConnected = false;
  }
};

void setup() {
  Serial.begin(115200);
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
                      BLECharacteristic::PROPERTY_WRITE  |
                      BLECharacteristic::PROPERTY_READ   |
                      BLECharacteristic::PROPERTY_NOTIFY |
                      BLECharacteristic::PROPERTY_INDICATE
                    );
                    
  pLngCharacteristic = pGpsService->createCharacteristic(
                      GPS_LNG_CHARACTERISTIC_UUID,
                      BLECharacteristic::PROPERTY_WRITE  |
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
  pLatCharacteristic->setCallbacks(new GpsLatCharacteristicCallbacks());
  pLngCharacteristic->setCallbacks(new GpsLonCharacteristicCallbacks());
  BLEDevice::startAdvertising();
  deviceConnected = false;
  delay(10);
  Serial<< F("--> Waiting a client connection to notify them.") << endl;
}

void loop() {
  // Connecting
  if (deviceConnected && !oldDeviceConnected) {
    // do stuff here on connecting
    oldDeviceConnected = deviceConnected; // oldDeviceConnected = true
    Serial<< F("--> Connecting\n");
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
    Serial<< F("--> Start Advertising Device Name") << endl;
    oldDeviceConnected = deviceConnected; // oldDeviceConnected = false
  }
  // Wait for connection
  if(!deviceConnected){
    Serial<< F("--> Waiting for connection");
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
    Serial<<endl;
    digitalWrite(LED_PIN, LOW);
    GpsUart.flush();
  }
  // Notify changed value
  if (deviceConnected && processGPS()) {
    digitalWrite(LED_PIN, HIGH);
    String Lat = String(Posllh.lat/10000000) + "." + String(Posllh.lat%10000000);
    String Lon = String(Posllh.lon/10000000) + "." + String(Posllh.lon%10000000);
    Serial<< "Lat: " << Lat << F(" N  ") << "Lon: " << Lon << F(" E") << endl;
    pLatCharacteristic->setValue(Lat.c_str());
    pLngCharacteristic->setValue(Lon.c_str());
    pLatCharacteristic->notify();
    pLngCharacteristic->notify();
    delay(100); // bluetooth stack will go into congestion, if too many packets are sent, in 6 hours test i was able to go as low as 3ms
    digitalWrite(LED_PIN, LOW);
    delay(400);
  }
}                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                            
