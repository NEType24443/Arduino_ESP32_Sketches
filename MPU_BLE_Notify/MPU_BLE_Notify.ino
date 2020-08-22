#include <Wire.h>
#include <MPU6050.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <Streaming.h>

#define LED_PIN 2
#define INTERRUPT_PIN 23

#define SDA_PIN 21
#define SCL_PIN 22

#define MOTION_THRESHOLD 5   // 12
#define MOTION_EVENT_DURATION 500    //50

BLEServer* pServer = NULL;
BLECharacteristic* pCharacteristic = NULL;
bool  deviceConnected = false,
      oldDeviceConnected = false;

// See the following for generating UUIDs:
// https://www.uuidgenerator.net/

#define MPU_INT_SERVICE_UUID      "4fafc201-1fb5-459e-8fcc-c5c9c331914b"  // "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define INT_CHARACTERISTIC_UUID   "00002a59-0000-1000-8000-00805f9b34fb"  // "beb5483e-36e1-4688-b7f5-ea07361b26a8"

MPU6050 accelgyro;

volatile bool ledState = false,
              crashDetected = false;

volatile uint32_t crashCount = 0;

void IRAM_ATTR interruptServiceRoutine() {
  ledState ^= 1;
  crashDetected = 1;
  crashCount++;
}

class MyServerCallbacks: public BLEServerCallbacks {
  void onConnect(BLEServer* pServer) {
    deviceConnected = true;
    //BLEDevice::startAdvertising();
  };

  void onDisconnect(BLEServer* pServer) {
    deviceConnected = false;
  };
  
  void onNotify(BLEServer* pServer){
    
  };
};

void setup() {
  Wire.begin(SDA_PIN, SCL_PIN);
  Serial.begin(115200);
  pinMode(LED_PIN, OUTPUT);
  accelgyro.initialize();
  // verify connection to MPU6050
  Serial<<" Testing device connections..."<<endl;
  Serial<<  ( accelgyro.testConnection() ? 
            " MPU6050 connection successful" : 
            " MPU6050 connection failed")<<endl;
  delay(1);
  accelgyro.setAccelerometerPowerOnDelay(3);
  
  accelgyro.setInterruptMode(true); // Interrupts enabled
  
  accelgyro.setInterruptLatch(0); // Interrupt pulses when triggered instead of remaining on until cleared
  
  accelgyro.setIntMotionEnabled(true); // Interrupts sent when motion detected
  // Set sensor filter mode.
  // 0 -> Reset (disable high pass filter)
  // 1 -> On (5Hz)
  // 2 -> On (2.5Hz)
  // 3 -> On (1.25Hz)
  // 4 -> On (0.63Hz)
  // 5 -> Hold (Future outputs are relative to last output when this mode was set)
  accelgyro.setDHPFMode(1);

  // Motion detection acceleration threshold. 1LSB = 2mg.
  accelgyro.setMotionDetectionThreshold(MOTION_THRESHOLD);  // max uint8_t 
  // Motion detection event duration in ms
  accelgyro.setMotionDetectionDuration(MOTION_EVENT_DURATION);  // max uint8_t 
  digitalWrite(LED_PIN,LOW);
  attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), interruptServiceRoutine, RISING);
  delay(1); 
  
  // Create the BLE Device
  BLEDevice::init("ESP32");

  // Create the BLE Server
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  // Create the BLE Service
  BLEService *pService = pServer->createService(MPU_INT_SERVICE_UUID);

  // Create a BLE Characteristic
  pCharacteristic = pService->createCharacteristic(
                      INT_CHARACTERISTIC_UUID,
                      BLECharacteristic::PROPERTY_READ   |
                      BLECharacteristic::PROPERTY_WRITE  |
                      BLECharacteristic::PROPERTY_NOTIFY |
                      BLECharacteristic::PROPERTY_INDICATE
                    );

  // https://www.bluetooth.com/specifications/gatt/viewer?attributeXmlFile=org.bluetooth.descriptor.gatt.client_characteristic_configuration.xml
  // Create a BLE Descriptor
  pCharacteristic->addDescriptor(new BLE2902());

  // Start the service
  pService->start();

  // Start advertising
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(MPU_INT_SERVICE_UUID);
  pAdvertising->setScanResponse(false);
  pAdvertising->setMinPreferred(0x0);  // set value to 0x00 to not advertise this parameter
  BLEDevice::startAdvertising();
  Serial<< " Waiting a client connection to notify..." << endl;
}


void loop() {
  // Notify changed values
  if (deviceConnected && crashDetected) {
    crashDetected = 0;
    digitalWrite(LED_PIN, ledState);
    pCharacteristic->setValue((uint8_t*)&crashCount, 1);
    pCharacteristic->notify();
    Serial<<" LED State: "<<ledState<<endl
          <<" Crash Detected: "<< crashCount << endl;
    delay(50); // bluetooth stack will go into congestion, if too many packets are sent, in 6 hours test i was able to go as low as 3ms
  }
  // Disconnecting
  if (!deviceConnected && oldDeviceConnected) {
    delay(500); // give the bluetooth stack the chance to get things ready
    pServer->startAdvertising(); // restart advertising
    Serial<< endl << " Start Advertising" << endl;
    oldDeviceConnected = deviceConnected;
  }
  // Connecting
  if (deviceConnected && !oldDeviceConnected) {
    // do stuff here on connecting
    Serial<< endl << " Connecting..." << endl;
    oldDeviceConnected = deviceConnected;
  }
  // Wait for connection
  if(!deviceConnected){
    bool b = 0;
    while(!deviceConnected){
      Serial<< (b ? "." : " Waiting for connection");
      digitalWrite(LED_PIN, ledState^=1);
      delay(1000);
      b = 1;
    }
    digitalWrite(LED_PIN, LOW);
  }
}
