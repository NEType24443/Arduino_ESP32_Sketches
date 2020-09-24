#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <Wire.h>
#include <MPU6050.h>
#include <Streaming.h>
#include <HardwareSerial.h>

#define ON    1
#define OFF   0

#define DEBUG ON

#define INITIAL_SETUP   0
#define SENTRY_MODE     1
#define ALERT_MODE      2
#define TRAVEL_MODE     3

#define LED_PIN         GPIO_NUM_2    //  ONBOARD LED

#define INTERRUPT_RING  GPIO_NUM_18   //  SIM800L RING PIN
#define DTR_PIN         GPIO_NUM_5    //  SIM800L DTR  PIN

#define INTERRUPT_MPU   GPIO_NUM_23   //  23
#define SDA_PIN         GPIO_NUM_22   //  22
#define SCL_PIN         GPIO_NUM_21   //  21

#define RX1             GPIO_NUM_32   //  GPS TX
#define TX1             GPIO_NUM_33   //  GPS RX

#define RX2             GPIO_NUM_16   //  SIM800L TX 
#define TX2             GPIO_NUM_17   //  SIM800L RX

#define ACCIDENT_MOTION_THRESHOLD       5
#define ACCIDENT_MOTION_EVENT_DURATION  500

#define SENTRY_MOTION_THRESHOLD         12
#define SENTRY_MOTION_EVENT_DURATION    50

// See the following for generating UUIDs:
// https://www.uuidgenerator.net/

#define GPS_SERVICE_UUID            "00001819-0000-1000-8000-00805f9b34f1"
#define GPS_LAT_CHARACTERISTIC_UUID "00002aae-0000-1000-8000-00805f9b34fb"
#define GPS_LNG_CHARACTERISTIC_UUID "00002aaf-0000-1000-8000-00805f9b34fb"
#define SETUP_CHARACTERISTIC_UUID   "eae80b48-5b8b-49bb-b5d3-ec3aed3ca178"

#define NO_FIX  0x00  //  NO-FIX
#define DR      0x01  //  DEAD RECKONING
#define _2D_Fix 0x02  //  2D-FIX
#define _3D_Fix 0x03  //  3D-FIX
#define GPS_DR  0x04  //  GPS + DEAD RECKONING
#define TOF     0x05  //  TIME ONLY FIX

HardwareSerial GpsUart(1);
HardwareSerial GsmUart(2);

MPU6050 accelgyro;

// Operation Modes
RTC_DATA_ATTR uint8_t mode;

// Phone Number Resgistration variables
RTC_DATA_ATTR String ownerName;
RTC_DATA_ATTR String phoneNumbers[5] = {}; // = "09481501967"; //"09448517225";
RTC_DATA_ATTR uint8_t registeredCount = 0;
//RTC_DATA_ATTR uint8_t 
const String BaseLink = "Automated System: The current location of %'s vehicle is https://www.google.com/maps/place/"; 

volatile  bool stolen = 0, crashed = 0, ringPin = 0;
          bool lastCrashed = 0;

BLEServer* pServer = NULL;
BLECharacteristic * pLatCharacteristic = NULL, * pLngCharacteristic = NULL;

// Stored like this to prevent overflow
uint32_t  phoneLatLow = 0, phoneLonLow = 0; // Low is to right of decimal pt.
int16_t   phoneLatUp  = 0, phoneLonUp  = 0; // Up  is to left  of decimal pt.

bool  deviceConnected = false,
      oldDeviceConnected = false;

const unsigned char UBX_HEADER[] = { 0xB5, 0x62 };

const byte posllh_start[] = {0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0x01, 0x02, 0x01, 0x0E, 0x47};
                            // B5 62 06 01 03 00 01 02 01 0E 47 -> POSLLH START
const byte posllh_stop[]  = {0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0x01, 0x02, 0x00, 0x0D, 0x46}; 
                            // B5 62 06 01 03 00 01 02 00 0D 46 -> POSLLH STOP
const byte    sol_start[] = {0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0x01, 0x06, 0x01, 0x12, 0x4F}; 
                            // B5 62 06 01 03 00 01 06 01 12 4F -> SOL    START
const byte    sol_stop[]  = {0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0x01, 0x06, 0x00, 0x11, 0x4E}; 
                            // B5 62 06 01 03 00 01 06 00 11 4E -> SOL    STOP
# if DEBUG > 0
uint32_t match_counter = 0, not_match_counter = 0;
# endif

struct NAV_POSLLH {
  // Type         Name             Unit     Description (scaling)
  unsigned char   cls;
  unsigned char   id;
  unsigned short  len;
  unsigned long   iTOW;         // ms       GPS time of week of the navigation epoch. See the description of iTOW for details
  long            lon = 0;      // deg      Longitude (1e-7)
  long            lat = 0;      // deg      Latitude (1e-7)
  long            height = 0;   // mm       Height above ellipsoid
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

NAV_SOL Sol;        // 52 bytes + class + id + length = 56
NAV_POSLLH Posllh;  // 28 bytes + class + id + length = 32

void IRAM_ATTR interruptServiceRoutineMpu(){
  if(mode == SENTRY_MODE) {
    stolen = 1;
  }
  else if(mode == TRAVE_MODE){
    //ledState ^= 1;
    crashed = 1;
    //digitalWrite(LED_PIN, ledState);
    //Serial<<"LED State: "<<ledState<<endl;
    Serial<<"---> Crashed "<< endl;
  }
}

void IRAM_ATTR interruptServiceRoutineRing(){
  crashed = 1;
  ringPin = 1;
  Serial <<"\n---> RING PIN HIGH \n"; 
}

byte calcChecksum(unsigned char* CK, int payloadSize) {
  memset(CK, 0, 2); //  Setting the 2 bytes of the checksum array as zero
  for (int i = 0; i < payloadSize; i++) {
    switch(payloadSize){
      case 32:
        CK[0] += ((unsigned char*)(&Posllh))[i];
        break;
      case 56:
        CK[0] += ((unsigned char*)  (&Sol) )[i];
        break;
    }
    CK[1] += CK[0];
  }
}

bool processGPS(uint8_t payloadSize) {
  static int fpos = 0; //static
  static unsigned char checksum[2]; //static
//  int payloadSize = 0;
  //GpsUart.flush();
  # if DEBUG > 0
  Serial<< F("\nR --> ");
  # endif
  while (!GpsUart.available());
  while ( GpsUart.available()){
    byte c = GpsUart.read();
    # if DEBUG > 0
    Serial.write(c);
    # endif
    if ( fpos < 2 ){
      if ( c == UBX_HEADER [fpos] )
        fpos++;
      else
        fpos = 0;
    }
    // Sync with header after success.
    else{
      // Put the byte read to a particular address of this object which depends on the carriage position.
      if ( (fpos - 2) < payloadSize ){
        switch(payloadSize){
          case 32:
            ((unsigned char*)(&Posllh))[fpos - 2] = c;
            break;
          case 56:
            ((unsigned char*)(&Sol))[fpos - 2] = c;
            break;
        }
      }
      // Move the carriage forward.  
      fpos++;

      // Carriage is at the first checksum byte, we can calculate our checksum, but not compare, because this byte is
      // not read.
      if ( fpos == (payloadSize + 2) ) {
        calcChecksum(checksum, payloadSize);
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
          //checksum[0] = 0;
          //checksum[1] = 0;
          # if DEBUG > 0
          match_counter++;  
          Serial<<F("\nC --> Checksum match !")<<endl
                <<F("Match Counter")<<_WIDTH(match_counter,7)
                <<F("\tNot Match Counter")<<_WIDTH(not_match_counter,7)<<endl;
          # endif
          return true;
        }
      }
      // Reset the carriage if it is out of a packet.
      else if ( fpos > (payloadSize + 4) ) {
        # if DEBUG > 0
        Serial<< F("\nE --> Carrige has been reset") <<endl;
        # endif
        fpos = 0;
      }
    }
  }
  # if DEBUG > 0
  not_match_counter++;
  Serial<< F("\nC --> No checksum match !")<< endl
        << F("Match Counter")       << _WIDTH(  match_counter,   7)
        << F("\tNot Match Counter") << _WIDTH(not_match_counter, 7) <<endl;
  # endif
  return false;
}

class GpsLatCharacteristicCallbacks: public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic* pCharacteristic){
    std::string Lat = pCharacteristic->getValue();
    uint8_t i = 0, decimalPlaces = 0;
    bool    decimalPt = 0;
    phoneLatUp = 0;
    phoneLatLow = 0;
    while(Lat.length() > i){
      #if DEBUG > 0
      Serial<< Lat[i];
      #endif
      if(Lat[i] != '.' ){  // Conver Char to Int without the '.' (decimal point) character of the string
        if(!decimalPt){
          phoneLatUp  += (Lat[i] - '0');
          phoneLatUp  *= 10; 
        }
        else{
          phoneLatLow += (Lat[i] - '0');
          phoneLatLow *= 10;
          decimalPlaces++;
        }
      }
      else decimalPt = 1;
      i++;
    }
    phoneLatUp  /= 10;
    phoneLatLow /= 10;
    while(decimalPlaces<7){
      phoneLatLow *= 10;
      decimalPlaces++;
    }
    #if DEBUG > 0
    Serial<<" N"<< endl;
    Serial<<phoneLatUp<<phoneLatLow<<endl;
    #endif
    pCharacteristic->notify();
  }
};

class GpsLonCharacteristicCallbacks: public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic* pCharacteristic){
    std::string Lon = pCharacteristic->getValue();
    uint8_t i = 0, decimalPlaces = 0;
    bool    decimalPt = 0;
    phoneLonUp = 0;
    phoneLonLow = 0;
    while(Lon.length() > i){
      #if DEBUG > 0
      Serial<< Lon[i];
      #endif
      if(Lon[i] != '.' ){  // Conver Char to Int without the '.' (decimal point) character of the string
        if(!decimalPt){
          phoneLonUp  += (Lon[i] - '0');
          phoneLonUp  *= 10;
        }
        else{
          phoneLonLow += (Lon[i] - '0');
          phoneLonLow *= 10;
          decimalPlaces++;
        }
      }
      else decimalPt = 1;
      i++;
    }
    phoneLonUp  /= 10;
    phoneLonLow /= 10;
    while(decimalPlaces<7){
      phoneLonLow *= 10;
      decimalPlaces++;
    }
    #if DEBUG > 0
    Serial<<" E"<< endl;
    Serial<<phoneLonUp<<phoneLonLow<<endl;
    #endif
    pCharacteristic->notify();
  }
};

class InitialSetupCharacteristicCallbacks: public BLECharacteristicCallbacks{
  void onWrite(BLECharacteristic* pCharacteristic){
    
  }
};

class MyServerCallbacks: public BLEServerCallbacks {
  void onConnect(BLEServer* pServer) {
    deviceConnected = true;
  }

  void onDisconnect(BLEServer* pServer) {
    deviceConnected = false;
    mode = SENTRY_MODE;
  }
};

String GsmUartSend(String incoming){ //Function to communicate with SIM800 module
  GsmUart<<incoming<<endl; delay(100); //Print what is being sent to GSM module
  String result = "";
  while (GsmUart.available()) //Wait for result
  {
    char letter = GsmUart.read();
    result = result + String(letter); //combine char to string to get result
  }
  return result; //return the result
}

void requestGPS( bool state ){
  # if DEBUG > 0
  Serial<<_FMT("\nR <-- POSLLH  Periodic % Request",(state)? F("START") : F("STOP"));
  # endif
  for(uint8_t i = 0; i < 11; i++){
    if(state){
      GpsUart.write(posllh_start[i]);
      # if DEBUG > 0
      Serial.write(posllh_start[i]);
      # endif
    }
    else{
      GpsUart.write(posllh_stop[i]);
      # if DEBUG > 0
      Serial.write(posllh_stop[i]);
      # endif
    }
  }
  delay(5);
  # if DEBUG > 0
  Serial<<_FMT("\nR <-- SOL Periodic % Request",(state)? F("START") : F("STOP"));
  # endif
  for(uint8_t i = 0; i < 11; i++){
    if(state){
      GpsUart.write(sol_start[i]);
      # if DEBUG > 0
      Serial.write(sol_start[i]);
      # endif
    }
    else{
      GpsUart.write(sol_stop[i]);
      # if DEBUG > 0
      Serial.write(sol_stop[i]);
      # endif
    }
  }
}

void setupAccelGyro(){
  accelgyro.initialize();
  // verify connection to MPU6050
  Serial<<"Testing device connections..."<<endl;
  Serial<<(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed")<<endl;
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
  accelgyro.setMotionDetectionThreshold(ACCIDENT_MOTION_THRESHOLD);
  // Motion detection event duration in ms
  accelgyro.setMotionDetectionDuration(ACCIDENT_MOTION_EVENT_DURATION);
  delay(100);
}

void setupGsmModule(){
  //Enable Echo if not enabled by default
  # if DEBUG > 0
  Serial<<"Response: "<<GsmUartSend("ATE1")<<endl;
  #elif
  GsmUart<<"ATE1"<<endl;
  #endif
  delay(1000);
  
  //Set the SIM800L in GPRS mode
   # if DEBUG > 0
  Serial<<"Response: "<<GsmUartSend("AT+CGATT=1")<<endl;
  #elif
  GsmUart<<"AT+CGATT=1"<<endl;
  #endif
  delay(1000);

  //Activate Bearer profile
   # if DEBUG > 0
  Serial<<"Response: "<<GsmUartSend("AT+SAPBR=3,1,\"CONTYPE\",\"GPRS\" ")<<endl;
  #elif
  GsmUart<<"AT+SAPBR=3,1,\"CONTYPE\",\"GPRS\" "<<endl;
  #endif
  delay(1000);

  //Set VPN options => 'RCMNET' 'www' 
  # if DEBUG > 0
  Serial<<"Response: "<<GsmUartSend("AT+SAPBR=3,1,\"APN\",\"airtelgprs.com\" ")<<endl;
  #elif
  GsmUart<<"AT+SAPBR=3,1,\"APN\",\"airtelgprs.com\" "<<endl;
  #endif
  delay(1000);

  //Open bearer Profile
  # if DEBUG > 0
  Serial<<"Response: "<<GsmUartSend("AT+SAPBR=1,1")<<endl;
  #elif
  GsmUart<<"AT+SAPBR=1,1"<<endl;
  #endif
  delay(1000);

  //Get the IP address of the bearer profile
  # if DEBUG > 0
  Serial<<"Response: "<<GsmUartSend("AT+SAPBR=2,1")<<endl;
  #elif
  GsmUart<<"AT+SAPBR=2,1"<<endl;
  #endif
  delay(1000);
}

void setupGpsService(){
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
  
}

void setup() {
  pinMode(DTR_PIN,      OUTPUT);
  digitalWrite(DTR_PIN, LOW   );
  Wire.begin(SDA_PIN, SCL_PIN );

  #if DEBUG>0
  Serial.begin(115200);
  #endif
  GpsUart.begin(115200, SERIAL_8N1, RX1, TX1);
  GsmUart.begin(9600  , SERIAL_8N1, RX2, TX2);

  pinMode(LED_PIN,        OUTPUT);
  pinMode(INTERRUPT_MPU,  INPUT );
  pinMode(INTERRUPT_RING, INPUT );
  
  // Create the BLE Device
  BLEDevice::init("ESP32 Crash Detector");

  // Create the BLE Server
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());
  #if DEBUG>0
  Serial<< F("--> BLE server initialised.") << endl;
  #endif

  setupGpsService();
  #if DEBUG>0
  Serial<< F("--> Gps Service initialised.") << endl;
  #endif
  
  setupAccelGyro();
  #if DEBUG>0
  Serial<< F("--> MPU6050 initialised.") << endl;
  #endif
  
  setupGsmModule();
  #if DEBUG>0
  Serial<< F("--> SIM800L initialised.") << endl;
  #endif

  digitalWrite(LED_PIN,LOW);
  attachInterrupt(digitalPinToInterrupt(INTERRUPT_MPU) , interruptServiceRoutineMpu , RISING  );
  attachInterrupt(digitalPinToInterrupt(INTERRUPT_RING), interruptServiceRoutineRing, FALLING );
  delay(1);
}

void loop() {
  switch(mode){
    case TRAVEL_MODE:
      // Wait for connection
      if (!deviceConnected && !oldDeviceConnected){
        Serial<< F("--> Waiting for connection");
        while(!deviceConnected){
          //if( GpsUart.available() ) GpsUart.flush();
          digitalWrite(LED_PIN, ledState^=1);
          if(ledState) delay(100);
          else{
            delay(900);
            Serial<<(".");
          }
//          delay((ledState) ? 100 : 2900);
        }
        Serial<<endl;
        digitalWrite(LED_PIN, LOW);
        //GpsUart.flush();
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
      // Connecting
      if (deviceConnected && !oldDeviceConnected) {
        // do stuff here on connecting
        oldDeviceConnected = deviceConnected; // oldDeviceConnected = true
        Serial<< F("--> Connecting\n"); 
      }
      // Notify changed value
      if(deviceConnected || ringPin){
        if(ringPin){
          if(GsmUart.available()){
            if(GsmUart.readString() == "RING"){
              GsmUart<<"ATH"<<endl;
            }
            delay(5);
            GsmUart.flush();
          }
          #if DEBUG > 0
          Serial<<"Response: "<<GsmUartSend("ATE0")<<endl;
          #elif
          GsmUart<<"ATE0"<<endl;
          #endif
          GsmUart.flush();
        }
        if(!lastCrashed && crashed ){
          requestGPS(ON);
          digitalWrite(LED_PIN, HIGH);
          lastCrashed = 1;
          delay(5);
        }
        if( lastCrashed && !crashed){
          requestGPS(OFF);
          delay(5);
          GpsUart.flush();
          digitalWrite(LED_PIN, LOW);
          lastCrashed = 0;
        }
        if( lastCrashed && crashed ){
          uint8_t counter = 0;
          bool gotGpsData = false;
          double  lat = 0, lon = 0;
          while(!Sol.gpsFix){
            if(processGPS(sizeof(NAV_POSLLH))){
              # if DEBUG > 0
              Serial<<"Got Data"<<endl;
              # endif
        //      String Lat = String(Posllh.lat/10000000) + "." + String(Posllh.lat%10000000);
              lat = Posllh.lat/10000000.F;
        //      String Lon = String(Posllh.lon/10000000) + "." + String(Posllh.lon%10000000);
              lon = Posllh.lon/10000000.F;
              # if DEBUG > 0
              Serial<< F("Lat: ")  << _FLOAT(lat, 7) << F(" N   ") 
                    << F("Lon: ")  << _FLOAT(lon, 7) << F(" E") << endl
                    << F("iTOW: ") << Posllh.iTOW    << endl;
              #endif
              //delay(200);
              //digitalWrite(LED_PIN, LOW);
              //delay(700);
            }
            if(processGPS(sizeof(NAV_SOL))){
              # if DEBUG > 0
              Serial<<"Got Data"<<endl;
              // digitalWrite(LED_PIN, HIGH);
              String Fix = "";
              switch (Sol.gpsFix){
                case NO_FIX:
                  Fix = F("No Fix");
                  break;
                case DR:
                  Fix = F("Dead Reckoning");
                  break;
                case _2D_Fix:
                  Fix = F("2D-Fix");
                  break;
                case _3D_Fix:
                  Fix = F("3D-Fix");
                  break;
                case GPS_DR:
                  Fix = F("GPS + Dead Reckoning");
                  break;
                case TOF:
                  Fix = F("Time Only Fix");
                  break;
                default:
                  Fix = F("INVALID");
              }
              Serial<< "Fix Status:"<< Fix << endl;
              #endif
              if(Sol.gpsFix){
                gotGpsData = true;
                break;
              }
            }
            counter++;
            if(counter>50){
              gotGpsData = false; 
              break;
            }
          }
          
          #if DEBUG > 0
          Serial<<"Response: "<<GsmUartSend("ATE0")<<endl; //Disable Echo
          #elif
          GsmUart<<"ATE0"<<endl;
          #endif
          delay (1000);
    
          #if DEBUG > 0
          Serial <<"Sending sms"<<endl;
          #endif
          GsmUart<<"AT+CMGF=1"<<endl; //Set the module in SMS mode
          delay(1000);
            
          if(gotGpsData){
            #if DEBUG > 0
            Serial<<"GOT GPS LOCATION"<<endl;
            #endif
    //        #if DEBUG > 0
    //        Serial<<"Response:"<<GsmUartSend("ATH" )<<endl; //Hand up the incoming call using ATH
    //        #elif
    //        GsmUartSend("ATH");
    //        #endif
    //        delay (1000);
            
            GsmUart<<"AT+CMGS="<<"\""<<phoneNumbers[0]<<"\""<<endl; //Send SMS to this number
            delay(1000);
            
            #if DEBUG > 0
            Serial << BaseLink << _FLOAT(lat, 7) <<","<< _FLOAT(lon, 7) << endl;
            #endif
            GsmUart<< BaseLink << _FLOAT(lat, 7) <<","<< _FLOAT(lon, 7) << endl<< (char)26 <<endl;
            delay(1000);
          }
          else{
            #if DEBUG > 0
            Serial<<"COULD NOT GET GPS LOCATION USING LAST PHONE LOCATION"<<endl;
            #endif
            
            GsmUart<<"AT+CMGS="<<"\""<<phoneNumbers[0]<<"\""<<endl; //Send SMS to this number
            delay(1000);
    
            #if DEBUG > 0
            Serial << BaseLink 
                   << phoneLatUp << "." << phoneLatLow << "," 
                   << phoneLonUp << "." << phoneLonLow << endl;
            #endif
            GsmUart<< BaseLink 
                   << phoneLatUp << "." << phoneLatLow << "," 
                   << phoneLonUp << "." << phoneLonLow << endl<< (char)26 <<endl;
            delay(1000);
          }
          crashed = 0;
          ringPin = 0;
        }
      }
      break;
    
    case ALERT_MODE:
      
      break;
    
    case SENTRY_MODE: 
      
      break;
      
  }
}
