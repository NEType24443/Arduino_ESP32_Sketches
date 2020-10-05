#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <Wire.h>
#include <MPU6050.h>
#include <Streaming.h>
#include <EEPROM.h>
#include <FreeRTOS.h>
#include <HardwareSerial.h>
// #include <BluetoothSerial.h>

// #define INCLUDE_uxTaskGetStackHighWaterMark 1

#define ON    1
#define OFF   0

#define DEBUG ON

#define PSWD "password"

#define INITIAL_SETUP   0
#define SENTRY_MODE     1
#define ALERT_MODE      2
#define TRAVEL_MODE     3

#define SENTRY_INTERVAL 2000UL

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

#define GPS_SERVICE_UUID                        "00001819-0000-1000-8000-00805f9b34f1"
#define INITIAL_SERVICE_UUID                    "eae80b48-5b8b-49bb-b5d3-ec3aed3ca178"
#define GPS_LAT_CHARACTERISTIC_UUID             "00002aae-0000-1000-8000-00805f9b34fb"
#define GPS_LNG_CHARACTERISTIC_UUID             "00002aaf-0000-1000-8000-00805f9b34fb"
#define OWNER_FIRST_NAME_CHARACTERISTIC_UUID    "00002a8a-5b8b-49bb-b5d3-ec3aed3ca178"
#define OWNER_LAST_NAME_CHARACTERISTIC_UUID     "00002a90-5b8b-49bb-b5d3-ec3aed3ca178"
#define OWNER_NUM_CHARACTERISTIC_UUID           "00002a25-5b8b-49bb-b5d3-ec3aed3ca178"

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

// Phone number resgistration variables
RTC_DATA_ATTR String owner_first_name="", owner_last_name="", owner_name="", owner_number="";
RTC_DATA_ATTR String phoneNumbers[5] = { "0000000000000", 
                                         "0000000000000", 
                                         "0000000000000", 
                                         "0000000000000", 
                                         "0000000000000" }; // = "09481501967"; //"09448517225";
                                         
const String BaseLink = "The current location of the vehicle is https://www.google.com/maps/place/"; 

// SENTRY Report timer
uint32_t last = 0;

// Interrupt Flags
volatile  bool crashed = 0,      ringPin = 0;
          bool lastCrashed = 0;

volatile bool stolen = 0;
//         bool stolen_ff = 0;

volatile uint16_t stolen_interrupt_count = 0;
         uint16_t stolen_loop_count = 0;

// Command Flags
bool sentSMS = 0,
     location_cmd = 0,
     register_cmd = 0,
     deregister_cmd = 0;

String temp_s = "", r = "";

// Boot Variables
RTC_DATA_ATTR uint16_t boot_cnt = 0;

const uint16_t  boot_cnt_addr = 0,
                boot_cnt_size = sizeof(uint16_t),     // unsigned short
                base_num_addr = boot_cnt_size + 1,    // Starts at EEPROM base OR the first char can immediately be read
                num_spacing   = 0x0D, // 13 Chars     // Gap of uint8_t with 0
                o_num_addr    = (5*num_spacing)+base_num_addr+10, //  owner number starts here
                o_name_addr   = o_num_addr + num_spacing + 2;     //  owner name after owner number + 1
      uint16_t  passwrd_addr;

// Stored like this to prevent overflow
uint32_t  phoneLatLow = 0, phoneLonLow = 0; // Low is to right of decimal pt.
int16_t   phoneLatUp  = 0, phoneLonUp  = 0; // Up  is to left  of decimal pt.

// BLE Connection Flags
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

BLEServer* pServer = NULL;
BLECharacteristic  * pLatCharacteristic = NULL,
                   * pLngCharacteristic = NULL,
                   * pOwnrFirstNameCharacteristic = NULL,
                   * pOwnrLastNameCharacteristic = NULL,
                   * pOwnrNumCharacteristic = NULL;;

// FreeRTOS Variables

TaskHandle_t      Task1;
SemaphoreHandle_t gsmInUseFlag,
                  gpsInUseFlag;

// Structures

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
  unsigned char   gpsFix = 0; // -        GPSfix Type, range 0..5
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

// ISR Functions

void IRAM_ATTR interruptServiceRoutineMpu(){
  if(mode == SENTRY_MODE) {
    stolen=1;
    stolen_interrupt_count++;
    Serial<<"---> Stolen "<< endl;
  }
  else if(mode == TRAVEL_MODE){
    crashed = 1;
    mode = ALERT_MODE;
    Serial<<"---> Crashed "<< endl;
  }
}

void IRAM_ATTR interruptServiceRoutineRing(){
  if(!ringPin){
    Serial <<"\n---> RING PIN HIGH \n";
    ringPin = 1;
  }
  else Serial <<".";
}

// BLE Functions

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
    // static bool gotNum = false, gotFirst = false, gotLast = false;
    if(pCharacteristic == pOwnrNumCharacteristic){
      std::string Num = pCharacteristic->getValue();
      uint8_t i = 0;
      while(Num.length() > i){
        owner_number += Num[i];
        i++;
      }
      owner_number.trim();
      // if(gsmIsValidNumber(owner_number))
      //   gotNum = true;
      // else
      //   gotNum = false;
      
    }
    if(pCharacteristic == pOwnrFirstNameCharacteristic){
      std::string first = pCharacteristic->getValue();
      uint8_t i = 0;
      while(first.length() > i){
        owner_first_name += first[i]; 
        i++;
      }
      owner_first_name.trim();
    }
    if(pCharacteristic == pOwnrLastNameCharacteristic){
      std::string last = pCharacteristic->getValue();
      uint8_t i = 0;
      while(last.length() > i){
        owner_last_name += last[i];
        i++;
      }
      owner_last_name.trim();
    }
  }
};

class MyServerCallbacks: public BLEServerCallbacks {
  void onConnect(BLEServer* pServer) {
    deviceConnected = true;
    // mode = TRAVEL_MODE;
  }

  void onDisconnect(BLEServer* pServer) {
    deviceConnected = false;
    // mode = SENTRY_MODE;
  }
};

// GPS Functions

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

bool processGPS(uint8_t payloadSize) {  // It's going to wait for GPS Data and process it
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

void requestGPS( bool state ){  //  It sends a command out to start or stop GPS Data
  # if DEBUG > 0
  Serial<<_FMT("\nR <-- POSLLH  Periodic % Request",(state)? F("START") : F("STOP"));
  # endif
  for(uint8_t i = 0; i < 11; i++){
    if(state){
      GpsUart.write(posllh_start[i]);
      # if DEBUG > 0
      //Serial.write(posllh_start[i]);
      # endif
    }
    else{
      GpsUart.write(posllh_stop[i]);
      # if DEBUG > 0
      //Serial.write(posllh_stop[i]);
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
//      Serial.write(sol_start[i]);
      # endif
    }
    else{
      GpsUart.write(sol_stop[i]);
      # if DEBUG > 0
//      Serial.write(sol_stop[i]);
      # endif
    }
  }
  if(!state){
    delay(100);
    GpsUart.flush();
  }
}

bool updateGpsData(){
  uint32_t not_match_counter_last = not_match_counter;
  if(processGPS(sizeof(NAV_POSLLH))){
    # if DEBUG > 0
    Serial<< "Got POSLLH Data" << endl
          << "Lat: "    << _FLOAT(Posllh.lat/10000000.F, 7) << F(" N   ") 
          << "Lon: "    << _FLOAT(Posllh.lon/10000000.F, 7) << F(" E   ") 
          << "iTOW: "   << Posllh.iTOW    << endl;
    #endif
  }
  if(processGPS(sizeof(NAV_SOL))){
    # if DEBUG > 0
    Serial<<"Got SOL Data"<<endl;
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
  }
  return(!(not_match_counter-not_match_counter_last>=1));
}

// GSM Functions

String gsmUartSend(String incoming) { //Function to communicate with SIM800 module
  GsmUart<<incoming<<endl;
  uint32_t t = millis();
  String result = "";
  while(!GsmUart.available()){delay(500);if(millis()-t>5000)return result;}
  if(GsmUart.available()){ //Wait for result
    result = GsmUart.readString();
    result.trim();
  }
//  incoming.trim();
  # if DEBUG > 0
  String redone_result = result;
  uint8_t i = 0;
  while(redone_result.indexOf("\n")>0)redone_result.replace("\n", " ");
  Serial<< incoming << " - " 
        << redone_result   << endl;
  # endif
  return result; //return the result
}

bool gsmConnectionCheck() { // Commands sent to verify connectivity
  String s = gsmUartSend("AT+CSQ");
  if(s.indexOf("OK",5)<0) return false;
  s = gsmUartSend("AT+CCID");
  if(s.indexOf("OK",5)<0) return false;
  s = gsmUartSend("AT+CREG?");
  if(s.indexOf("OK",5)<0) return false;
  return true;
}

void gsmSleep(bool sleep) { // Sleep Commands
  digitalWrite(DTR_PIN, HIGH);
  if(sleep){
  
    # if DEBUG > 0
    Serial<<endl<<"Putting GSM to sleep"<<endl;
    # endif
    while(!gsmVerify(gsmUartSend("AT+CSCLK = 1")));
    # if DEBUG > 0
    Serial<<"GSM is asleep."<<endl;
    # endif
  }
  else{
    # if DEBUG > 0
    Serial<<endl<<"Waking GSM from sleep"<<endl;
    # endif
    GsmUart<<1; // Works without this but put just in case because the 
                // below command takes some time also to send and recieve.
    while(!gsmVerify(gsmUartSend("AT+CSCLK=0")));
    # if DEBUG > 0
    Serial<<"GSM is awake."<<endl;
    # endif
  }
  digitalWrite(DTR_PIN, LOW);
}

bool gsmVerify(String r) {  // Helper verification function
//  return (r == "OK");
  return (r.indexOf("ERROR")<0);  // If no error .indexOf returns -1 Implies expression is true
}

bool gsmIsValidNumber(String number){ // Helper number verification function
  if(number.length()==13){
    if(number.startsWith("+")){
      # if DEBUG > 0
      Serial<<endl<<"valid Number"<<endl;
      # endif
      return(true);
    }
  }
  # if DEBUG > 0
  Serial<<endl<<"Invalid Number"<<endl;
  # endif
  return(false);
}

bool gsmSendSmsToNumber(String number, String sms, bool wake_before = true, bool sleep_after = true) { // Send SMS function
  if(gsmIsValidNumber(number)){
    if(wake_before)gsmSleep(OFF);
    # if DEBUG > 0
    Serial<<endl<<"Sending SMS..."<<endl;
    # endif
    while(!gsmVerify(gsmUartSend("AT+CMGF=1"))); //Set the module in SMS mode
    GsmUart<<"AT+CMGS="<<"\""<<number<<"\""<<endl;
    uint32_t t = millis();
    while(!GsmUart.available()){
      delay(500);
      if(millis()-t>5000)return(false);
    }
    String result = "";
    if(GsmUart.available()){ //Wait for result
      result = GsmUart.readString();
      result.trim();
    }
    if(result == ">"){
      sms.trim();
      GsmUart << sms << (char)26;
    }
    else return(false);
    t = millis();
    while(!GsmUart.available()){
      delay(500);
      if(millis()-t>5000)return(false);
    }
    result = "";
    if(GsmUart.available()){ //Wait for result
      result = GsmUart.readString();
      result.trim();
    }
    if(!gsmVerify(result))return(false);
    if(sleep_after)gsmSleep(ON);
    return(true);
  }
  return(false);
}

bool gsmSendSms(uint8_t num, String sms, bool wake_before = true, bool sleep_after = true){ // Alternative way to send SMS only to verified numbers
  return(gsmSendSmsToNumber(getNumberOfIndex(num), sms, wake_before, sleep_after));
}

String gsmReadSms(bool wake_before = true, bool sleep_after = true) { // Read SMS function
  if(wake_before)gsmSleep(OFF);
  String  result = "", 
          number = "", 
          content = "";
  result = gsmUartSend("AT+CMGR=1");
  int8_t i = result.indexOf(",\"+"), j;
  if(i>0) {
    number = result.substring(i+2,i+15);
    i = result.indexOf("?");
    j = result.indexOf("\n", i+1);
    content = result.substring(i,j+1);
    # if DEBUG > 0
    Serial<< "\nNumber: "  << number  //<< " " << i << " " << j
          << "\nContent: " << content << endl;
    # endif
  }
  gsmUartSend("AT+CMGD=1");
  ringPin = 0;
  if(sleep_after)gsmSleep(ON);
  return(number + ":" + content);
}

bool gsmCallIdSet() { 
  //if(!gsmVerify(gsmUartSend("AT+CLIP=1")))return(false);  // Caller ID
  if(!gsmVerify(gsmUartSend("AT+CMGF=1")))return(false);  // Set the SMS format to be readable
  return(true);
}

bool registerNewNumber( String new_num, String old_num = "0000000000000"){
  uint8_t i = 0;
  if(new_num.length() == phoneNumbers[0].length()){
    //while(EEPROM.readString((i*num_spacing)+base_num_addr+(i*2)) != old_num)i++;
    // while(phoneNumbers[i] != old_num)i++;
    i = getIndexOfNumber(old_num);
    EEPROM.writeString((i*num_spacing)+base_num_addr+(i*2), new_num);
    phoneNumbers[i] = new_num;
    EEPROM.commit();
    #if DEBUG>0
    Serial<< "Registered: " << new_num <<endl;
    #endif
    return(true);
  }
  #if DEBUG>0
  Serial<< "Failed to Register: " << new_num <<endl
        << "Reason: " << ((i==0)? "Wrong number size." : "No space or Old Num not found.") <<endl;
  #endif
  return(false); 
}

uint8_t getIndexOfNumber(String num){
  uint8_t i = 0;
  for(; phoneNumbers[i] != num && i<5; i++);
  return(i);
}

String  getNumberOfIndex(uint8_t index){
  return(phoneNumbers[index]);
}

void updatePhoneNumbersOnWake(){
  uint8_t i = 0;
  for(i=0; i<5; i++){
    phoneNumbers[i] = EEPROM.readString((i*num_spacing)+base_num_addr+(i*2));
    Serial<<"Phone Number["<<i<<"]: "<< phoneNumbers[i]<<endl;
  }
  owner_number = EEPROM.readString(o_num_addr);
  owner_name   = EEPROM.readString(o_name_addr);
  Serial << "Owner Name: " << owner_name<<endl
         << "Owner Number: " << owner_number<<endl;
}

bool gsmParseCommand(String cmd){
  String source = cmd.substring(0,13), command = cmd.substring(14);
  uint8_t i = getIndexOfNumber(source);
  command.trim();
  #if DEBUG>0
  Serial<<endl<< "Parsing Command" << endl
              << "source:  " << source  << endl
              << "command: " << command << endl
              << "index:   " << i       << endl;
  #endif
  if(i<5) {  // If number is registered.
    #if DEBUG>0
    Serial<< "Registered number.\n";
    #endif
    if(command == "?location"){
      #if DEBUG>0
      Serial<< "Location..." <<endl;
      #endif
      requestGPS(ON);
      bool reliable = updateCoords();
      requestGPS(OFF);
      // xSemaphoreTake( gsmInUseFlag, portMAX_DELAY );
      if(!reliable)temp_s += "\nUnreliable Coordinates.";
      gsmSendSmsToNumber(source, temp_s, 0, 1);
      // xSemaphoreGive( gsmInUseFlag );
      // gsmSendSmsToNumber(source, "PLACEHOLDER", 0, 1);
    }
    else if(command.substring(0,12) == "?deregister "){  // If number exists and command is to deregister then remove from list.
      #if DEBUG>0
      Serial<< "Deregister..." <<endl;
      #endif
      String pswd = command.substring(12);
      pswd.trim();
      #if DEBUG>0
      Serial<< "Password: " << pswd <<endl;
      #endif
      if(pswd == PSWD){ // If pswd is correct deregister
        deregister_cmd = 1;
        if(registerNewNumber("0000000000000", source)){
          // xSemaphoreTake( gsmInUseFlag, portMAX_DELAY );
          gsmSendSmsToNumber(source, "Deregistered successfully", 0, 1);
          #if DEBUG>0
          Serial<<"Sent Deregistered SMS"<<endl;
          #endif
          // xSemaphoreGive( gsmInUseFlag );
        }
        else{
          // xSemaphoreTake( gsmInUseFlag, portMAX_DELAY );
          // gsmSendSmsToNumber(source, "Wrong Password", 0, 1);
          #if DEBUG>0
          Serial<<"Wrong Password"<<endl;
          #endif
          // xSemaphoreGive( gsmInUseFlag );
        }
        
      }
    }
    else if(command == "?help"){
      #if DEBUG>0
      Serial<< "Help..." <<endl;
      #endif
      String sms = "--COMMANDS--\n";
      // xSemaphoreTake( gsmInUseFlag, portMAX_DELAY );
      gsmSendSmsToNumber(source, sms, 0, 0);
      sms = "?location (1/4)\n";
      gsmSendSmsToNumber(source, sms, 0, 0);
      sms = "?register <password> (2/4)\n";
      gsmSendSmsToNumber(source, sms, 0, 0);
      sms = "?deregister <password> (3/4)\n";
      gsmSendSmsToNumber(source, sms, 0, 0);
      sms = "?help (4/4)";
      gsmSendSmsToNumber(source, sms, 0, 1);
      Serial<<"Sent Help SMS"<<endl;
      // xSemaphoreGive( gsmInUseFlag );
    }
    else  {
      gsmSleep(ON);
      return(0);
    }
    #if DEBUG>0
    Serial<< "Parsed command." <<endl;
    #endif
    return(1);
  }
  else if(command.substring(0,10) == "?register "){  // If number does not exist and command is to register then add to list
    #if DEBUG>0
    Serial<< "Registering..." <<endl;
    #endif
    String pswd = command.substring(10);
    pswd.trim();
    if(pswd == PSWD){ // If pswd is correct register
      register_cmd = 1;
      // xSemaphoreTake( gsmInUseFlag, portMAX_DELAY );
      if(registerNewNumber(source)){ // Registered successfully or not.
        gsmSendSmsToNumber(source,"Registered successfully.", 0, 1);
        #if DEBUG>0
        Serial<<"Sent Registration Confirmation SMS"<<endl;
        #endif
        // xSemaphoreGive( gsmInUseFlag );
        return(1);
      }
      else{
        gsmSendSmsToNumber(source,"Failed to Register", 0, 1);
        #if DEBUG>0
        Serial<<"Sent Failed to Register SMS"<<endl;
        #endif
        // xSemaphoreGive( gsmInUseFlag );
        return(0);
      }
    }
    else{
      // xSemaphoreTake( gsmInUseFlag, portMAX_DELAY );
      gsmSendSmsToNumber(source, "Wrong Password", 0, 1);
      #if DEBUG>0
      Serial<<"Wrong Password"<<endl;
      #endif
      // xSemaphoreGive( gsmInUseFlag );
      return(0);
    }
    
  }
  else if(command == "?help"){
      #if DEBUG>0
      Serial<< "Help..." <<endl;
      #endif
      String sms = "--COMMANDS--\n";
      // xSemaphoreTake( gsmInUseFlag, portMAX_DELAY );
      gsmSendSmsToNumber(source, sms, 0, 0);
      sms = "?location (1/4))\n";
      gsmSendSmsToNumber(source, sms, 0, 0);
      sms = "?register <password> (2/4)\n";
      gsmSendSmsToNumber(source, sms, 0, 0);
      sms = "?deregister <password> (3/4)\n";
      gsmSendSmsToNumber(source, sms, 0, 0);
      sms = "?help (4/4)";
      gsmSendSmsToNumber(source, sms, 0, 1);
      // xSemaphoreGive( gsmInUseFlag );
    }
  else {
    #if DEBUG>0
    Serial<<"Unregistered number OR Wrong password entered."<<endl;
    #endif
    // xSemaphoreTake( gsmInUseFlag, portMAX_DELAY );
    gsmSendSmsToNumber(source, "Unregistered number OR Wrong password entered.", 0, 1);
    // xSemaphoreGive( gsmInUseFlag );
    return(0);
  }
}

// EEPROM Functions

void eepromDump(){
  Serial<< "-X-X-EEPROM DATA-X-X-"   << endl
        << "Boot Count: "  << EEPROM.readUShort(0)                                << endl
        << "Num 1 - "      << EEPROM.readString((0*num_spacing)+base_num_addr+0)  << endl
        << "Num 2 - "      << EEPROM.readString((1*num_spacing)+base_num_addr+2)  << endl
        << "Num 3 - "      << EEPROM.readString((2*num_spacing)+base_num_addr+4)  << endl
        << "Num 4 - "      << EEPROM.readString((3*num_spacing)+base_num_addr+6)  << endl
        << "Num 5 - "      << EEPROM.readString((4*num_spacing)+base_num_addr+8)  << endl
        <<"Owner Number - "<< EEPROM.readString(o_num_addr)                       << endl
        <<"Owner Name - "  << EEPROM.readString(o_name_addr)                      << endl;
}

void registerOwner(String name, String number){// Helper function
  EEPROM.writeString(o_num_addr, number);
  EEPROM.writeUChar (o_name_addr-1, 0   );
  EEPROM.writeString(o_name_addr, name  );
  EEPROM.commit();
}

void getOwnerData(){
  Serial<<"Owner Number: "<<endl;
  while(!Serial.available());
  owner_number = Serial.readString();
  owner_number.trim();
  Serial<<"Owner Name: "<<endl;
  while(!Serial.available());
  owner_name = Serial.readString();
  owner_name.trim();
  registerOwner(owner_name, owner_number);
}

void printDebugData(){
  Serial<< "---DEBUG_DATA---\n"
  << "ringPin: "<< ringPin <<endl
  << "stolen: " << stolen  <<endl
  << "crashed: "<< crashed <<endl
  << "lastCrashed: "<< lastCrashed <<endl
  << "stolen_interrupt_count: "<< stolen_interrupt_count <<endl
  << "stolen_loop_count: "<< stolen_loop_count <<endl
  << "deviceConnected: " << deviceConnected <<endl
  << "oldDeviceConnected: " << oldDeviceConnected <<endl
  << "mode :"<< mode <<endl
  << "last: "<< last  <<endl;
}

// MPU6050 Functions

void mpuSensitivityChange(){
  switch (mode) {
    case TRAVEL_MODE:
      // Motion detection acceleration threshold. 1LSB = 2mg.
      accelgyro.setMotionDetectionThreshold(ACCIDENT_MOTION_THRESHOLD);
      // Motion detection event duration in ms
      accelgyro.setMotionDetectionDuration(ACCIDENT_MOTION_EVENT_DURATION);
      break;
    default:
      // Motion detection acceleration threshold. 1LSB = 2mg.
      accelgyro.setMotionDetectionThreshold(SENTRY_MOTION_THRESHOLD);
      // Motion detection event duration in ms
      accelgyro.setMotionDetectionDuration(SENTRY_MOTION_EVENT_DURATION);
      break;
  }
}

// Free RTOS functions

void loop2(void * parameter){
  for (;;){
    if(ringPin && mode != ALERT_MODE ){
      Serial<<"\n\nGot into loop2\n\n";
      detachInterrupt(INTERRUPT_RING);
//       detachInterrupt(INTERRUPT_MPU);
      xSemaphoreTake( gsmInUseFlag, portMAX_DELAY );
      delay(10);
      if(GsmUart.available()){
        temp_s = GsmUart.readString();
        Serial << "Gsm Available: "<< temp_s <<endl;
        temp_s.trim();
        if(temp_s.startsWith("RING"))
          while(!gsmVerify(gsmUartSend("ATH")))Serial<<"Failed to Cut call."<<endl;
        else{
          Serial << "Parsing SMS" << endl;
          gsmParseCommand(gsmReadSms(1, 0));
        }
        delay(50);
        Serial << "Flushing GSM" << endl;
        GsmUart.flush();
      }
      delay(50);
      GsmUart.flush();
      xSemaphoreGive( gsmInUseFlag );
      ringPin = 0;
//       attachInterrupt(INTERRUPT_MPU, interruptServiceRoutineMpu, RISING);
      attachInterrupt(INTERRUPT_RING, interruptServiceRoutineRing, FALLING);
    }
    delay(1000);
  }
}

// Loop Convienience Functions

bool updateCoords(){
  double  lat, lon;
  uint8_t counter = 0;
  Serial<<endl<<"updateCoords"<<endl;
  Sol.gpsFix = 0;
  while(true){
    Serial<<"gpsloop"<<endl;
    bool valid = updateGpsData();
    counter++;
    if(counter>50){break;}
    if(valid && Sol.gpsFix){
      Serial<<"gotLock: "<<Sol.gpsFix<<endl;
      break;
    }
    Serial<<endl<<counter<<endl;
  }
  if(Sol.gpsFix){
    lat = Posllh.lat/10000000.F;
    lon = Posllh.lon/10000000.F;
    #if DEBUG > 0
    Serial<<"Got GPS Coords from GPS module"<< endl
          << BaseLink << _FLOAT(lat, 7) <<","<< _FLOAT(lon, 7) << endl;
    #endif
    temp_s = BaseLink +  String(lat, 7) + "," + String(lon, 7);
    return(1);
  }
  else{
    temp_s = BaseLink + phoneLatUp + "." + phoneLatLow + "," + phoneLonUp + "." + phoneLonLow;
    #if DEBUG > 0
    Serial<<"Using Phone Location"<< endl
          <<        temp_s        << endl;
    #endif
    return(0);
  }
}

// Setup Functions

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
  mpuSensitivityChange();
  delay(100);
}

void setupGsmModule(){
  while(!gsmVerify(gsmUartSend("AT")));
  while(!gsmVerify(gsmUartSend("ATE0")));
  while(!gsmVerify(gsmUartSend("AT+IPR=115200")));
  while(!gsmVerify(gsmUartSend("AT+CMGF=1")));
  while(!gsmConnectionCheck());
  while(!gsmCallIdSet());
}

BLEService* setupInitialService(){
  // Create the BLE Service
  BLEService* pInitialService = pServer->createService( INITIAL_SERVICE_UUID );
  
  // Create a BLE Characteristic
  pOwnrFirstNameCharacteristic =  pInitialService->createCharacteristic(
                                  OWNER_FIRST_NAME_CHARACTERISTIC_UUID,
                                  BLECharacteristic::PROPERTY_WRITE   |  
                                  BLECharacteristic::PROPERTY_READ   
                                  );
  pOwnrLastNameCharacteristic =   pInitialService->createCharacteristic(
                                  OWNER_LAST_NAME_CHARACTERISTIC_UUID,
                                  BLECharacteristic::PROPERTY_WRITE   |  
                                  BLECharacteristic::PROPERTY_READ   
                                  );
  pOwnrNumCharacteristic      =   pInitialService->createCharacteristic(
                                  OWNER_NUM_CHARACTERISTIC_UUID,
                                  BLECharacteristic::PROPERTY_READ     |
                                  BLECharacteristic::PROPERTY_WRITE
                                  );
  pInitialService->start();
  delay(10);
  // Start advertising
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(INITIAL_SERVICE_UUID);
  pAdvertising->setScanResponse(false);
  pAdvertising->setMinPreferred(0x0);
  pOwnrFirstNameCharacteristic->setCallbacks(new InitialSetupCharacteristicCallbacks());
  pOwnrLastNameCharacteristic ->setCallbacks(new InitialSetupCharacteristicCallbacks());
  pOwnrNumCharacteristic      ->setCallbacks(new InitialSetupCharacteristicCallbacks());
  BLEDevice::startAdvertising();
  deviceConnected = false;
  delay(10);
  return pInitialService;
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
  pAdvertising->addServiceUUID( GPS_SERVICE_UUID );
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
  pinMode(LED_PIN,        OUTPUT);
  digitalWrite(LED_PIN, HIGH);
  pinMode(INTERRUPT_MPU,  INPUT_PULLDOWN);  
  pinMode(INTERRUPT_RING, INPUT_PULLUP);
  Wire.begin(SDA_PIN, SCL_PIN );
  // #if DEBUG>0
  Serial.begin(115200);
  // #endif
  GpsUart.begin(115200, SERIAL_8N1, RX1, TX1);
  GsmUart.begin(115200, SERIAL_8N1, RX2, TX2);
  if (!EEPROM.begin(1000)) {
    Serial.println("Failed to initialise EEPROM");
    Serial.println("Restarting...");
    delay(1000);
    ESP.restart();
  }
   boot_cnt = EEPROM.readUShort(0); 
//  boot_cnt = 0;
  if(boot_cnt == 0){ // Implies the system restarted
    mode = INITIAL_SETUP;
    Serial<<"First Start, waiting for 5 seconds to let everything settle"<<endl;
    EEPROM.writeString((0*num_spacing)+base_num_addr+0, "0000000000000");
    EEPROM.writeUChar ((1*num_spacing)+base_num_addr+1, 0);
    EEPROM.writeString((1*num_spacing)+base_num_addr+2, "0000000000000");
    EEPROM.writeUChar ((2*num_spacing)+base_num_addr+3, 0);
    EEPROM.writeString((2*num_spacing)+base_num_addr+4, "0000000000000");
    EEPROM.writeUChar ((3*num_spacing)+base_num_addr+5, 0);
    EEPROM.writeString((3*num_spacing)+base_num_addr+6, "0000000000000");
    EEPROM.writeUChar ((4*num_spacing)+base_num_addr+7, 0);
    EEPROM.writeString((4*num_spacing)+base_num_addr+8, "0000000000000");
    delay(5000);
  }
  else{
    Serial<<"Boot Count: "<< boot_cnt <<endl
          <<"Updating Phone Numbers"  <<endl;
    updatePhoneNumbersOnWake();
    mode = SENTRY_MODE;
  }

  setupGsmModule();
  gsmSleep(ON);
  Serial<< "--> SIM800L initialised." << endl;

  setupAccelGyro();
  Serial<< "--> MPU6050 initialised." << endl;

  // Create the BLE Device
  BLEDevice::init("ESP32 Crash Detector");
  // Create the BLE Server
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());
  if(mode == INITIAL_SETUP){
    BLEService* pInitialService = NULL;
    pInitialService = setupInitialService();
    while(!deviceConnected){
      digitalWrite(LED_PIN, millis()%1000>900);
      delay(200);
    }
    Serial<<"Connected to owners phone."<<endl;
    while (deviceConnected){ //break out when got details of owner
      delay(1000);
      digitalWrite(LED_PIN, millis()%2000>1800);
      if(owner_number!="" && owner_first_name!="" && owner_last_name!=""){
        delay(2000);
        pInitialService->stop();
        pInitialService->executeDelete();
        break;
      }
    }
    owner_name = owner_first_name + " " + owner_last_name;
    Serial << "owner_name: "   << owner_name   << endl
           << "owner_number: " << owner_number << endl; 
    registerOwner(owner_name, owner_number);
  }
  setupGpsService();
  Serial<< "--> Gps Service initialised." << endl;
  // #if DEBUG>0
  // Serial<< "--> SIM800L initialised." << endl;
  // Serial<< "--> Gps Service initialised." << endl;
  // Serial<< "--> MPU6050 initialised." << endl;
  // Serial<< "--> BLE server initialised." << endl;
  // #endif
  boot_cnt++;
  EEPROM.writeUShort(boot_cnt_addr, boot_cnt);
  EEPROM.commit();
  gsmInUseFlag = xSemaphoreCreateMutex();
  gpsInUseFlag = xSemaphoreCreateMutex();

  xTaskCreatePinnedToCore( // Gsm Read Task
    loop2,
    "GsmTask",
    4000,     //  Stack Size 
    NULL,
    2,        //  Priority
    &Task1,   //  Task Handler
    1         //  Core 2
  );

  delay(500);// To let FreeRTOS to initialize and handle the new task.
  attachInterrupt(INTERRUPT_MPU , interruptServiceRoutineMpu , RISING  );
  attachInterrupt(INTERRUPT_RING, interruptServiceRoutineRing, FALLING );
  delay(1);
  digitalWrite(LED_PIN,LOW);
  mode = SENTRY_MODE;
}

void loop() {
  if(Serial.available()){
    r = Serial.readString();
    r.trim();
    Serial<<""<<endl;
    if(r == "0") getOwnerData();
    if(r == "1") printDebugData();
    if(r == "2") eepromDump();
  }

  switch(mode){
    case TRAVEL_MODE:
      // Disconnecting
      if(!deviceConnected &&  oldDeviceConnected){  //  If disconnected then advertise the BLE name and SENTRY mode
        #if DEBUG>0
        delay(200); // give the bluetooth stack the chance to get things ready
        Serial<< ".";
        delay(200);
        Serial<< ".";
        delay(200);
        Serial<< "." << endl
              << "--> Start Advertising Device Name" << endl;
        #elif
        delay(600);
        #endif
        // requestGPS(ON);
        // updateCoords(); // To Prep for being stolen
        // requestGPS(OFF);
        pServer->startAdvertising(); // restart advertising
        oldDeviceConnected = deviceConnected; // oldDeviceConnected = false
        mode = SENTRY_MODE;
        mpuSensitivityChange();
      }
      // Notify changed value
      if( deviceConnected &&  oldDeviceConnected){  //  Device is connected
        // phoneLoc variables are updated asychronously but require delay so as to not 
        // slow it down (if BLE code runs on core 0 but it most likely runs on core 1)
        delay(1000);
        digitalWrite(LED_PIN, !digitalRead(LED_PIN));
      }
      break;
    
    case ALERT_MODE:
      if(!lastCrashed &&  crashed) { //  Request GPS Module to start spewing out the location
        xSemaphoreTake( gpsInUseFlag, portMAX_DELAY );
        requestGPS(ON);
        digitalWrite(LED_PIN, HIGH);
        lastCrashed = crashed;
      }
      if( lastCrashed && !crashed) { //  Stop the GPS module giving out location
        // requestGPS(OFF); // Put after Update coord cuz don't want buffer overflow
        GpsUart.flush();
        digitalWrite(LED_PIN, LOW);
        lastCrashed = crashed;
        mode = TRAVEL_MODE;
      }
      if( lastCrashed &&  crashed) { //  Read the GPS location and if no coord fix then depend on the last phoneLocation
        crashed = 0;
        bool reliable = updateCoords();
        requestGPS(OFF);
        xSemaphoreGive( gpsInUseFlag );
        xSemaphoreTake( gsmInUseFlag, portMAX_DELAY );
        uint8_t   number_counter = 0;
        temp_s += "\nPossible vehicle crash detected.";
        if(!reliable)temp_s += "\nUnreliable Coordinates.";
        for(number_counter = 0; number_counter < 5; number_counter++) 
          if(gsmIsValidNumber(getNumberOfIndex(number_counter)) && gsmSendSms(number_counter, temp_s))
            number_counter--;
        xSemaphoreGive( gsmInUseFlag );
      }
      else delay(100);
      break;
    
    case SENTRY_MODE: 
      // Device has been bothered
      if(stolen){ // Send SMS to the owner
        // Send every SENTRY_INTERVAL seconds as long as it is in motion;
        if((uint32_t)millis()-last>SENTRY_INTERVAL  && stolen_interrupt_count>=stolen_loop_count){
          Serial<<"stolen_interrupt_count: "<<stolen_interrupt_count<<"\t"<<"stolen_loop_count: "<<stolen_loop_count<<endl;
          if(owner_number.length()>1){
            detachInterrupt(INTERRUPT_MPU);
            detachInterrupt(INTERRUPT_RING);
            xSemaphoreTake( gpsInUseFlag, portMAX_DELAY );
            requestGPS(ON);
            bool reliable = updateCoords();
            requestGPS(OFF);
            xSemaphoreGive( gpsInUseFlag );
            xSemaphoreTake( gsmInUseFlag, portMAX_DELAY );
            temp_s += "\nPossible vehicle theft detected.";
            if(!reliable)temp_s += "\nUnreliable Coordinates.";
            while(!gsmSendSmsToNumber(owner_number, temp_s))Serial<<"Failed to send SMS to "<<owner_name<<endl;
            xSemaphoreGive( gsmInUseFlag );
            attachInterrupt(INTERRUPT_RING, interruptServiceRoutineRing, FALLING);
            attachInterrupt(INTERRUPT_MPU, interruptServiceRoutineMpu, RISING);
          }
          stolen_loop_count++;
          last = millis();
        }
        else if(stolen_interrupt_count < stolen_loop_count){ // Stopped motion so stop sending data
          stolen_loop_count = 0; 
          stolen_interrupt_count = 0;
          stolen = 0;
        }
        else{
          delay(100);
          Serial<<"Stuck here"<<endl;
        }
      }
      // Wait for connection
      if (!stolen && !deviceConnected && !oldDeviceConnected) { //  Wait till device is connected
        #if DEBUG > 0
        static bool wait_till_connected = true;
        if(wait_till_connected)Serial<< "--> Waiting for connection";
        wait_till_connected = false;
        #endif
        digitalWrite(LED_PIN, !digitalRead(LED_PIN));
        if(digitalRead(LED_PIN)) delay(100);
        else{
          delay(900);
          #if DEBUG > 0
          Serial<<(".");
          #endif
        }
        // #if DEBUG > 0
        // if(!wait_till_connected)Serial<<endl;
        // #endif
        // digitalWrite(LED_PIN, LOW);
      }
      // Connecting
      if (!stolen &&  deviceConnected && !oldDeviceConnected) { //  Device is connecting so switch from SENTRY to TRAVEL
        // do stuff here on connecting
        oldDeviceConnected = deviceConnected; // oldDeviceConnected = true
        mode = TRAVEL_MODE;
        mpuSensitivityChange();
        #if DEBUG>0
        Serial<< F("--> Connecting\n");
        #endif
      }
      break;
  }
}
