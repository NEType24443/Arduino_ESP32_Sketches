
// Works without Checksum error if delay between POSLLH requests is greater than or equal to 1.5 second
// Works without Checksum error if delay between SOL requests is greater than or equal to 1.0 second
// Major issue seems to be iming between requests
// Recommend to use the Sketch GPS_POSLLH_SOL_Periodic_Request

#include <Arduino.h>
#include <HardwareSerial.h>
#include <Streaming.h>

#define LED_PIN 2

#define RX1 GPIO_NUM_32   //  GPS TX
#define TX1 GPIO_NUM_33   //  GPS RX

#define RX2 GPIO_NUM_25   //  SIM800L TX 
#define TX2 GPIO_NUM_26   //  SIM800L RX

HardwareSerial GpsUart(1);
  
#define NO_FIX  0x00  //  NO-FIX
#define DR      0x01  //  DEAD RECKONING
#define _2D_Fix 0x02  //  2D-FIX
#define _3D_Fix 0x03  //  3D-FIX
#define GPS_DR  0x04  //  GPS + DEAD RECKONING
#define TOF     0x05  //  TIME ONLY FIX

const unsigned char UBX_HEADER[] = {0xB5, 0x62};

const byte posllh_poll[] = {0xB5, 0x62, 0x01, 0x02, 0x00, 0x00, 0x03, 0x0A}; //  B5 62 01 02 00 00 03 0A   -> POSLLH Poll
const byte    sol_poll[] = {0xB5, 0x62, 0x01, 0x06, 0x00, 0x00, 0x07, 0x16}; //  B5 62 01 06 00 00 07 16   -> SOL Poll

uint32_t match_counter = 0, not_match_counter = 0;

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
//  // Type         Name           Unit     Description (scaling)
  unsigned char   cls;
  unsigned char   id;
  unsigned short  len;        // 
  unsigned long   iTOW;       // ms       GPS time of week of the navigation epoch. See the description of iTOW for details
  long            fTOW;       // ns       Fractional part of iTOW (range: +/-500000). The precise GPS time of week in
                              //          seconds is: (iTOW * 1e-3) + (fTOW * 1e-9)
  short           week;       // weeks    GPS week number of the navigation epoch
  unsigned char   gpsFix;     // -        GPSfix Type, range 0..5
  byte            flags;      // -        Fix Status Flags (see graphic below) //char
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

byte calcChecksum(unsigned char* CK, int payloadSize) {
//  int payloadSize = 0;
  memset(CK, 0, 2); //  Setting the 2 bytes of the checksum array as zero
//  switch(data_struct){
//    case 0:
//      payloadSize = sizeof(NAV_POSLLH);
//      break;
//    case 1:
//      payloadSize = sizeof(NAV_SOL);
//      break;
//  }  
  for (int i = 0; i < payloadSize; i++) {
    switch(payloadSize){
      case 32:
        CK[0] += ((unsigned char*)(&Posllh))[i];
        break;
      case 56:
        CK[0] += ((unsigned char*)(&Sol))[i];
        break;
    }
    CK[1] += CK[0];
  }
}

bool processGPS(int payloadSize) {
  static int fpos = 0; //static
  static unsigned char checksum[2]; //static
//  int payloadSize = 0;
  //GpsUart.flush();
  switch(payloadSize){
    case 32:
      Serial<<F("\nR <-- POSLLH Request");
      for(uint8_t i = 0; i < 8; i++){
        GpsUart.write(posllh_poll[i]);
        Serial.write(posllh_poll[i]);
      }
      break;
    case 56:      
      Serial<<F("\nR <-- SOL Request");
      for(uint8_t i = 0; i < 8; i++){
        GpsUart.write(sol_poll[i]);
        Serial.write(sol_poll[i]);
      }
      break;
  }
  Serial<< F("\nR --> ");
  while (!GpsUart.available());
  while ( GpsUart.available()){
    byte c = GpsUart.read();
    Serial.write(c);
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
          match_counter++;
          Serial<<F("\nC --> Checksum match !")<<endl<<F("Match Counter")<<_WIDTH(match_counter,5)<<F("\tNot Match Counter")<<_WIDTH(not_match_counter,5)<<endl;
          return true;
        }
      }
      // Reset the carriage if it is out of a packet.
      else if ( fpos > (payloadSize + 4) ) {
        Serial<< F("\nE --> Carrige has been reset") <<endl;
        fpos = 0;
      }
    }
  }
  not_match_counter++;
  Serial<<F("\nC --> No checksum match !")<<endl<<F("Match Counter")<<_WIDTH(match_counter,5)<<F("\tNot Match Counter")<<_WIDTH(not_match_counter,5)<<endl;
  return false;
}

void setup() {
  Serial.begin(115200);
  GpsUart.begin(115200, SERIAL_8N1, RX1, TX1);
  pinMode(LED_PIN, OUTPUT);
  delay(1000); // FOR ANTENNA RESET
  GpsUart.flush();
}

void loop() {
  
  if(processGPS(sizeof(NAV_POSLLH))){
    Serial<<"Got Data"<<endl;
    digitalWrite(LED_PIN, HIGH);
    String Lat = String(Posllh.lat/10000000) + "." + String(Posllh.lat%10000000);
    String Lon = String(Posllh.lon/10000000) + "." + String(Posllh.lon%10000000);
    Serial<< "Lat: " << Lat << F(" N  ") << "Lon: " << Lon << F(" E") <<endl
          << "iTOW: " << Posllh.iTOW <<endl;
    delay(200);
    digitalWrite(LED_PIN, LOW);
    delay(700);

    if(processGPS(sizeof(NAV_SOL))){
      Serial<<"Got Data"<<endl;
      digitalWrite(LED_PIN, HIGH);
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
      delay(200);
      digitalWrite(LED_PIN, LOW);
      delay(800);
    }
    else{
      Serial<< F("E --> Could not get NAV-SOL Data\n");
    }
  }
  else{
//    Serial<< "Size of posllh:" << sizeof(NAV_POSLLH) << endl; // 28 bytes + class + id + length = 32
//    Serial<< "Size of sol:"    << sizeof(NAV_SOL) << endl;    // 52 bytes + class + id + length = 56    
    digitalWrite(LED_PIN, HIGH);
    delay(700);
    digitalWrite(LED_PIN, LOW);
    delay(300);
  }
  
}
