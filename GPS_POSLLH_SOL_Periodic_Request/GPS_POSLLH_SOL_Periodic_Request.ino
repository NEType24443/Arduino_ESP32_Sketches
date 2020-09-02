#include <HardwareSerial.h>
#include <Streaming.h>

#define DEBUG 1   // 1 = DEBUG OUTPUT, 0 = NO DEBUG OUTPUT

#define LED_PIN       GPIO_NUM_2

#define DIP_SWITCH    GPIO_NUM_5   // SWITCH_2 IS reserved why?

#define RX1           GPIO_NUM_32   //  GPS TX
#define TX1           GPIO_NUM_33   //  GPS RX

#define RX2           GPIO_NUM_25   //  SIM800L TX 
#define TX2           GPIO_NUM_26   //  SIM800L RX

HardwareSerial GpsUart(1);
  
#define NO_FIX  0x00  //  NO-FIX
#define DR      0x01  //  DEAD RECKONING
#define _2D_Fix 0x02  //  2D-FIX
#define _3D_Fix 0x03  //  3D-FIX
#define GPS_DR  0x04  //  GPS + DEAD RECKONING
#define TOF     0x05  //  TIME ONLY FIX

const unsigned char UBX_HEADER[] = {0xB5, 0x62};

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
  memset(CK, 0, 2); //  Setting the 2 bytes of the checksum array as zero
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
  Serial<<F("\nC --> No checksum match !")<<endl
        <<F("Match Counter")<<_WIDTH(match_counter,7)
        <<F("\tNot Match Counter")<<_WIDTH(not_match_counter,7)<<endl;
  # endif
  return false;
}

void requestGPS( bool state ){
  Serial<<_FMT("\nR <-- POSLLH  Periodic % Request",(state)? F("START") : F("STOP"));
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
  delay(10);
  Serial<<_FMT("\nR <-- SOL Periodic % Request",(state)? F("START") : F("STOP"));
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

void setup() {
  Serial.begin(115200);
  GpsUart.begin(115200, SERIAL_8N1, RX1, TX1);
  pinMode(LED_PIN, OUTPUT);
  pinMode(DIP_SWITCH,INPUT);
  //delay(5000); // FOR ANTENNA RESET ON POWER ON
}


bool last_switch = 0;


void loop() {
  if(!last_switch &&  digitalRead(DIP_SWITCH)){   
    requestGPS(1);
    digitalWrite(LED_PIN, HIGH);
    last_switch = 1;
    delay(5);
  }
  if( last_switch && !digitalRead(DIP_SWITCH)){   
    requestGPS(0);
    digitalWrite(LED_PIN, LOW);
    last_switch = 0;
    delay(5);
  }
  while(last_switch && digitalRead(DIP_SWITCH)){
    if(processGPS(sizeof(NAV_POSLLH))){
      # if DEBUG > 0
      Serial<<"Got Data"<<endl;
      # endif
      // digitalWrite(LED_PIN, HIGH);
      //String Lat = String(Posllh.lat/10000000) + "." + String(Posllh.lat%10000000);
      double  lat = Posllh.lat/10000000 + Posllh.lat%10000000;
      //String Lon = String(Posllh.lon/10000000) + "." + String(Posllh.lon%10000000);
      double  lon = Posllh.lon/10000000 + Posllh.lon%10000000;
      Serial<< F("Lat: ")  << _FLOAT(lat, 7) << F(" N   ") 
            << F("Lon: ")  << _FLOAT(lon, 7) << F(" E") << endl
            << F("iTOW: ") << Posllh.iTOW << endl;
      //delay(200);
      //digitalWrite(LED_PIN, LOW);
      //delay(700);
    }
    if(processGPS(sizeof(NAV_SOL))){
      # if DEBUG > 0
      Serial<<"Got Data"<<endl;
      # endif
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
      //delay(200);
      //digitalWrite(LED_PIN, LOW);
      //delay(800);
    }
  }
}
