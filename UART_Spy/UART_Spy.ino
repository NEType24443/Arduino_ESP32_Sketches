#include <HardwareSerial.h>
#include <Streaming.h>

#define RX1 GPIO_NUM_32   //  GPS TX
#define TX1 GPIO_NUM_33   //  GPS RX

#define RX2 GPIO_NUM_16   //  SIM800L TX 
#define TX2 GPIO_NUM_17   //  SIM800L RX

HardwareSerial PeekUart(1);

void setup() {
  Serial.begin(115200);
  PeekUart.begin(9600, SERIAL_8N1, RX2, TX2);
}

void loop() {
  while( Serial.available()){
    PeekUart.write(Serial.read());
  }
  while( PeekUart.available()){
    Serial.write(PeekUart.read());
  }
}
