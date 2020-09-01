#include <HardwareSerial.h>
#include <Streaming.h>

#define RX1 GPIO_NUM_32   //  GPS TX
#define TX1 GPIO_NUM_33   //  GPS RX

HardwareSerial PeekUart(1);

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  PeekUart.begin(115200, SERIAL_8N1, RX1, TX1);
}

void loop() {
  // put your main code here, to run repeatedly:
  int b = 0;
  while(!Serial.available());
  Serial<<"Got Data"<<endl;
  while( Serial.available()){
    b = Serial.read();
    Serial.write(b);
    PeekUart.write(b);
  }
  b = 0;
  while(!PeekUart.available());
  while( PeekUart.available()){
    b = PeekUart.read();
    Serial.write(b);
  }
  Serial<<"Recieved Data"<<endl;
}

/*#include <HardwareSerial.h>
#include <Streaming.h>

#define RX1 GPIO_NUM_32   //  GPS TX
#define TX1 GPIO_NUM_33   //  GPS RX

HardwareSerial PeekUart(1);

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  PeekUart.begin(115200, SERIAL_8N1, RX1, TX1);
}

void loop() {
  // put your main code here, to run repeatedly:
  byte b[] = {};
  uint8_t i = 0;
  while(!PeekUart.available());
  Serial<<"Got Data"<<endl;
  while( PeekUart.available()){
    b[i] = PeekUart.read();
    Serial<<_HEX(b[i]);
    i++;
  }
  Serial<<endl<<"Done"<<endl;
  while(1){
    for(int j = 0; j < i; j++)
      Serial<<_HEX(b[j]);
    Serial<<endl;
    delay(7000);
  }
}*/
