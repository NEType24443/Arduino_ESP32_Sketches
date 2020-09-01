byte posllh_poll[] = {0xB5, 0x62, 0x01, 0x02, 0x00, 0x00, 0x03, 0x0A};
void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
}

void loop() {
  // put your main code here, to run repeatedly:
  Serial.write(0xB5);
  Serial.write(0x62);
  Serial.write(0x01);
  Serial.write(0x02);
  Serial.write(0x00);
  Serial.write(0x00);
  Serial.write(0x03);
  Serial.write(0x0A);
  while(true){delay(2000);}
}
