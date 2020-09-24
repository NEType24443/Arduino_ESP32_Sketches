#include <Streaming.h>
#include <EEPROM.h>
#include <HardwareSerial.h>

#define ON    1
#define OFF   0

#define DEBUG ON

#define LED_PIN         GPIO_NUM_2    //  ONBOARD LED

#define INTERRUPT_RING  GPIO_NUM_18   //  SIM800L RING PIN
#define DTR_PIN         GPIO_NUM_5    //  SIM800L DTR  PIN

#define RX2             GPIO_NUM_16   //  SIM800L TX 
#define TX2             GPIO_NUM_17   //  SIM800L RX

//#define GSM_CONFIRMED   "OK"

HardwareSerial GsmUart(2);

bool ringPin = 0;
bool sentSMS ;

String phoneNumbers[5] =  { "0000000000000", 
                            "0000000000000", 
                            "0000000000000", 
                            "0000000000000", 
                            "0000000000000" };
                      //  {"+919448517225", "+919481501967"};

const String  reg_cmd     = "?register",
              req_loc_cmd = "?location";
              
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

bool gsmConnectionCheck() { 
  String s = gsmUartSend("AT+CSQ");
  if(s.indexOf("OK",5)<0) return false;
  s = gsmUartSend("AT+CCID");
  if(s.indexOf("OK",5)<0) return false;
  s = gsmUartSend("AT+CREG?");
  if(s.indexOf("OK",5)<0) return false;
  return true;
}

void gsmSleep(bool sleep) { 
  digitalWrite(DTR_PIN, HIGH);
  if(sleep){
    # if DEBUG > 0
    Serial<<"Putting SIM800L to sleep"<<endl;
    # endif
    while(!gsmVerify(gsmUartSend("AT+CSCLK = 1")));
    # if DEBUG > 0
    Serial<<"GSM is asleep."<<endl;
    # endif
  }
  else{
    # if DEBUG > 0
    Serial <<"Waking SIM800L from sleep"<<endl;
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

bool gsmSendSms(uint8_t num, String sms) {
  # if DEBUG > 0
  Serial<<"Sending Test SMS"<<endl;
  # endif
  gsmSleep(OFF);
  while(!gsmVerify(gsmUartSend("AT+CMGF=1"))); //Set the module in SMS mode
  GsmUart<<"AT+CMGS="<<"\""<<phoneNumbers[num]<<"\""<<endl;
  uint32_t t = millis();
  String result = "";
  while(!GsmUart.available()){
    delay(500);
    if(millis()-t>5000)return false;
  }
  if(GsmUart.available()){ //Wait for result
    result = GsmUart.readString();
    result.trim();
  }
  if(result == ">"){
    GsmUart << sms << endl<< (char)26 <<endl;
  }
  else return false;
  gsmSleep(ON);
  return true;
}

bool gsmVerify(String r) {
//  return (r == "OK");
  return (r.indexOf("ERROR")<0);  // If no error .indexOf returns -1
}

bool gsmCallIdSet() { 
  //if(!gsmVerify(gsmUartSend("AT+CLIP=1")))return(false);  // Caller ID
  if(!gsmVerify(gsmUartSend("AT+CMGF=1")))return(false);  // Set the SMS format to be readable
  return(true);
}

String gsmReadSms(){
  gsmSleep(OFF);
//  uint16_t index = 0;
//  index++;
  String  result = "", 
          number = "", 
//          send = "", 
          content = "";
//  send = "AT+CMGR=" + String(index);
//  result = gsmUartSend(send);
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
//    send = "AT+CMGD=" + String(index);
  }
//  gsmUartSend(send);
  gsmUartSend("AT+CMGD=1");
  ringPin = 0;
  gsmSleep(ON);
  return(number + " " + content)
}

void IRAM_ATTR interruptServiceRoutineRing() {
  if(!ringPin){
    Serial <<"\n---> RING PIN HIGH \n"; 
    ringPin = 1;
  }
  else Serial <<".";
}

void setup() {
  pinMode(DTR_PIN,        OUTPUT);
  pinMode(INTERRUPT_RING, INPUT );
  pinMode(LED_PIN,        OUTPUT);
  digitalWrite(DTR_PIN, LOW);
  Serial.begin(115200);
  GsmUart.begin(9600  , SERIAL_8N1, RX2, TX2);
  while(!gsmVerify(gsmUartSend("AT")));//delay(100);
  while(!gsmVerify(gsmUartSend("ATE0")));//delay(100);
  while(!gsmConnectionCheck());
  while(!gsmCallIdSet());
  //while(!GsmUart.available());
  //GsmUart.flush();
  Serial<<"GSM is Awake."<<endl;  
  attachInterrupt(digitalPinToInterrupt(INTERRUPT_RING), interruptServiceRoutineRing, FALLING );
}
 
void loop() {
  while(!Serial.available()){
    digitalWrite(LED_PIN, millis()%500>490);
    if(ringPin)break;
  }
  String r = Serial.readString();
  r.trim();
  digitalWrite(LED_PIN, HIGH);
  if(!ringPin){
    if     (r == "0") gsmSleep(OFF);
    else if(r == "1") gsmSleep(ON );
    else if(r == "2") gsmSendSms(0 ,"Test Message!");
    else if(r == "3") gsmReadSms();
    else              gsmUartSend(r);
  }
  else{
    gsmReadSms();
  }
}
