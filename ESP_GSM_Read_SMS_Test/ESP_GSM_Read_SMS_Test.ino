//#include <Arduino.h>
#include <Streaming.h>
#include <EEPROM.h>
#include <HardwareSerial.h>

#define ON    1
#define OFF   0

#define DEBUG ON

#define PSWD "password"

#define LED_PIN         GPIO_NUM_2    //  ONBOARD LED

#define INTERRUPT_RING  GPIO_NUM_18   //  SIM800L RING PIN
#define DTR_PIN         GPIO_NUM_5    //  SIM800L DTR  PIN

#define RX2             GPIO_NUM_16   //  SIM800L TX 
#define TX2             GPIO_NUM_17   //  SIM800L RX

//#define GSM_CONFIRMED   "OK"

HardwareSerial GsmUart(2);

volatile bool ringPin = 0;
bool sentSMS = 0,
     location_cmd = 0,
     register_cmd = 0,
     deregister_cmd = 0;


String phoneNumbers[5] =  { "0000000000000", 
                            "0000000000000", 
                            "0000000000000", 
                            "0000000000000", 
                            "0000000000000" };
                        // { "+919448517225", "+919481501967"};

uint16_t        address = 0;

RTC_DATA_ATTR uint16_t boot_cnt = 0;

const uint16_t  boot_cnt_addr = 0,
                boot_cnt_size = sizeof(uint16_t),     // unsigned short
                base_num_addr = boot_cnt_size + 1,    // Starts at EEPROM base OR the first char can immediately be read
                num_spacing   = 0x0D, // 13 Chars
                end_num_addr  = base_num_addr + 0x41; // beyond is no longer phone number
             
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
  for(i=0; i<5; i++)
    phoneNumbers[i] = EEPROM.readString((i*num_spacing)+base_num_addr+(i*2));
}

bool gsmParseCommand(String cmd){
  String source = cmd.substring(0,13), command = cmd.substring(14);
  uint8_t i = getIndexOfNumber(source);
  command.trim();
  #if DEBUG>0
  Serial<<endl<<"Parsing Command" << endl
              <<"source:  " << source  << endl
              <<"command: " << command << endl
              <<"index:   " << i       << endl;
  #endif
  // while (source != phoneNumbers[i])i++;
  if(i<5){  // If number is registered.
    #if DEBUG>0
    Serial<< "Registered number.\n";
    #endif
    if(command == "?location"){
      #if DEBUG>0
      Serial<< "Location..." <<endl;
      #endif
      location_cmd = 1;
      gsmSendSmsToNumber(source, "PLACEHOLDER", 0, 1);
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
          gsmSendSmsToNumber(source, "Deregistered successfully", 0, 1);
        }
      }
    }
    else if(command == "?help"){
      #if DEBUG>0
      Serial<< "Help..." <<endl;
      #endif
      String sms = "--COMMANDS--\n";
      gsmSendSmsToNumber(source, sms, 0, 0);
      sms = "?location\n";
      gsmSendSmsToNumber(source, sms, 0, 0);
      sms = "?register <password>\n";
      gsmSendSmsToNumber(source, sms, 0, 0);
      sms = "?deregister <password>\n";
      gsmSendSmsToNumber(source, sms, 0, 0);
      sms = "?help";
      gsmSendSmsToNumber(source, sms, 0, 1);
    }
    else  return(0);
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
      if(registerNewNumber(source)){ // Registered successfully or not.
        gsmSendSmsToNumber(source,"Registered successfully.", 0, 1);
        return(1);
      }
      else{
        gsmSendSmsToNumber(source,"Failed to register.", 0, 1);
        return(0);
      }
    }
  }
  else {
    #if DEBUG>0
    Serial<<"Unregistered number OR Wrong password entered."<<endl;
    #endif
    gsmSendSmsToNumber(source, "Unregistered number OR Wrong password entered.", 0, 1);
    return(0);
  }
}

void eepromDump(){
  Serial<< "EEPROM DATA" << endl
        << "Boot Count: "<< EEPROM.readUShort(0)                                << endl
        << "Num 1 - "    << EEPROM.readString((0*num_spacing)+base_num_addr+0)  << endl
        << "Num 2 - "    << EEPROM.readString((1*num_spacing)+base_num_addr+2)  << endl
        << "Num 3 - "    << EEPROM.readString((2*num_spacing)+base_num_addr+4)  << endl
        << "Num 4 - "    << EEPROM.readString((3*num_spacing)+base_num_addr+6)  << endl
        << "Num 5 - "    << EEPROM.readString((4*num_spacing)+base_num_addr+8)  << endl;
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
  pinMode(INTERRUPT_RING, INPUT_PULLUP);
  pinMode(LED_PIN,        OUTPUT);
  digitalWrite(DTR_PIN, LOW);
  Serial.begin(115200);
  GsmUart.begin(115200  , SERIAL_8N1, RX2, TX2);
  if (!EEPROM.begin(1000)) {
    Serial.println("Failed to initialise EEPROM");
    Serial.println("Restarting...");
    delay(1000);
    ESP.restart();
  }
  boot_cnt = EEPROM.readUShort(0); 
  if(boot_cnt == 0){ // Implies the system restarted
    //mode = INITIAL_SETUP;
    Serial<<"First Start, waiting for 5 seconds"<<endl;
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
  }
  delay(10);
  while(!gsmVerify(gsmUartSend("AT")));
  while(!gsmVerify(gsmUartSend("ATE0")));
  while(!gsmVerify(gsmUartSend("AT+IPR=115200")));
  while(!gsmConnectionCheck());
  while(!gsmCallIdSet());
//  while(!GsmUart.available());
//  GsmUart.flush();
//  Serial<<"Putting GSM to sleep"<<endl;
  gsmSleep(ON);
  boot_cnt++;
  EEPROM.writeUShort(0,boot_cnt);
  EEPROM.commit();
  attachInterrupt(digitalPinToInterrupt(INTERRUPT_RING), interruptServiceRoutineRing, FALLING );
}

void loop() {
  while(!Serial.available()) {
    digitalWrite(LED_PIN, millis()%500>490);
    if(ringPin)break;
  }
  String r = Serial.readString();
  r.trim();
  digitalWrite(LED_PIN, HIGH);
  if(!ringPin){
    if     (r == "0")           gsmSleep(OFF);
    else if(r == "1")           gsmSleep(ON );
    else if(r == "2")           gsmSendSms(0 ,"Test Message!");
    else if(r == "3")           gsmParseCommand(gsmReadSms(1,0));
    else if(r == "4")           eepromDump();
    else if(r.startsWith("+"))  {
      if(getIndexOfNumber(r)<5){
        Serial<< "Deregistering..." <<endl;
        registerNewNumber("0000000000000", r);
      }
      else{  
        Serial<< "Registering..."   <<endl;
        registerNewNumber(r);
      }
    }
    else                        gsmUartSend(r);
  }
  else{
    gsmParseCommand(gsmReadSms(1,0));
    Serial<<"Delt with sms"<<endl;
  }
}
