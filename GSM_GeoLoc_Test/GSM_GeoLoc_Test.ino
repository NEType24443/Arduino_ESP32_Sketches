#include <HardwareSerial.h>

#define RX2             GPIO_NUM_16   //  SIM800L TX 
#define TX2             GPIO_NUM_17   //  SIM800L RX

HardwareSerial SIM800(2); // RX, TX

String Link = "The current Location is https://www.google.com/maps/place/"; //we will append the Lattitude and longitude value later int the program
String Response = "";
String Longitude = "";
String Latitude = "";

String SIM800_send(String incoming) //Function to communicate with SIM800 module
{
  SIM800.println(incoming); delay(100); //Print what is being sent to GSM module
  String result = "";
  while (SIM800.available()) //Wait for result
  {
    char letter = SIM800.read();
    result = result + String(letter); //combine char to string to get result
  }
  return result; //return the result
}

void setup() {

  //PWRKY pin of GSM module has to be pulled low for 1sec to enable the module
  //pinMode(12, OUTPUT);
  //digitalWrite(12, LOW);   //Pull-down
  //delay(1000);
  //digitalWrite(12, HIGH); //Release

  Serial.begin(115200); //Serial COM for debugging
  SIM800.begin(9600  , SERIAL_8N1, RX2, TX2); //Software serial called SIM800 to speak with SIM800 Module

  delay(1000); //wait for serial COM to get ready

  Response = SIM800_send("ATE1"); //Enable Echo if not enabled by default
  Serial.print ("Response:"); Serial.println(Response);
  delay(1000);

  Response = SIM800_send("AT+CGATT=1"); //Set the SIM800 in GPRS mode
  Serial.print ("Response:"); Serial.println(Response);
  delay(1000);

  Response = SIM800_send("AT+SAPBR=3,1,\"CONTYPE\",\"GPRS\" "); //Activate Bearer profile
  Serial.print ("Response:"); Serial.println(Response);
  delay(1000);

  Response = SIM800_send("AT+SAPBR=3,1,\"APN\",\"airtelgprs.com\" "); //Set VPN options => 'RCMNET' 'www'
  Serial.print ("Response:"); Serial.println(Response);
  delay(2000);

  Response = SIM800_send("AT+SAPBR=1,1"); //Open bearer Profile
  Serial.print ("Response:"); Serial.println(Response); //Open bearer Profile
  delay(2000);

  Response = SIM800_send("AT+SAPBR=2,1"); //Get the IP address of the bearer profile
  Serial.print ("Response:"); Serial.println(Response);
  delay(1000);

}

void prepare_message()
{
  //Sample Output for AT+CIPGSMLOC=1,1   ==> +CIPGSMLOC: 0,75.802460,26.848892,2019/04/23,08:32:35 //where 26.8488832 is Lattitude and 75.802460 is longitute
  int first_comma = Response.indexOf(','); //Find the position of 1st comma
  int second_comma = Response.indexOf(',', first_comma + 1); //Find the position of 2nd comma
  int third_comma = Response.indexOf(',', second_comma + 1); //Find the position of 3rd comma

  for (int i = first_comma + 1; i < second_comma; i++) //Values form 1st comma to 2nd comma is Longitude
    Longitude = Longitude + Response.charAt(i);

  for (int i = second_comma + 1; i < third_comma; i++) //Values form 2nd comma to 3rd comma is Latitude
    Latitude = Latitude + Response.charAt(i);

  Serial.println(Latitude); Serial.println(Longitude);
  Link = Link + Latitude + "," + Longitude; //Update the Link with latitude and Logitude values
  Serial.println(Link);
}

String incoming = "";

void loop() {

  if (SIM800.available()) { //Check if the SIM800 Module is telling anything
    char a = SIM800.read();
    Serial.write(a); //print what the module tells on serial monitor
    incoming = incoming + String(a);
    if (a == 13) //check for new line
      incoming = ""; //clear the string if new line is detected
    incoming.trim(); //Remove /n or /r from the incomind data

    if (incoming == "RING") //If an incoming call is detected the SIM800 module will say "RING" check for it
    {
      Serial.println ("Sending sms"); delay(1000);
      Response = SIM800_send("ATH"); //Hand up the incoming call using ATH
      delay (1000);
      Response = SIM800_send("ATE0"); //Disable Echo
      delay (1000);

      Response = ""; Latitude = ""; Longitude = ""; //initialise all string to null
      SIM800.println("AT+CIPGSMLOC=1,1"); delay(5000); //Request for location data
      while (SIM800.available())
      {
        char letter = SIM800.read();
        Response = Response + String(letter); //Store the location information in string Response
      }
      Serial.print("Result Obtained as:");   Serial.print(Response); Serial.println("*******");

      prepare_message(); delay(1000); //use prepare_message funtion to prepare the link with the obtained LAT and LONG co-ordinates

      SIM800.println("AT+CMGF=1"); //Set the module in SMS mode
      delay(1000);

      SIM800.println("AT+CMGS=\"09481501967\""); //Send SMS to this number
      delay(1000);

      SIM800.println(Link); // we have send the string in variable Link
      delay(1000);

      SIM800.println((char)26);// ASCII code of CTRL+Z - used to terminate the text message
      delay(1000);
    }
  }

  if (Serial.available()) { //For debugging
    SIM800.write(Serial.read());
  }

}
