
// Which pins are connected to which LED
const byte oneLED = 5;
const byte twoLED = 4;

// setting PWM properties
const int freq = 5000;
const int oneledChannel = 0;
const int twoledChannel = 1;
const int resolution = 12; //Resolution 8, 10, 12, 15

int oneLEDdim ;
int twoLEDdim ;

const unsigned int oneLEDdelay = 10;
const unsigned int twoLEDdelay = 20;

unsigned long oneLEDprev;
unsigned long twoLEDprev;
unsigned long printPrev;

int oneLEDbrightness = 0;
int twoLEDbrightness = 0;

void setup () 
{
  Serial.begin(115200);
  pinMode (oneLED, OUTPUT);
  pinMode (twoLED, OUTPUT);
  
  ledcSetup(oneledChannel, freq, resolution);
  ledcSetup(twoledChannel, freq, resolution);
  
  ledcAttachPin(oneLED, oneledChannel);
  ledcAttachPin(twoLED, twoledChannel);
  
  oneLEDprev = 0;
  twoLEDprev = 0;
  printPrev = 0;
  oneLEDdim = 5;
  twoLEDdim = 5;
}

void fadeoneLED ()
{
  oneLEDbrightness += oneLEDdim;
  if(oneLEDbrightness > 4094 || oneLEDbrightness < 0)
    oneLEDdim = -oneLEDdim;
    oneLEDbrightness += oneLEDdim;
  ledcWrite(oneledChannel, oneLEDbrightness);   
  oneLEDprev = millis();   
}

void fadetwoLED ()
{
  twoLEDbrightness += twoLEDdim;
  if(twoLEDbrightness > 4094 || twoLEDbrightness < 0)
    twoLEDdim = -twoLEDdim;
    twoLEDbrightness += twoLEDdim;
  ledcWrite(twoledChannel, twoLEDbrightness);   
  twoLEDprev = millis();
}

void loop ()
{
  if(millis() - oneLEDprev >= oneLEDdelay)
    fadeoneLED ();
  if(millis() - twoLEDprev >= twoLEDdelay)
    fadetwoLED ();
  if(millis() - printPrev >= 5){
    Serial.println(String(oneLEDbrightness) + "\t" + String(twoLEDbrightness));
    printPrev = millis();
  }
/* Other code that needs to execute goes here.
 It will be called many thousand times per second because the above code
 does not wait for the LED fade interval to finish. */

}
