#include <Wire.h>
#include <MPU6050.h>
#include <Streaming.h>
#include <EEPROM.h>
#include <driver/rtc_io.h>

#define LED_PIN GPIO_NUM_2
#define INTERRUPT_PIN GPIO_NUM_33

#define SDA_PIN GPIO_NUM_22
#define SCL_PIN GPIO_NUM_21

#define MOTION_THRESHOLD 5   // 12
#define MOTION_EVENT_DURATION 50    //50

MPU6050 accelgyro;

RTC_DATA_ATTR String lastString = "";
RTC_DATA_ATTR int bootCount = 0;
RTC_DATA_ATTR bool ledState = false;

void setup() {
  Wire.begin(SDA_PIN, SCL_PIN);
  Serial.begin(115200);
  pinMode(INTERRUPT_PIN, INPUT_PULLDOWN);
  //pinMode(LED_PIN, OUTPUT);
  //gpio_hold_en(LED_PIN);
  ledStateChange();
  Serial<<"Boot Count: "<<bootCount<<endl;
  if(!bootCount){
    rtc_gpio_hold_dis(LED_PIN);
    rtc_gpio_set_level(LED_PIN, HIGH);
    accelgyro.initialize();
    // verify connection to MPU6050
    Serial<<"Testing device connections..."<<endl;
    Serial<<(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed")<<endl;
    delay(1);
    accelgyro.setAccelerometerPowerOnDelay(3);
    
    accelgyro.setInterruptMode(0); // Interrupts enabled
    
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
  
    // Motion detection acceleration threshold. 1LSB = 2mg.
    accelgyro.setMotionDetectionThreshold(MOTION_THRESHOLD);
    // Motion detection event duration in ms
    accelgyro.setMotionDetectionDuration(MOTION_EVENT_DURATION);
    delay(1000);
    rtc_gpio_set_level(LED_PIN,LOW);
    rtc_gpio_hold_en(LED_PIN);
}
  bootCount++;
  //delay(1);
  //Configure GPIO33 as ext0 wake up source for HIGH logic level
  esp_sleep_enable_ext0_wakeup(INTERRUPT_PIN,1);

}

void ledStateChange() {
  rtc_gpio_init(LED_PIN);
  rtc_gpio_set_direction(LED_PIN, RTC_GPIO_MODE_OUTPUT_ONLY);
  rtc_gpio_hold_dis(LED_PIN);
  rtc_gpio_set_level(LED_PIN, ledState);
  rtc_gpio_hold_en(LED_PIN);
  //digitalWrite(LED_PIN, ledState);
  Serial<<"LED State: "<<ledState<<endl;
  ledState ^= 1;
}

void loop() {
  Serial<<"Waiting for command"<<endl;
  while(!Serial.available());
  if(Serial.available()){
    //Go to sleep now
    Serial<<"Last String was: "<<lastString<<" and " ;
    lastString = Serial.readString();
    Serial<<"New String is: "<<lastString<<endl
          <<"Going to Sleep"<<endl;
    esp_deep_sleep_start();
  }
}
