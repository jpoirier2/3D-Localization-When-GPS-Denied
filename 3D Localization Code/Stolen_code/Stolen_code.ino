#include <Wire.h>

#define START_COUNT_PIN 34
#define STOP_COUNT_PIN 39

volatile unsigned long startCountVar = 0;
volatile unsigned long stopCountVar = 0;
volatile boolean sendFlag = false;

void setup() {
  // put your setup code here, to run once:

  Wire.begin();

  attachInterrupt(START_COUNT_PIN,startCount,RISING);
  attachInterrupt(STOP_COUNT_PIN,stopCount,RISING);

}

void startCount(){
  startCountVar = ESP.getCycleCount();
}

void stopCount(){
  stopCountVar = ESP.getCycleCount();
  ESP.restart();
  sendFlag = true;
}

void loop() {
  // put your main code here, to run repeatedly:
   if(sendFlag){
    unsigned int count = stopCountVar - startCountVar - 53;
    Wire.beginTransmission(8);
    Wire.write(count);
    Wire.endTransmission();
    sendFlag = false;
   }

}
