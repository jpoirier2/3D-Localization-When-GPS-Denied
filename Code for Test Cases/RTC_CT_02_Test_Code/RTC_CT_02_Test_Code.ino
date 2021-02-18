
/*

 * Datasheet https://www.microchip.com/wwwproducts/en/ATSAMD21G18
 */

 
#include <RTClib.h>
#include <Wire.h>

RTC_DS3231 rtc;

// the pin that is connected to SQW
#define CLOCK_INTERRUPT_PIN 5


volatile int timestamp = 0;

void setup() {
  // put your setup code here, to run once:

Serial.begin(115200);

initializeRTC();

initializeAlarm();

}

void loop() {
  // put your main code here, to run repeatedly:

}


void initializeRTC(){
  // Initialize RTC and disable the 32k pin as we do not use it.
      if(!rtc.begin()) {
        Serial.println("Couldn't find RTC!");
        Serial.flush();
        abort();
    }
    
    if(rtc.lostPower()) {
        // this will adjust to the date and time at compilation
        rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
    }
    
    //we don't need the 32K Pin, so disable it
    rtc.disable32K();
  
}

void initializeAlarm(){
  //This function initializes the RTC's PerSecond Alarm function and attachs the interrupt to the pin we have choosen.

   // Making it so, that the alarm will trigger an interrupt
    pinMode(CLOCK_INTERRUPT_PIN, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(CLOCK_INTERRUPT_PIN), onAlarm, FALLING);
    
    // set alarm 1, 2 flag to false (so alarm 1, 2 didn't happen so far)
    // if not done, this easily leads to problems, as both register aren't reset on reboot/recompile
    rtc.clearAlarm(1);
    rtc.clearAlarm(2);
    
    // stop oscillating signals at SQW Pin
    // otherwise setAlarm1 will fail
    rtc.writeSqwPinMode(DS3231_OFF);
    
    // turn off alarm 2 (in case it isn't off already)
    // again, this isn't done at reboot, so a previously set alarm could easily go overlooked
    rtc.disableAlarm(2);
    
    // schedule an alarm 10 seconds in the future
    if(!rtc.setAlarm1(
            rtc.now(),
            DS3231_A1_PerSecond // this mode triggers the alarm every second. See Doxygen for other options
    )) {
        Serial.println("Error, alarm wasn't set!");
    }else {
        Serial.println("Alarm will happen every second!");  
    }
}


void onAlarm() {

  
      char date[10] = "hh:mm:ss";
    rtc.now().toString(date);
    Serial.print(date);

    if(rtc.alarmFired(1)) {
        rtc.clearAlarm(1);
     //   Serial.println("Alarm cleared");
    }
    
}
