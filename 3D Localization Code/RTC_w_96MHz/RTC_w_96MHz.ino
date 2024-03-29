
/*
 * 
 * Datasheet https://www.microchip.com/wwwproducts/en/ATSAMD21G18
 * Timer 5 and Interrupt Example  https://gist.github.com/nonsintetic/ad13e70f164801325f5f552f84306d6f
 * 
 * General SAMD timer https://www.avrfreaks.net/forum/using-internal-oscillator-not-asf
 * 
 * https://gist.github.com/jdneo/43be30d85080b175cb5aed3500d3f989
 * 
 * https://shawnhymel.com/1727/arduino-zero-samd21-fdpll-with-cmsis/
 * 
 * 
 */

 
#include <RTClib.h>
#include <Wire.h>

RTC_DS3231 rtc;

#define LED 13

#define TIMER_FREQ 96000000

// the pin that is connected to SQW
#define CLOCK_INTERRUPT_PIN 5


//void timerConfigure()
//
//void timerCheck()

volatile int timestamp = 0;

volatile float OV_count = 0;

volatile bool state = false;

volatile int count = 0;

volatile float Rcount = 0;

volatile float startCount = 0;

bool test = true;
uint16_t period = 48000;
TcCount16* TC = (TcCount16*) TC3;

void setup() {
  // put your setup code here, to run once:

Serial.begin(115200);
pinMode(LED,OUTPUT);
digitalWrite(LED,LOW);

PLLClockConfigure();

initializeFDPLL();

timerConfigure();

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

void timerCheck(){
// Step 1: Issue READYSYNC command
//TCC2->CTRLBSET.reg = TCC_CTRLBSET_CMD_READSYNC;
//
//// Step 2: Wait until the command is fully executed
//while (TCC2->SYNCBUSY.bit.CTRLB); // or while (TCC2->SYNCBUSY.reg);
//  uint16_t count = TCC2->COUNT.reg;
//  float Rcount = (float)count + (OV_count*48000);
//  Serial.print("Counter: ");
//  Serial.println(Rcount);
}
void PLLClockConfigure(){
// Set up the PLL to take the 48MHz clock input

  while (GCLK->STATUS.bit.SYNCBUSY);              // Wait for synchronization

  GCLK->CLKCTRL.reg = GCLK_CLKCTRL_CLKEN |         // Enable the Generic Clock
                     GCLK_CLKCTRL_GEN_GCLK3|      // Set the source of the changing clock to be GLCK3 which is OSC8M, 8MHz
                     GCLK_CLKCTRL_ID_FDPLL;   // Set FDPLL Clock source to be GLCK1, 1MHz
  while (GCLK->STATUS.bit.SYNCBUSY);              // Wait for synchronization                   

}

void initializeFDPLL() {

  SYSCTRL->DPLLRATIO.reg = SYSCTRL_DPLLRATIO_LDR(11)| // Set LDR to 11 for 12*8MHz
                          SYSCTRL_DPLLRATIO_LDRFRAC(0); // Set LDRFRAC to 0 just in case

  SYSCTRL->DPLLCTRLB.reg = SYSCTRL_DPLLCTRLB_DIV(0)|     // set the CLK_DPLL_REF1 clock division factor
                          (SYSCTRL_DPLLCTRLB_LBYPASS&0)| // select Lock Bypass mode 
                          SYSCTRL_DPLLCTRLB_LTIME(0)|   // select the FDPLL lock time-out
                          SYSCTRL_DPLLCTRLB_REFCLK(SYSCTRL_DPLLCTRLB_REFCLK_GCLK_Val)|  // select the FDPLL clock reference: 2 is GCLK_DPLL
                          SYSCTRL_DPLLCTRLB_WUF|     // select if output is gated during lock time
                          (SYSCTRL_DPLLCTRLB_LPEN&0)|    // Low-Power Enable
                          SYSCTRL_DPLLCTRLB_FILTER(0);  // select Proportional Integral Filter mode 

 SYSCTRL->DPLLCTRLA.reg = SYSCTRL_DPLLCTRLA_ONDEMAND| // FDPLL is always on once enabled.
                         (SYSCTRL_DPLLCTRLA_RUNSTDBY&1)| // FDPLL is NOT stopped in standby sleep mode
                         SYSCTRL_DPLLCTRLA_ENABLE; // FDPLL is Enabled
 while (SYSCTRL->DPLLSTATUS.bit.ENABLE);
 
}

void timerConfigure(){
  GCLK->GENCTRL.reg = GCLK_GENCTRL_RUNSTDBY |   // Have it run in standby
                     GCLK_GENCTRL_GENEN |        // Enable the Generic Clock
                     GCLK_GENCTRL_SRC_FDPLL |  // Set the clock source to be the 96MHz Output
                     GCLK_GENCTRL_ID(5);         // Select GLCK2 as output
 while (GCLK->STATUS.bit.SYNCBUSY);

   // Set clock divider of 1 to generic clock generator 5 (96 MHz)
  GCLK->GENDIV.reg = GCLK_GENDIV_DIV(1) |        // Divide 96 MHz by 1
                     GCLK_GENDIV_ID(5);           // Apply to GCLK5
  while (GCLK->STATUS.bit.SYNCBUSY);              // Wait for synchronization
 
  GCLK->CLKCTRL.reg = GCLK_CLKCTRL_CLKEN |       // Enable the Generic Clock
                     GCLK_CLKCTRL_GEN_GCLK5 |   // Select Source for Generic Clock to be GLCK2
                     GCLK_CLKCTRL_ID_TCC2_TC3;  // Output clock to Timer 2-3 clock
 while (GCLK->STATUS.bit.SYNCBUSY);     
 // Divide counter by 1 giving 96 MHz (10.42 ns) on each TC3 tick
  TC->CTRLA.reg |= TC_CTRLA_PRESCALER_DIV1|
                    TC_CTRLA_MODE_COUNT16|
                    TC_CTRLA_WAVEGEN_NFRQ; // Match Frequency Mode, makes CC0 the top value
        
       
  while (TC->STATUS.bit.SYNCBUSY);                // Wait for synchronization


//  // Set the period (the number to count to (TOP) before resetting timer)
//  TC->CC[0].reg = period;
//  while (TC->STATUS.bit.SYNCBUSY);

  TC->EVCTRL.reg &= ~TC_EVCTRL_OVFEO; // Overflow/underflow counter event is enabled

  TC->INTENSET.reg = TC_INTENSET_OVF; // Enable the Overflow interrupt

  NVIC_EnableIRQ(TC3_IRQn);


  TC->CTRLA.reg |= TC_CTRLA_ENABLE;
  while (TC->STATUS.bit.SYNCBUSY);              // Wait for synchronization
  
  
  TC->READREQ.reg = TC_READREQ_RCONT |            // Enable a continuous read request
                    TC_READREQ_ADDR(0x10);        // Offset of the 16 bit COUNT register
  while (TC->STATUS.bit.SYNCBUSY);       // Wait for (read) synchronization

}

void TC3_Handler(){

  if (TC->INTFLAG.bit.OVF == 1) {  // A overflow caused the interrupt
     //TC3->INTFLAG.bit.OVF = 1;    // writing a one clears the flag ovf flag
     OV_count++;
     TC->INTFLAG.bit.OVF = 1; 
  }
}

void onAlarm() {

    count = TC->COUNT.reg;
  float Rcount = (float)count + (OV_count*65535);
//  Serial.print("Counter: ");
//  Serial.println(Rcount);
  OV_count = 0;
//      char date[10] = "hh:mm:ss";
//    rtc.now().toString(date);
//    Serial.print(date);
//    Serial.print("  ");
//    Serial.print("Counter: ");
  Serial.println(Rcount);

    if(rtc.alarmFired(1)) {
        rtc.clearAlarm(1);
     //   Serial.println("Alarm cleared");
    }
    
}
