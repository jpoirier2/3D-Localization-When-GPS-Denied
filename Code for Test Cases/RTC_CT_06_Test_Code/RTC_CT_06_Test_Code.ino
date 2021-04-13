#include <SPI.h>
#include <RH_RF95.h>

#include <RTClib.h>
#include <Wire.h>

RTC_DS3231 rtc;


// Feather M0 pinout
#define RFM95_CS 8
#define RFM95_RST 4
#define RFM95_INT 3

#define LED 13

// the pin that is connected to SQW
#define CLOCK_INTERRUPT_PIN 5

// Setting frequency (MHz)
#define RF95_FREQ 915.0

// Singleton instance of the radio driver
RH_RF95 rf95(RFM95_CS, RFM95_INT);

// Device ID character
char ID[] = "A"; // This is a char array instead of a char because I spent an hour and a half trying to make it work as a char and it was a massive pain in my ass

void broadcast(char *msg, int len, bool includeID = false);
// Broadcasts a signal via RF95
// msg is a pointer to a message
// len contains the length of msg (can be obtained using sizeof(msg)/sizeof(msg[0]))
// includeID determines whether the device ID is included at the beginning of the message

void blinkyBlinky(); // Literally just blinks the light to save space in main

volatile bool sendMessage =false;

volatile bool state = true;

volatile int OV_count = 0;

volatile int Rcount = 0;

TcCount16* TC = (TcCount16*) TC3;

void setup() {
  // serial setup and radio hard-reset
  pinMode(LED, OUTPUT); 
  pinMode(RFM95_RST, OUTPUT); // Sets the reset pin to output
  digitalWrite(RFM95_RST, HIGH); // Writes high to the reset pin

  Serial.begin(115200); // Starts serial transmission at a baud rate of 115200
  // Waits for the serial console to be opened
  // REMOVE THIS IF THE BOARD ISN'T CONNECTED TO A COMPUTER
  while (!Serial) {
    delay(1); // wait
 }
  delay(100);

  // RFM95 reset
  digitalWrite(RFM95_RST, LOW); // Resets RFM95
  delay(10);
  digitalWrite(RFM95_RST, HIGH);
  delay(10);

  while (!rf95.init()) {
    Serial.println("Something's wrong. I can feel it.\nLoRa radio init failed"); // Error detection that I decided to leave in
    while (1); // Stops the program in its tracks
  }
  Serial.println("LoRa radio successfully initialized");

  if (!rf95.setFrequency(RF95_FREQ)) {
    Serial.println("setFrequency failed"); // Checks if a frequency is not sent
    while(1);
  }
  Serial.print("Broadcast frequency: "); Serial.println(RF95_FREQ);
  rf95.setTxPower(23, false); // Sets power level to 23 dBm with RFO disabled
  digitalWrite(LED, LOW);

 initRTC();

 PLLClockConfigure();

initializeFDPLL();

timerConfigure();

}

uint8_t count = 0; // Will count the number of messages sent

void loop() {
  // put your main code here, to run repeatedly:
   if(sendMessage == true){
  char date[10] = "hh:mm:ss";
  char message[50] = "";
  char mbuff[10] = "";
  rtc.now().toString(date);
  strcat(message,date);
  strcat(message," cycles:");
  itoa(Rcount,mbuff,10);
  strcat(message,mbuff);
  int l = sizeof(message)/sizeof(message[0]);

 
  broadcast(message, l, true);
  digitalWrite(LED,state);
  state = ~state;
  sendMessage = false;
  }


  
}

void onAlarm() {
    sendMessage = true;

  count = TC->COUNT.reg;
  Rcount = count + (OV_count*65535);
   OV_count = 0;

       if(rtc.alarmFired(1)) {
        rtc.clearAlarm(1);
    }
}

void broadcast(char *msg, int len, bool includeID) {
  delay(10); // Sample code does this. Probably wouldn't hurt to wait a bit before transmission. We can remove this later if needed
  char msgB[len+2]; // Char that includes the ID designation and the broadcasted message
  if (includeID) {
    //msgB[0] = ID[0];
    //msgB[1] = '|';
    strcpy(msgB, ID);
    strcat(msgB, "|");
    strcat(msgB, msg);
    /*for (int i = 2; i<len+2; i++) {
      msgB[i] = msg[i-2];
    }*/
    
    Serial.print("Sending "); Serial.println(msgB);
    rf95.send((uint8_t *)msgB, sizeof(msgB));  
  }
  else {
    //Serial.print("msg size: "); Serial.println(sizeof(msg)/sizeof(char));
    //Serial.print("msgB size: "); Serial.println(sizeof(msgB)/sizeof(char));
    strcpy(msgB, msg);
    Serial.print("Sending "); Serial.println(msg);
    rf95.send((uint8_t *)msgB, sizeof(msgB));
    //char realmsg[] = "Hello there";
    //rf95.send((uint8_t *)realmsg, sizeof(realmsg));
  }
  Serial.println("Waiting for packet to complete...");
  delay(10);
  rf95.waitPacketSent(); // Waits for the package to send
  Serial.println("Packet completed");
}

void blinkyBlinky() {
  digitalWrite(LED, HIGH);
  delay(100);
  digitalWrite(LED, LOW);
  delay(100);
  digitalWrite(LED, HIGH);
  delay(100);
  digitalWrite(LED, LOW);  
}

void initRTC(){
      // initializing the rtc
    if(!rtc.begin()) {
        Serial.println("Couldn't find RTC!");
        Serial.flush();
        abort();
    }

    
        // this will adjust to the date and time at compilation
        rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
    
    
    //we don't need the 32K Pin, so disable it
    rtc.disable32K();
    
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
        Serial.println("Alarm will happen in 10 seconds!");  
    }
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
