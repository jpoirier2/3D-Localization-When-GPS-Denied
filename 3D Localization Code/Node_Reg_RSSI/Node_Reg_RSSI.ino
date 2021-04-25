#include <SPI.h>
#include <RH_RF95.h>
#include <RTClib.h>
#include <Wire.h>
#include <math.h>

// Feather M0 pinout
#define RFM95_CS 8
#define RFM95_RST 4
#define RFM95_INT 3

#define LED 13

// Setting frequency (MHz)
#define RF95_FREQ 915.0

// the pin that is connected to SQW
#define CLOCK_INTERRUPT_PIN 5

// Singleton instance of the radio driver
RH_RF95 rf95(RFM95_CS, RFM95_INT);

RTC_DS3231 rtc;

#define TIMER_FREQ 96000000
#define NODE_DELAY 100// Change this from node to node

// Device ID character
char ID[] = "A"; // This is a char array instead of a char because I spent an hour and a half trying to make it work as a char and it was a massive pain in my ass

char TT[] = "1"; //Tracking target ID

void RFM95_SETUP(bool cereal = false);
// Performs the standard setup process for the RF95 as used by the project
// Setting "cereal" to True pauses the program until the serial monitor is opened. It does NOT prevent the serial console from being started
// Yes I am aware that it's spelled "serial"
// I am very funny

void receiveRaw(char **msg);
// recieves a raw LoRa transmission (Includes ID)
// msg is a pointer to an array with the max message length

bool isIdentified(char *msg, int len); 
// Determines if the message contained in msg is identified or not
// msg is a pointer to the message, len is the message's length 
// NOTE: Function will return true for an unidentified message if msg[0] is a letter and msg[1] = "|"

char getID(char *msg);
// Returns the ID of an identified message
// msg is the identified message

void broadcast(char *msg, int len, bool includeID = false);
// Broadcasts a signal via RF95
// msg is a pointer to a message
// len contains the length of msg (can be obtained using sizeof(msg)/sizeof(msg[0]))
// includeID determines whether the device ID is included at the beginning of the message

void blinkyBlinky(); // Literally just blinks the light to save space in main

float distanceRSSI(float R, float RAVG); // Computes distance and meters using the RSSI and the RSSI (average is ideal) at 1 meter 

void getTimestamp();

volatile int timestamp = 0;

volatile float OV_count = 0;

volatile bool state = false;

volatile int count = 0;

volatile int Rcount = 0;


volatile float unLogRSSIavg = pow(10, -22.84/20); // RSSI average converted out of dB

TcCount16* TC = (TcCount16*) TC3;
void setup() {
  // put your setup code here, to run once:

  Serial.begin(115200);

// while (!Serial) {
//    delay(1); // wait
// }

  
  PLLClockConfigure();

  initializeFDPLL();
  
  timerConfigure();
  
  initializeRTC();
  
  initializeAlarm();

  RFM95_SETUP();
}

void loop() {
  // put your main code here, to run repeatedly:
  char *message;
  char date[20] = "hh:mm:ss"; // Must be done for formatting fo rtc.now()
  char sendDistance[20];
  int len;
  char i[10] = ""; // 5 Might be enough
  digitalWrite(LED, HIGH);
  receiveRaw(&message);
 // getTimestamp();
  //rtc.now().toString(date);
  digitalWrite(LED, LOW);
  len = strlen(message);
  if (isIdentified(message, len) && (getID(message) == TT[0])) { // Checks if the message is from the target
    //Serial.print("Raw message received: "); Serial.println(message); 
   // Serial.println("IN IF STATEMENT");
//    strcat(date, ";");
//    itoa(Rcount, i, 10);
//    strcat(date, i);
//    Serial.print("After Count Cat: ");Serial.println(date);

   float R = (float)rf95.lastRssi();
  //  Serial.print("R variable is: ");Serial.println(R);Serial.println();
    float unLogR = pow(10.0,R/20);
    Serial.print("unLogR variable is: ");Serial.println(unLogR,10);Serial.println();
    float distance = distanceRSSI(R,-22.84);
    distance = distance ;
    Serial.print("Distance is: "); Serial.println(distance);
    int whole = floor(distance);
    int decimal = floor(distance * 100) - whole * 100;
    char mbuff[10] = "";
    itoa(whole, mbuff, 10);
    strcat(sendDistance, mbuff);
    strcat(sendDistance, ".");
    itoa(decimal, mbuff, 10);
    strcat(sendDistance, mbuff);
    delay(NODE_DELAY);
    broadcast(sendDistance, sizeof(sendDistance)/sizeof(sendDistance[0]), true);
    strcpy(sendDistance,"");
  }
  digitalWrite(LED,HIGH);
}

void receiveRaw(char **msg) {
    if (rf95.waitAvailableTimeout(1000)) {
      char *rec = (char*)malloc(RH_RF95_MAX_MESSAGE_LEN);
      uint8_t buf[RH_RF95_MAX_MESSAGE_LEN]; // Creating a buffer and defining its length
      uint8_t len = sizeof(buf);
  
      if (rf95.recv(buf, &len)) {
        //digitalWrite(LED, HIGH); 
        //RH_RF95::printBuffer("Received: ", buf, len);
        Serial.print("Got: "); Serial.println((char*)buf);
        strcpy(rec, (char*)buf);
        *msg = rec;
      }
      else {
        Serial.println("Receive failed");  
      }
    }
    else {
      *msg = "No message";  
    }
}

bool isIdentified(char *msg, int len) {
  return (isAlphaNumeric(msg[0])&&(msg[1]=='|'));  
}

char getID(char *msg) {
  return msg[0];  
}

void broadcast(char *msg, int len, bool includeID) {
  //delay(10); // Sample code does this. Probably wouldn't hurt to wait a bit before transmission. We can remove this later if needed
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


float distanceRSSI(float Rin, float RAVG) {
    float N = 4; // "Low strength" (Default of 2)
    Serial.print("Variables are : ");Serial.print(Rin,10);Serial.print(" ");Serial.println(RAVG,10);
    float dist = pow(10.0, (RAVG - Rin )/(10*N));
    Serial.println(dist);
    return dist;
}

void getTimestamp(){
  count = TC->COUNT.reg;
  Rcount = count + (OV_count*65535);
 // Serial.println(Rcount);
  OV_count = 0;
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
    Rcount = count + (OV_count*65535);
//  Serial.print("Counter: ");
//  Serial.println(Rcount);
  OV_count = 0;
//      char date[10] = "hh:mm:ss";
//    rtc.now().toString(date);
//    Serial.print(date);
//    Serial.print("  ");
//    Serial.print("Counter: ");
 // Serial.println(Rcount);

    if(rtc.alarmFired(1)) {
        rtc.clearAlarm(1);
     //   Serial.println("Alarm cleared");
    }
    
}



void RFM95_SETUP(bool cereal) {
  pinMode(LED, OUTPUT); 
  pinMode(RFM95_RST, OUTPUT); // Sets the reset pin to output
  digitalWrite(RFM95_RST, HIGH); // Writes high to the reset pin
  Serial.begin(115200); // Starts serial transmission at a baud rate of 115200
  // Waits for the serial console to be opened
  // REMOVE THIS IF THE BOARD ISN'T CONNECTED TO A COMPUTER
  while (cereal && !Serial) {
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
}
