// Feather9x_RX
// -*- mode: C++ -*-
// Example sketch showing how to create a simple messaging client (receiver)
// with the RH_RF95 class. RH_RF95 class does not provide for addressing or
// reliability, so you should only use RH_RF95 if you do not need the higher
// level messaging abilities.
// It is designed to work with the other example Feather9x_TX

#include <SPI.h>
#include <RH_RF95.h>

/* for Feather32u4 RFM9x
#define RFM95_CS 8
#define RFM95_RST 4
#define RFM95_INT 7
*/

// for feather m0 RFM9x
#define RFM95_CS 8
#define RFM95_RST 4
#define RFM95_INT 3


/* for shield 
#define RFM95_CS 10
#define RFM95_RST 9
#define RFM95_INT 7
*/

/* Feather 32u4 w/wing
#define RFM95_RST     11   // "A"
#define RFM95_CS      10   // "B"
#define RFM95_INT     2    // "SDA" (only SDA/SCL/RX/TX have IRQ!)
*/

/* Feather m0 w/wing 
#define RFM95_RST     11   // "A"
#define RFM95_CS      10   // "B"
#define RFM95_INT     6    // "D"
*/

#if defined(ESP8266)
  /* for ESP w/featherwing */ 
  #define RFM95_CS  2    // "E"
  #define RFM95_RST 16   // "D"
  #define RFM95_INT 15   // "B"

#elif defined(ESP32)  
  /* ESP32 feather w/wing */
  #define RFM95_RST     27   // "A"
  #define RFM95_CS      33   // "B"
  #define RFM95_INT     12   //  next to A

#elif defined(NRF52)  
  /* nRF52832 feather w/wing */
  #define RFM95_RST     7   // "A"
  #define RFM95_CS      11   // "B"
  #define RFM95_INT     31   // "C"
  
#elif defined(TEENSYDUINO)
  /* Teensy 3.x w/wing */
  #define RFM95_RST     9   // "A"
  #define RFM95_CS      10   // "B"
  #define RFM95_INT     4    // "C"
#endif


// Change to 434.0 or other frequency, must match RX's freq!
#define RF95_FREQ 915.0

// Singleton instance of the radio driver
RH_RF95 rf95(RFM95_CS, RFM95_INT);

// Blinky on receipt
#define LED 13

#define TIMER_FREQ 96000000

//void TimerConfigure()
//
//void TimerCheck()

volatile float timestamp = 0;

void setup()
{
  pinMode(LED, OUTPUT);
//  pinMode(RFM95_RST, OUTPUT);
//  digitalWrite(RFM95_RST, HIGH);

  Serial.begin(115200);
  delay(100);
//
//  Serial.println("Feather LoRa RX Test!");
//
//  // manual reset
//  digitalWrite(RFM95_RST, LOW);
//  delay(10);
//  digitalWrite(RFM95_RST, HIGH);
//  delay(10);
//
//  while (!rf95.init()) {
//    Serial.println("LoRa radio init failed");
//    Serial.println("Uncomment '#define SERIAL_DEBUG' in RH_RF95.cpp for detailed debug info");
//    while (1);
//  }
//  Serial.println("LoRa radio init OK!");
//
//  // Defaults after init are 434.0MHz, modulation GFSK_Rb250Fd250, +13dbM
//  if (!rf95.setFrequency(RF95_FREQ)) {
//    Serial.println("setFrequency failed");
//    while (1);
//  }
//  Serial.print("Set Freq to: "); Serial.println(RF95_FREQ);
//
//  // Defaults after init are 434.0MHz, 13dBm, Bw = 125 kHz, Cr = 4/5, Sf = 128chips/symbol, CRC on
//
//  // The default transmitter power is 13dBm, using PA_BOOST.
//  // If you are using RFM95/96/97/98 modules which uses the PA_BOOST transmitter pin, then
//  // you can set transmitter powers from 5 to 23 dBm:
//  rf95.setTxPower(23, false);

  TimerConfigure();

}

void loop()
{
//  if (rf95.available())
//  {
//    // Should be a message for us now
//    uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
//    uint8_t len = sizeof(buf);
//
//    if (rf95.recv(buf, &len))
//    {
//      digitalWrite(LED, HIGH);
//      RH_RF95::printBuffer("Received: ", buf, len);
//      Serial.print("Got: ");
//      Serial.println((char*)buf);
//       Serial.print("RSSI: ");
//      Serial.println(rf95.lastRssi(), DEC);
//
//      // Send a reply
//      uint8_t data[] = "And hello back to you";
//      rf95.send(data, sizeof(data));
//      rf95.waitPacketSent();
//      Serial.println("Sent a reply");
//      digitalWrite(LED, LOW);
//    }
//    else
//    {
//      Serial.println("Receive failed");
//      digitalWrite(LED, HIGH);
//      delay(100);
//      digitalWrite(LED,LOW);
//    }

    
    delay(10);
    TimerCheck();
 // }
}


void TimerCheck(){
  timestamp = TIMER_FREQ / TC4->COUNT32.COUNT.reg;
  Serial.println(timestamp);
  TC4->COUNT32.COUNT.reg = 0;
}
void TimerConfigure(){
GCLK->CLKCTRL.reg = (uint16_t) (GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN_GCLK0 | GCLK_CLKCTRL_ID(GCM_TC4_TC5)) ;
 while (GCLK->STATUS.bit.SYNCBUSY);

GCLK->GENCTRL.reg = (uint16_t) (GCLK_GENCTRL_RUNSTDBY | GCLK_GENCTRL_GENEN | GCLK_GENCTRL_SRC_DFLL48M );
 while (GCLK->STATUS.bit.SYNCBUSY);
 TC4->COUNT32.CTRLA.reg |= TC_CTRLA_RUNSTDBY|
                            TC_CTRLA_PRESCSYNC_PRESC |
                           TC_CTRLA_MODE_COUNT32;  // Set the TC4 timer to 32-bit mode in conjuction with timer TC5
                   
  TC4->COUNT32.CTRLA.bit.ENABLE = 1;               // Enable TC4
  while (TC4->COUNT32.STATUS.bit.SYNCBUSY);        // Wait for synchronization
 
  TC4->COUNT32.READREQ.reg = TC_READREQ_RCONT |            // Enable a continuous read request
                             TC_READREQ_ADDR(0x10);        // Offset of the 32-bit COUNT register
  while (TC4->COUNT32.STATUS.bit.SYNCBUSY);                // Wait for (read) synchronization
  
}
