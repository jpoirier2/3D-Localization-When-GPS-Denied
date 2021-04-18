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
 */
#define LED 13

#define INTERRUPT_PIN 12

#define TIMER_FREQ 96000000

#define GCLK_GENCTRL_SRC_DPLL96M_Val 0x8ul  // (GCLK_GENCTRL) DPLL96M output
#define GCLK_GENCTRL_SRC_DPLL96M (GCLK_GENCTRL_SRC_DPLL96M_Val << \
                                  GCLK_GENCTRL_SRC_Pos)

//void TimerConfigure()
//
//void TimerCheck()

volatile int timestamp = 0;

volatile float OV_count = 0;

volatile bool state = false;

volatile float count = 0;

volatile float Rcount = 0;

bool test = true;
uint16_t period = 19199;
TcCount16* TC = (TcCount16*) TC3;

void setup() {
  // put your setup code here, to run once:

Serial.begin(115200);
pinMode(LED,OUTPUT);
digitalWrite(LED,LOW);

attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), TimerCheck, RISING);

PLLClockConfigure();

FDPLL_Init();

TimerConfigure();


}

void loop() {
  // put your main code here, to run repeatedly:
    

}

void TimerCheck(){
        count = TC->COUNT.reg;
        Rcount = (float)count + (OV_count*65535);
        //Serial.print("Counter: ");
        Serial.println(Rcount);
       digitalWrite(LED,state);
          OV_count = 0;
         state = !state;  

}
void PLLClockConfigure(){
// Set up the generic clock (GCLK 4) used to clock timers

//  GCLK->GENCTRL.reg = (uint16_t) (GCLK_GENCTRL_RUNSTDBY|   // Have it run in standby
//                     GCLK_GENCTRL_GENEN|         // Enable GCLK 4
//                     GCLK_GENCTRL_SRC_OSC8M |   // Set the 48MHz clock source
//                     (GCLK_GENCTRL_DIVSEL&0) |    // Setting it so that it divides by GENDIV
//                     GCLK_GENCTRL_ID(4));          // Select GCLK 4
//  while (GCLK->STATUS.bit.SYNCBUSY);              // Wait for synchronization
//  
//  GCLK->GENDIV.reg = GCLK_GENDIV_DIV(8) |          // Divide the 8MHz clock source by divisor 8: 8MHz/8= 1MHz
//                    GCLK_GENDIV_ID(4);            // Select Generic Clock (GCLK) 1
//  while (GCLK->STATUS.bit.SYNCBUSY);              // Wait for synchronization

  GCLK->CLKCTRL.reg = GCLK_CLKCTRL_CLKEN |         // Enable the Generic Clock
                     GCLK_CLKCTRL_GEN_GCLK3|      // Set the source of the changing clock
                     GCLK_CLKCTRL_ID_FDPLL;   // Set FDPLL Clock source to be GLCK4, 1MHz
  while (GCLK->STATUS.bit.SYNCBUSY);              // Wait for synchronization                   

}

void FDPLL_Init() {

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

void TimerConfigure(){
  GCLK->GENCTRL.reg = GCLK_GENCTRL_RUNSTDBY |   // Have it run in standby
                     GCLK_GENCTRL_GENEN |        // Enable the Generic Clock
                     GCLK_GENCTRL_SRC_FDPLL |  // Set the clock source to be the 96MHz Output
                     GCLK_GENCTRL_ID(5);         // Select GLCK5 as output
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
