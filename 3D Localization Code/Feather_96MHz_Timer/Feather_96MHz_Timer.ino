
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

#define TIMER_FREQ 96000000

//void TimerConfigure()
//
//void TimerCheck()

volatile int timestamp = 0;

volatile float OV_count = 0;

volatile bool state = false;

volatile float count = 0;

volatile float Rcount = 0;

bool test = true;
uint16_t period = 48000;
void setup() {
  // put your setup code here, to run once:

Serial.begin(115200);
pinMode(LED,OUTPUT);
digitalWrite(LED,LOW);

PLLClockConfigure();

FDPLL_Init();

TimerConfigure();


}

void loop() {
  // put your main code here, to run repeatedly:

}

void TimerCheck(){
// Step 1: Issue READYSYNC command
TCC2->CTRLBSET.reg = TCC_CTRLBSET_CMD_READSYNC;

// Step 2: Wait until the command is fully executed
while (TCC2->SYNCBUSY.bit.CTRLB); // or while (TCC2->SYNCBUSY.reg);
  uint16_t count = TCC2->COUNT.reg;
  float Rcount = (float)count + (OV_count*48000);
  Serial.print("Counter: ");
  Serial.println(Rcount);
}
void PLLClockConfigure(){
// Set up the generic clock (GCLK 4) used to clock timers

  GCLK->GENCTRL.reg = (uint16_t) (GCLK_GENCTRL_RUNSTDBY|   // Have it run in standby
                     GCLK_GENCTRL_GENEN|         // Enable GCLK 4
                     GCLK_GENCTRL_SRC_DFLL48M |   // Set the 48MHz clock source
                     GCLK_GENCTRL_ID(4));          // Select GCLK 4
  while (GCLK->STATUS.bit.SYNCBUSY);              // Wait for synchronization
  
  GCLK->GENDIV.reg = GCLK_GENDIV_DIV(48) |          // Divide the 48MHz clock source by divisor 48: 48MHz/48= 1MHz
                    GCLK_GENDIV_ID(4);            // Select Generic Clock (GCLK) 1
  while (GCLK->STATUS.bit.SYNCBUSY);              // Wait for synchronization

  GCLK->CLKCTRL.reg = GCLK_CLKCTRL_CLKEN |         // Enable the Generic Clock
                     GCLK_CLKCTRL_GEN_GCLK4|      // Set the source of the changing clock
                     GCLK_CLKCTRL_ID_FDPLL;   // Set FDPLL Clock source to be GLCK1, 1MHz
  while (GCLK->STATUS.bit.SYNCBUSY);              // Wait for synchronization                   

}

void FDPLL_Init() {

  SYSCTRL->DPLLRATIO.reg = SYSCTRL_DPLLRATIO_LDR(95)| // Set LDR to 47 for 48*MHz
                          SYSCTRL_DPLLRATIO_LDRFRAC(0); // Set LDRFRAC to 0 just in case

  SYSCTRL->DPLLCTRLB.reg = SYSCTRL_DPLLCTRLB_DIV(0)|     // set the CLK_DPLL_REF1 clock division factor
                          SYSCTRL_DPLLCTRLB_LBYPASS| // select Lock Bypass mode 
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

  // Divide counter by 1 giving 96 MHz (10.42 ns) on each TCC2 tick
  TCC2->CTRLA.reg |= TCC_CTRLA_PRESCALER(TCC_CTRLA_PRESCALER_DIV1_Val);

    TCC2->WAVE.reg = TCC_WAVE_WAVEGEN_NFRQ;         // Select NFRQ as waveform
  while (TCC2->SYNCBUSY.bit.WAVE);                // Wait for synchronization
  
  // Set the period (the number to count to (TOP) before resetting timer)
  TCC2->PER.reg = period;
  while (TCC2->SYNCBUSY.bit.PER);

  TCC2->EVCTRL.reg = TCC_EVCTRL_OVFEO; // Overflow/underflow counter event is enabled

  TCC2->INTENSET.reg = TCC_INTENSET_OVF; // Enable the Overflow interrupt

  NVIC_EnableIRQ(TCC2_IRQn);

// Step 1: Issue READYSYNC command
TCC2->CTRLBSET.reg = TCC_CTRLBSET_CMD_READSYNC;

// Step 2: Wait until the command is fully executed
while (TCC2->SYNCBUSY.bit.CTRLB); // or while (TCC2->SYNCBUSY.reg);


  TCC2->CTRLA.reg |= TCC_CTRLA_ENABLE;
  while (TCC2->SYNCBUSY.bit.ENABLE);              // Wait for synchronization
}

void TCC2_Handler(){

  if (TCC2->INTFLAG.bit.OVF == 1) {  // A overflow caused the interrupt
     //TCC2->INTFLAG.bit.OVF = 1;    // writing a one clears the flag ovf flag
     OV_count++;
       if(OV_count == 1){
     // Step 1: Issue READYSYNC command

       TCC2->CTRLBSET.reg = TCC_CTRLBSET_CMD_READSYNC;
  
  // Step 2: Wait until the command is fully executed
        while (TCC2->SYNCBUSY.bit.CTRLB); // or while (TCC2->SYNCBUSY.reg);
        
        count = TCC2->COUNT.reg;
        Rcount = (float)count + (OV_count*48000);
        //Serial.print("Counter: ");
        Serial.println(Rcount);
       // digitalWrite(LED,state);
          OV_count = 0;
      //    state = !state;
    }
     TCC2->INTFLAG.bit.OVF = 1; 
  }
}

/**************************************************************************
Object: Configures the FDPLL to run at 96MHz using the Clock source of GCLK1
that had previously been configured to output a clock of 32KHz.
Parameters: none
Return: Nothing
**************************************************************************/
//void CLOCKS_FDPLL_Init(void)
//{
//  GCLK_CLKCTRL_Type clkctrl =
//  {
//    bit.ID = SYSCTRL_GCLK_ID_FDPLL,
//    .bit.GEN = GCLK_GENERATOR_1,
//    .bit.CLKEN = true,
//    .bit.WRTLOCK = false
//  };
//  GCLK->CLKCTRL.reg = clkctrl.reg;


//  SYSCTRL_DPLLRATIO_Type dpllratio =
//  {
//    .bit.LDRFRAC = 0,
//    .bit.LDR = 2999
//  };
//  SYSCTRL->DPLLRATIO.reg = dpllratio.reg;
//
//  SYSCTRL_DPLLCTRLB_Type dpllctrlb =
//  {
//    .bit.DIV = 0,     /* set the CLK_DPLL_REF1 clock division factor */
//    .bit.LBYPASS = 0, /* select Lock Bypass mode */
//    .bit.LTIME = 0,   /* select the DPLL lock time-out */
//    .bit.REFCLK = 2,  /* select the DPLL clock reference */
//    .bit.WUF = 0,     /* select if output is gated during lock time */
//    .bit.LPEN = 0,    /* Low-Power Enable */
//    .bit.FILTER = 0  /* select Proportional Integral Filter mode */
//  };
//  SYSCTRL->DPLLCTRLB.reg = dpllctrlb.reg;
//
//  SYSCTRL_DPLLCTRLA_Type dpllctrla =
//  {
//    .bit.ONDEMAND = 0,
//    .bit.RUNSTDBY = 0,
//    .bit.ENABLE = 1
//  };
//  SYSCTRL->DPLLCTRLA.reg = dpllctrla.reg;
//}
//
//void make_glck1_32kHz()
//{
//    // setup GENDIV divisor
//    GCLK_GENDIV_Type gendiv =
//    {
//        .bit.DIV = 1,          // <==== changed
//        .bit.ID = 1          // GCLK_GENERATOR_1
//    };
//    GCLK->GENDIV.reg = gendiv.reg;
//    // setup GENCTRL
//    GCLK_GENCTRL_Type genctrl =
//    {
//        .bit.RUNSTDBY = 0,        // Run in Standby
//        .bit.DIVSEL = 0,          // Divide Selection (0=linear, 1=powers of 2)
//        .bit.OE = 0,              // Output Enable to observe on a port pin
//        .bit.OOV = 0,             // Output Off Value
//        .bit.IDC = 1,             // Improve Duty Cycle
//        .bit.GENEN = 1,           // enable this GCLK
//        // select GCLK source
//        .bit.SRC = GCLK_SOURCE_OSC32K,  // <==== changed
//        // select GCLK1 to output on
//        .bit.ID = 1              // GCLK_GENERATOR_1
//    };
//    GCLK->GENCTRL.reg = genctrl.reg;
//    // wait for synchronization to complete
//    while (GCLK->STATUS.bit.SYNCBUSY);
//}
