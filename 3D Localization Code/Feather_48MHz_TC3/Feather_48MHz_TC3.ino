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

PLLClockConfigure();

TimerConfigure();

}

void loop() {
  // put your main code here, to run repeatedly:
         if(OV_count == 367){
        count = TC->COUNT.reg;
        Rcount = (float)count + (OV_count*65535);
        //Serial.print("Counter: ");
        Serial.println(Rcount);
       digitalWrite(LED,state);
          OV_count = 0;
          state = !state;
    }

}

void PLLClockConfigure(){
// Set up the generic clock (GCLK 4) used to clock timers
//
//  GCLK->GENCTRL.reg = (uint16_t) (GCLK_GENCTRL_IDC|   // Have it run in standby
//                     GCLK_GENCTRL_GENEN|         // Enable GCLK 4
//                     GCLK_GENCTRL_SRC_GCLK0 |   // Set the 48MHz clock source
//                     GCLK_GENCTRL_ID(4));          // Select GCLK 4
//  while (GCLK->STATUS.bit.SYNCBUSY);              // Wait for synchronization
//  
//  GCLK->GENDIV.reg = GCLK_GENDIV_DIV(1) |          // Divide the 48MHz clock source by divisor 1: 48MHz/1= 48MHz
//                    GCLK_GENDIV_ID(4);            // Select Generic Clock (GCLK) 4
//  while (GCLK->STATUS.bit.SYNCBUSY);              // Wait for synchronization

    GCLK->CLKCTRL.reg = GCLK_CLKCTRL_CLKEN |       // Enable the Generic Clock
                     GCLK_CLKCTRL_GEN_GCLK0 |   // Select Source for Generic Clock to be GLCK2
                     GCLK_CLKCTRL_ID_TCC2_TC3;  // Output clock to Timer 2-3 clock
 while (GCLK->STATUS.bit.SYNCBUSY);     
 
}

void TimerConfigure(){
  
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
