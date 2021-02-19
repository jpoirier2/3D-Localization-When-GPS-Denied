// I have no idea why these definitions were left out of the Arduino headers
#define GCLK_GENCTRL_SRC_DPLL96M_Val 0x8ul  // (GCLK_GENCTRL) DPLL96M output
#define GCLK_GENCTRL_SRC_DPLL96M (GCLK_GENCTRL_SRC_DPLL96M_Val << \
                                  GCLK_GENCTRL_SRC_Pos)

// Number to count to with PWM (TOP value). Frequency can be calculated by
// freq = GCLK4_freq / (TCC2_prescaler * (1 + TOP_value))
// With TOP of 95, we get a 1 MHz square wave in this example.
// TCC2 is 16 bits
uint16_t period = 96 - 1;
 
void setup() {

  // Flash wait states NVMCTRL->CTRLB.bit.RWS set to 1 by Arduino framework

  // Configure generic clock generator 4
  GCLK->GENCTRL.reg = GCLK_GENCTRL_IDC |          // Improve duty cycle
                      GCLK_GENCTRL_GENEN |        // Enable generic clock gen
                      GCLK_GENCTRL_SRC_DFLL48M |  // Select 48MHz as source
                      GCLK_GENCTRL_ID(4);         // Apply to GCLK4
  while (GCLK->STATUS.bit.SYNCBUSY);              // Wait for synchronization

  // Set clock divider of 48 to generic clock generator 4 (1 MHz)
  GCLK->GENDIV.reg = GCLK_GENDIV_DIV(48) |        // Divide 48 MHz by 48
                     GCLK_GENDIV_ID(4);           // Apply to GCLK4 4
  while (GCLK->STATUS.bit.SYNCBUSY);              // Wait for synchronization

  // Enable GCLK4 and connect it to GCLK_DPLL
  GCLK->CLKCTRL.reg = GCLK_CLKCTRL_CLKEN |        // Enable generic clock
                      GCLK_CLKCTRL_GEN_GCLK4 |    // Select GCLK4
                      GCLK_CLKCTRL_ID(1);         // Feed GCLK4 to GCLK_DPLL
  while (GCLK->STATUS.bit.SYNCBUSY);              // Wait for synchronization

  // Set DPLL ratio to 1 MHz * (95 + 1) = 96 MHz
  SYSCTRL->DPLLRATIO.reg = SYSCTRL_DPLLRATIO_LDRFRAC(0) | // Ratio fractional
                           SYSCTRL_DPLLRATIO_LDR(95);     // Ratio

  // Configure DPLL to disregard phase lock and select GCLK as source
  SYSCTRL->DPLLCTRLB.reg = SYSCTRL_DPLLCTRLB_LBYPASS |    // Bypass lock
                           SYSCTRL_DPLLCTRLB_WUF |        // Wake up fast
    SYSCTRL_DPLLCTRLB_REFCLK(SYSCTRL_DPLLCTRLB_REFCLK_GCLK_Val); // Select GCLK
  
  // Enable DPLL
  SYSCTRL->DPLLCTRLA.reg |= SYSCTRL_DPLLCTRLA_ENABLE;
  
  // Configure generic clock generator 5
  GCLK->GENCTRL.reg = GCLK_GENCTRL_IDC |          // Improve duty cycle
                      GCLK_GENCTRL_GENEN |        // Enable generic clock gen
                      GCLK_GENCTRL_SRC_DPLL96M |  // Select DPLL as source
                      GCLK_GENCTRL_ID(5);         // Apply to GCLK5
  while (GCLK->STATUS.bit.SYNCBUSY);              // Wait for synchronization

  // Set clock divider of 1 to generic clock generator 5 (96 MHz)
  GCLK->GENDIV.reg = GCLK_GENDIV_DIV(1) |        // Divide 96 MHz by 1
                     GCLK_GENDIV_ID(5);           // Apply to GCLK5
  while (GCLK->STATUS.bit.SYNCBUSY);              // Wait for synchronization

  // Enable GCLK5 and connect it to TCC2 and TC3
  GCLK->CLKCTRL.reg = GCLK_CLKCTRL_CLKEN |        // Enable generic clock
                      GCLK_CLKCTRL_GEN_GCLK5 |    // Select GCLK5
                      GCLK_CLKCTRL_ID_TCC2_TC3;   // Feed GCLK5 to TCC2/TC3
  while (GCLK->STATUS.bit.SYNCBUSY);              // Wait for synchronization
  
  // Divide counter by 1 giving 96 MHz (10.42 ns) on each TCC2 tick
  TCC2->CTRLA.reg |= TCC_CTRLA_PRESCALER(TCC_CTRLA_PRESCALER_DIV1_Val);

  // Use "Normal PWM" (single-slope PWM): count up to PER, match on CC[n]
  TCC2->WAVE.reg = TCC_WAVE_WAVEGEN_NPWM;         // Select NPWM as waveform
  while (TCC2->SYNCBUSY.bit.WAVE);                // Wait for synchronization

  // Set the period (the number to count to (TOP) before resetting timer)
  TCC2->PER.reg = period;
  while (TCC2->SYNCBUSY.bit.PER);

  // Set PWM signal to output 50% duty cycle
  // n for CC[n] is determined by n = x % 4 where x is from WO[x]
  TCC2->CC[0].reg = period / 2;
  while (TCC2->SYNCBUSY.bit.CC0);

  // Configure PA16 (D11 on Arduino Zero) to be output
  PORT->Group[PORTA].DIRSET.reg = PORT_PA16;      // Set pin as output
  PORT->Group[PORTA].OUTCLR.reg = PORT_PA16;      // Set pin to low

  // Enable the port multiplexer for PA16
  PORT->Group[PORTA].PINCFG[16].reg |= PORT_PINCFG_PMUXEN;

  // Connect TCC2 timer to PA16. Function E is TCC2/WO[0] for PA16.
  // n for PMUX[n] is given by floor(pin_num / 2)
  // Odd pin num (2*n + 1): use PMUXO
  // Even pin num (2*n): use PMUXE
  PORT->Group[PORTA].PMUX[8].reg = PORT_PMUX_PMUXE_E;

  // Enable output (start PWM)
  TCC2->CTRLA.reg |= TCC_CTRLA_ENABLE;
  while (TCC2->SYNCBUSY.bit.ENABLE);              // Wait for synchronization
}

void loop() {
  // Do nothing
}
