volatile uint16_t sample = 0;

void setup() {
  analogReadResolution(12);
  analogWriteResolution(12);
  pmc_set_writeprotect(false);
  pmc_enable_periph_clk(ID_TC0);

  TC_Configure(TC0, 0,
               TC_CMR_TCCLKS_TIMER_CLOCK1 |
               TC_CMR_WAVE |
               TC_CMR_WAVSEL_UP_RC);

  TC_SetRC(TC0, 0, 656); // 42MHz / 2 / 4000 = 5250 → pour 4kHz // Temporairement puisque avec 32kHz c'était trop pareil
  TC_Start(TC0, 0);

  TC0->TC_CHANNEL[0].TC_IER = TC_IER_CPCS;
  NVIC_EnableIRQ(TC0_IRQn);
}

void loop() {}

void TC0_Handler() {
  TC_GetStatus(TC0, 0);
  sample = analogRead(A0);
  analogWrite(DAC0, sample);
}
