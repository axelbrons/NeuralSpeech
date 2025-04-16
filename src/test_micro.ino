volatile uint32_t interruptCount = 0;
unsigned long lastPrint = 0;

void setup() {
  Serial.begin(115200); // Important : vitesse haute

  analogReadResolution(12);
  analogWriteResolution(12);

  pmc_set_writeprotect(false);
  pmc_enable_periph_clk(ID_TC0);

  TC_Configure(TC0, 0,
               TC_CMR_TCCLKS_TIMER_CLOCK1 |
               TC_CMR_WAVE |
               TC_CMR_WAVSEL_UP_RC);

  TC_SetRC(TC0, 0, 1312); // 42MHz / 2 / 32000 = 656.25 → x2 pour stabilité
  TC_Start(TC0, 0);

  TC0->TC_CHANNEL[0].TC_IER = TC_IER_CPCS;
  NVIC_EnableIRQ(TC0_IRQn);
}

void loop() {
  if (millis() - lastPrint >= 1000) {
    lastPrint = millis();
    noInterrupts();
    uint32_t count = interruptCount;
    interruptCount = 0;
    interrupts();

    //Serial.print("Interrupts en 1 seconde : ");
    Serial.println(count); // Doit être proche de 32000
  }
}

void TC0_Handler() {
  TC_GetStatus(TC0, 0); // Clear flag

  int sample = analogRead(A0);
  analogWrite(DAC0, sample);

  interruptCount++;
}
