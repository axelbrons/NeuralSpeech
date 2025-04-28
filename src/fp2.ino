volatile uint16_t sample = 0;
#define BUFFER_SIZE 33
volatile uint16_t rawSample = 0;
volatile float firBuffer[BUFFER_SIZE];
volatile int bufferIndex = 0;
volatile bool downsampleReady = false;
volatile int downsampleCounter = 0;

const float firCoeffs[BUFFER_SIZE] = { -0.0012, -0.0023, -0.0036, -0.0047, -0.0051, -0.0039, 0.0000, 0.0073,
  0.0176, 0.0301, 0.0434, 0.0553, 0.0635, 0.0669, 0.0650, 0.0584,
  0.0485, 0.0368, 0.0248, 0.0136, 0.0041, -0.0027, -0.0065, -0.0074,
  -0.0062, -0.0039, -0.0016, -0.0003,  0.0002,  0.0001,  0.0000,  0.0000,  0.0000 };


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
  rawSample = analogRead(A0);

  firBuffer[bufferIndex] = rawSample;
  bufferIndex = (bufferIndex + 1) % BUFFER_SIZE;

  // Downsample à 8 kHz = 1/4 des échantillons
  downsampleCounter++;
  if (downsampleCounter == 4) {
    downsampleCounter = 0;

    float filtered = 0;
    int idx = bufferIndex;
    for (int i = 0; i < BUFFER_SIZE; i++) {
      idx = (idx != 0) ? idx - 1 : BUFFER_SIZE - 1;
      filtered += firBuffer[idx] * firCoeffs[i];
    }

    // On reconvertit vers analogique (DAC)
    analogWrite(DAC0, (uint16_t)filtered);
  }
}
