volatile uint32_t interruptCount = 0;

// FIR filter coefficients (reduced to 15 taps for better voice quality)
const float firCoeffs[15] = {
  0.02, 0.04, 0.06, 0.08, 0.10,
  0.12, 0.14, 0.16, 0.14, 0.12,
  0.10, 0.08, 0.06, 0.04, 0.02
};

// Circular buffer for FIR filtering
#define BUFFER_SIZE 32
volatile int16_t sampleBuffer[BUFFER_SIZE];
volatile uint8_t bufferIndex = 0;

// Downsampling
volatile uint8_t downsampleCounter = 0;

// Serial output
#define SERIAL_BUFFER_SIZE 200
int16_t serialBuffer[SERIAL_BUFFER_SIZE];
volatile uint16_t serialIndex = 0;

void setup() {
  Serial.begin(115200);
  
  // Configure ADC
  PMC->PMC_PCER1 |= PMC_PCER1_PID37;  // Enable ADC clock
  ADC->ADC_MR = ADC_MR_PRESCAL(1) |   // ADCClock = MCK/4 = 21MHz
                ADC_MR_STARTUP_SUT8 | // 8 clock startup
                ADC_MR_TRACKTIM(15) | // 15 clock tracking
                ADC_MR_SETTLING_AST3; // 3 clock settling
  ADC->ADC_CHER = ADC_CHER_CH0;       // Enable channel 0 (A0)
  
  // Configure timer for 32kHz sampling
  pmc_set_writeprotect(false);
  pmc_enable_periph_clk(ID_TC0);
  TC_Configure(TC0, 0, TC_CMR_TCCLKS_TIMER_CLOCK1 | TC_CMR_WAVE | TC_CMR_WAVSEL_UP_RC);
  TC_SetRC(TC0, 0, 656);  // 42MHz/2/656 ≈ 32kHz
  TC_Start(TC0, 0);
  
  TC0->TC_CHANNEL[0].TC_IER = TC_IER_CPCS;
  NVIC_EnableIRQ(TC0_IRQn);

  Serial.println("Voice streaming started...");
}

void loop() {
  // Send buffered audio data when ready
  if (serialIndex >= SERIAL_BUFFER_SIZE) {
    noInterrupts();
    uint16_t currentIndex = serialIndex;
    int16_t tempBuffer[SERIAL_BUFFER_SIZE];
    for (int i = 0; i < currentIndex; i++) {
      tempBuffer[i] = serialBuffer[i];
    }
    serialIndex = 0;
    interrupts();

    for (int i = 0; i < currentIndex; i++) {
      Serial.println(tempBuffer[i]);
    }
  }
}

int16_t applyFIRFilter() {
  float output = 0;
  int idx;
  
  for (int i = 0; i < 15; i++) {
    idx = (bufferIndex - i + BUFFER_SIZE) % BUFFER_SIZE;
    output += firCoeffs[i] * sampleBuffer[idx];
  }
  return (int16_t)output; // Removed scaling for cleaner audio
}

void TC0_Handler() {
  TC_GetStatus(TC0, 0); // Clear interrupt flag
  
  // Read ADC sample (12-bit unsigned to 16-bit signed)
  ADC->ADC_CR = ADC_CR_START;
  while (!(ADC->ADC_ISR & ADC_ISR_DRDY));
  int16_t sample = (int16_t)(ADC->ADC_CDR[0] - 2048); // Center around 0
  
  // Store in circular buffer
  sampleBuffer[bufferIndex] = sample;
  bufferIndex = (bufferIndex + 1) % BUFFER_SIZE;
  
  // Downsample processing (32kHz -> 8kHz)
  downsampleCounter++;
  if (downsampleCounter >= 4) {
    downsampleCounter = 0;
    
    // Apply gentler FIR filter
    int16_t filtered = applyFIRFilter();
    //int16_t filtered = sample;
    // Store in serial buffer
    if (serialIndex < SERIAL_BUFFER_SIZE) {
      serialBuffer[serialIndex++] = filtered;
    }
  }
  
  interruptCount++;
}
