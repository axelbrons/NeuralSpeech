volatile uint32_t interruptCount = 0;
unsigned long lastPrint = 0;

// FIR filter coefficients (25 taps)
const float firCoeffs[25] = {
  -0.002312, -0.003347, -0.002100, 0.002668, 0.009544,
  0.013934, 0.011244, 0.000000, -0.015842, -0.027629,
  -0.025756, -0.006598, 0.025377, 0.056098, 0.073027,
  0.073027, 0.056098, 0.025377, -0.006598, -0.025756,
  -0.027629, -0.015842, 0.000000, 0.011244, 0.013934
};

// Circular buffer for FIR filtering
#define BUFFER_SIZE 32
volatile int16_t sampleBuffer[BUFFER_SIZE];
volatile uint8_t bufferIndex = 0;

// Downsampling and monitoring
volatile uint8_t downsampleCounter = 0;
volatile unsigned long conversionTime = 0;
volatile uint8_t displayBufferCounter = 0;

// Serial output control
#define SERIAL_BUFFER_SIZE 100
int16_t serialBuffer[SERIAL_BUFFER_SIZE];
uint16_t serialIndex = 0;
unsigned long lastSerialSend = 0;
const unsigned long serialSendInterval = 100; // ms

void setup() {
  Serial.begin(115200);
  
  // Configure ADC resolution (12-bit)
  ADC->ADC_MR |= ADC_MR_LOWRES_BITS_12;
  
  // Enable ADC clock
  PMC->PMC_PCER1 |= PMC_PCER1_PID37; // ADC peripheral ID
  
  // Configure ADC mode register
  ADC->ADC_MR = 
    ADC_MR_PRESCAL(1) |         // ADCClock = MCK/2*(PRESCAL+1) = 84MHz/4 = 21MHz
    ADC_MR_STARTUP_SUT8 |       // Startup time = 8 periods of ADCClock
    ADC_MR_TRACKTIM(15) |       // Tracking time = 15 periods of ADCClock
    ADC_MR_SETTLING_AST3;       // Settling time = 3 periods of ADCClock
  
  // Enable channel 0 (A0)
  ADC->ADC_CHER = ADC_CHER_CH0;
  
  // Configure DAC
  pmc_enable_periph_clk(ID_DACC); // Enable DAC clock
  DACC->DACC_MR = 
    DACC_MR_TRGEN_DIS |         // Free running mode
    DACC_MR_USER_SEL_CHANNEL0 | // Channel 0 (DAC0)
    DACC_MR_REFRESH(1) |        // Refresh period
    DACC_MR_STARTUP_8;          // Startup time
  
  DACC->DACC_CHER = DACC_CHER_CH0; // Enable DAC channel 0
  
  // Initialize sample buffer
  for (int i = 0; i < BUFFER_SIZE; i++) {
    sampleBuffer[i] = 0;
  }

  // Configure TC0 for 32kHz sampling
  pmc_set_writeprotect(false);
  pmc_enable_periph_clk(ID_TC0);

  TC_Configure(TC0, 0,
               TC_CMR_TCCLKS_TIMER_CLOCK1 |  // MCK/2 = 42MHz
               TC_CMR_WAVE |
               TC_CMR_WAVSEL_UP_RC);

  TC_SetRC(TC0, 0, 656);  // 42MHz / 2 / 656 ≈ 32kHz
  TC_Start(TC0, 0);

  TC0->TC_CHANNEL[0].TC_IER = TC_IER_CPCS;
  NVIC_EnableIRQ(TC0_IRQn);

  Serial.println("System initialized with direct register access");
}

void loop() {
  // Monitoring and statistics
  if (millis() - lastPrint >= 1000) {
    lastPrint = millis();
    noInterrupts();
    uint32_t count = interruptCount;
    unsigned long avgConversion = conversionTime / max(count, 1);
    conversionTime = 0;
    interruptCount = 0;
    interrupts();
/*
    Serial.print("Interrupts/s: ");
    Serial.print(count);
    Serial.print(" | Avg conversion: ");
    Serial.print(avgConversion);
    Serial.println("μs");*/
  }

  // Buffer content display (every 10 downsampling cycles)
  if (displayBufferCounter >= 10) {
    displayBufferCounter = 0;
    noInterrupts();
    int16_t tempBuffer[BUFFER_SIZE];
    uint8_t currentIndex = bufferIndex;
    for (int i = 0; i < BUFFER_SIZE; i++) {
      tempBuffer[i] = sampleBuffer[(currentIndex + i) % BUFFER_SIZE];
    }
    interrupts();

    //Serial.println("Buffer Contents:");
    for (int i = 0; i < BUFFER_SIZE; i++) {
      //Serial.println(tempBuffer[i]);
    }
    
  }

  // Audio data transmission
  if (millis() - lastSerialSend >= serialSendInterval) {
    lastSerialSend = millis();
    noInterrupts();
    uint16_t currentIndex = serialIndex;
    int16_t tempAudio[SERIAL_BUFFER_SIZE];
    for (int i = 0; i < currentIndex; i++) {
      tempAudio[i] = serialBuffer[i];
    }
    serialIndex = 0;
    interrupts();

    //Serial.println("Audio Data:");
    for (int i = 0; i < currentIndex; i++) {
      Serial.println(tempAudio[i]);
    }
  }
}

int16_t applyFIRFilter() {
  float output = 0;
  int idx;
  
  for (int i = 0; i < 25; i++) {
    idx = (bufferIndex - i + BUFFER_SIZE) % BUFFER_SIZE;
    output += firCoeffs[i] * sampleBuffer[idx];
  }
  return (int16_t)(output * 256);
}

void TC0_Handler() {
  unsigned long startTime = micros();
  TC_GetStatus(TC0, 0); // Clear interrupt flag

  // Start ADC conversion
  ADC->ADC_CR = ADC_CR_START;
  
  // Wait for conversion to complete
  while (!(ADC->ADC_ISR & ADC_ISR_DRDY));
  
  // Read ADC value and center around 0 (12-bit signed)
  int16_t sample = (int16_t)(ADC->ADC_CDR[0] - 2048);
  
  // Store in circular buffer
  sampleBuffer[bufferIndex] = sample;
  bufferIndex = (bufferIndex + 1) % BUFFER_SIZE;
  
  // Downsample processing
  downsampleCounter++;
  if (downsampleCounter >= 4) {
    downsampleCounter = 0;
    displayBufferCounter++;
    
    // Apply FIR filter
    int16_t filtered = applyFIRFilter();
    
    // Write to DAC (convert back to 0-4095 range)
    DACC->DACC_CDR = filtered + 2048;

    // Store in serial buffer if space available
    if (serialIndex < SERIAL_BUFFER_SIZE) {
      serialBuffer[serialIndex++] = filtered;
    }
  }

  conversionTime += micros() - startTime;
  interruptCount++;
}
