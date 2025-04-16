#define ARM_MATH_CM3
#include <Arduino.h>
#undef PI
#include <arm_math.h>

// Audio Configuration
#define SAMPLE_RATE 8000          // 8kHz
#define TOTAL_SAMPLES 8000        // Exactly 1 second of data
#define INITIAL_DELAY_MS 2000     // 2 second initial delay

// FIR filter coefficients
const float firCoeffs[15] = {
  0.02, 0.04, 0.06, 0.08, 0.10,
  0.12, 0.14, 0.16, 0.14, 0.12,
  0.10, 0.08, 0.06, 0.04, 0.02
};

// Circular buffer for sampling
#define BUFFER_SIZE 32
volatile int16_t sampleBuffer[BUFFER_SIZE];
volatile uint8_t bufferIndex = 0;
volatile uint8_t downsampleCounter = 0;

// Capture buffer
int16_t captureBuffer[TOTAL_SAMPLES];
volatile uint16_t captureIndex = 0;
volatile bool captureComplete = false;
uint32_t startTime;

void setup() {
  Serial.begin(115200);
  while (!Serial); // Wait for serial connection

  // Record startup time
  startTime = millis();

  // Configure ADC
  PMC->PMC_PCER1 |= PMC_PCER1_PID37;
  ADC->ADC_MR = ADC_MR_PRESCAL(1) | ADC_MR_STARTUP_SUT8 | 
               ADC_MR_TRACKTIM(15) | ADC_MR_SETTLING_AST3;
  ADC->ADC_CHER = ADC_CHER_CH0;
  
  // Configure timer for 32kHz sampling
  pmc_set_writeprotect(false);
  pmc_enable_periph_clk(ID_TC0);
  TC_Configure(TC0, 0, TC_CMR_TCCLKS_TIMER_CLOCK1 | TC_CMR_WAVE | TC_CMR_WAVSEL_UP_RC);
  TC_SetRC(TC0, 0, 656);  // 42MHz/2/656 ≈ 32kHz
  TC_Start(TC0, 0);
  
  TC0->TC_CHANNEL[0].TC_IER = TC_IER_CPCS;
  NVIC_EnableIRQ(TC0_IRQn);

  Serial.println("System initializing - 2 second delay before capture...");
}

void loop() {
  // Check if initial delay has passed and capture isn't complete
  if (!captureComplete && (millis() - startTime) >= INITIAL_DELAY_MS) {
    // Wait for capture to complete
    while (!captureComplete) {
      delay(1);
    }
    
    // Print all 8000 samples
    Serial.println("Sample_Number,Value");
    for (int i = 0; i < TOTAL_SAMPLES; i++) {
      //Serial.print(i);
      //Serial.print(",");
      Serial.println(captureBuffer[i]);
    }
    
    Serial.println("END_OF_DATA");
    
    // Stop further processing
    while(1) { delay(1000); }
  }
}

int16_t applyFIRFilter() {
  float output = 0;
  int idx;
  
  for (int i = 0; i < 15; i++) {
    idx = (bufferIndex - i + BUFFER_SIZE) % BUFFER_SIZE;
    output += firCoeffs[i] * sampleBuffer[idx];
  }
  return (int16_t)output;
}

void TC0_Handler() {
  TC_GetStatus(TC0, 0); // Clear interrupt flag
  
  // Skip processing if capture is complete
  if (captureComplete) return;
  
  // Read ADC sample
  ADC->ADC_CR = ADC_CR_START;
  while (!(ADC->ADC_ISR & ADC_ISR_DRDY));
  int16_t sample = (int16_t)(ADC->ADC_CDR[0] - 2048);
  
  // Store in circular buffer
  sampleBuffer[bufferIndex] = sample;
  bufferIndex = (bufferIndex + 1) % BUFFER_SIZE;
  
  // Downsample processing (32kHz -> 8kHz)
  if (++downsampleCounter >= 4) {
    downsampleCounter = 0;
    
    // Apply FIR filter after initial delay
    if ((millis() - startTime) >= INITIAL_DELAY_MS) {
      int16_t filtered = applyFIRFilter();
      
      // Store in capture buffer
      if (captureIndex < TOTAL_SAMPLES) {
        captureBuffer[captureIndex++] = filtered;
      } 
      else if (!captureComplete) {
        captureComplete = true;
        TC_Stop(TC0, 0); // Stop the timer interrupt
      }
    }
  }
}
