#include <Arduino.h>

// Audio Configuration
const int INPUT_RATE = 32000;    // Initial sampling rate
const int OUTPUT_RATE = 8000;    // Target rate after downsampling
const int DOWNSAMPLE_FACTOR = INPUT_RATE / OUTPUT_RATE; // 4:1 downsampling
const int BUFFER_SIZE = 512;     // Circular buffer size
const int FIR_TAPS = 25;         // Number of filter coefficients

// FIR Low-Pass Filter Coefficients (3.6kHz cutoff at 32kHz, 25 taps)
// Designed using Hamming window, Fs=32kHz, Fc=3.6kHz
const float firCoeffs[FIR_TAPS] = {
  -0.001312, -0.002147, -0.002100, 0.000668, 0.006544,
  0.011934, 0.009244, -0.003000, -0.022842, -0.034629,
  0.073027, 0.073027, -0.034629, -0.022842, -0.003000,
  0.009244, 0.011934, 0.006544, 0.000668, -0.002100,
  -0.002147, -0.001312, 0.000000, 0.000000, 0.000000
};

// Circular buffers
volatile uint32_t adcBuffer[BUFFER_SIZE];  // Raw ADC samples
volatile int16_t filteredBuffer[BUFFER_SIZE]; // Filtered samples
volatile int16_t outputBuffer[BUFFER_SIZE/DOWNSAMPLE_FACTOR]; // Downsampled output
volatile uint32_t writeIndex = 0;
volatile uint32_t readIndex = 0;
volatile bool bufferReady = false;

void setupADC() {
  PMC->PMC_PCER1 |= PMC_PCER1_PID37; // Enable ADC clock
  ADC->ADC_MR = ADC_MR_PRESCAL(255)  // Clock divider
              | ADC_MR_STARTUP_SUT64 // Startup time
              | ADC_MR_TRACKTIM(15)  // Tracking time
              | ADC_MR_SETTLING_AST3;// Settling time
  ADC->ADC_CHER = ADC_CHER_CH7;      // Enable channel 7 (A0)

  // Configure Timer Counter 0 for 32kHz sampling
  PMC->PMC_PCER0 |= PMC_PCER0_PID27; // Enable TC0
  TC0->TC_CHANNEL[0].TC_CMR = TC_CMR_TCCLKS_TIMER_CLOCK4 | TC_CMR_CPCTRG; 
  TC0->TC_CHANNEL[0].TC_RC = 656250 / INPUT_RATE - 1;
  TC0->TC_CHANNEL[0].TC_IER = TC_IER_CPCS;
  NVIC_EnableIRQ(TC0_IRQn);

  // Configure DMA
  PMC->PMC_PCER1 |= PMC_PCER1_PID39; // Enable PDC
  ADC->ADC_PTCR = ADC_PTCR_RXTDIS | ADC_PTCR_TXTDIS;
  ADC->ADC_RPR = (uint32_t)adcBuffer;
  ADC->ADC_RCR = BUFFER_SIZE;
  ADC->ADC_PTCR = ADC_PTCR_RXTEN;
}

// FIR Filter Implementation
int16_t applyFIRFilter(uint32_t index) {
  float output = 0;
  for (int i = 0; i < FIR_TAPS; i++) {
    uint32_t tapIndex = (index - i + BUFFER_SIZE) % BUFFER_SIZE;
    output += firCoeffs[i] * (adcBuffer[tapIndex] - 2048); // Convert to signed
  }
  return (int16_t)output;
}

void TC0_Handler() {
  TC0->TC_CHANNEL[0].TC_SR; // Clear interrupt flag
  ADC->ADC_CR = ADC_CR_START; // Start new conversion
  
  // Process every sample
  static uint8_t downsampleCounter = 0;
  
  // Apply FIR filter
  filteredBuffer[writeIndex] = applyFIRFilter(writeIndex);
  
  // Downsample 4:1
  if (++downsampleCounter >= DOWNSAMPLE_FACTOR) {
    downsampleCounter = 0;
    outputBuffer[writeIndex/DOWNSAMPLE_FACTOR] = filteredBuffer[writeIndex];
  }
  
  // Update circular buffer index
  writeIndex = (writeIndex + 1) % BUFFER_SIZE;
  
  // Signal when output buffer is full
  if (writeIndex % (BUFFER_SIZE/DOWNSAMPLE_FACTOR) == 0) {
    bufferReady = true;
  }
}

void setup() {
  Serial.begin(115200);
  setupADC();
  TC0->TC_CHANNEL[0].TC_CCR = TC_CCR_CLKEN | TC_CCR_SWTRG;
  Serial.println("Setup finished");
}

void loop() {
  Serial.println("Begin the loop");
  if (bufferReady) {
    noInterrupts();
    // Send downsampled data
    for (uint32_t i = 0; i < BUFFER_SIZE/DOWNSAMPLE_FACTOR; i++) {
      Serial.println(outputBuffer[i]);
    }
    bufferReady = false;
    interrupts();
  }
}
