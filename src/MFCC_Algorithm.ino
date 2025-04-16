#define __SAM3X8E__
#define ARM_MATH_CM3
#include <Arduino.h>
#undef PI  // Undefine Arduino's PI to avoid conflict with CMSIS
#include <arm_math.h>

// Audio Configuration
#define SAMPLE_RATE 8000
#define FRAME_SIZE 128
#define FRAME_OVERLAP 64
#define NUM_MFCC 13
#define NUM_MEL_FILTERS 20

// FIR filter coefficients
const float firCoeffs[15] = {
  0.02, 0.04, 0.06, 0.08, 0.10,
  0.12, 0.14, 0.16, 0.14, 0.12,
  0.10, 0.08, 0.06, 0.04, 0.02
};

// Circular buffer
#define BUFFER_SIZE 32
volatile int16_t sampleBuffer[BUFFER_SIZE];
volatile uint8_t bufferIndex = 0;
volatile uint8_t downsampleCounter = 0;

// Frame processing
int16_t audioBuffer[FRAME_SIZE + FRAME_OVERLAP];
uint16_t audioBufferIndex = 0;
bool frameReady = false;

// MFCC variables
float hammingWindow[FRAME_SIZE];
float mfccCoefficients[NUM_MFCC];
float dctMatrix[NUM_MFCC][NUM_MEL_FILTERS];
float melFilters[NUM_MEL_FILTERS][FRAME_SIZE/2];

// ARM DSP structures
arm_rfft_instance_f32 fftInstance;
arm_cfft_radix4_instance_f32 fftRadixInstance;

void initMelFilters() {
  // Simplified Mel filterbank
  for (int m = 0; m < NUM_MEL_FILTERS; m++) {
    int start = m * (FRAME_SIZE/2) / NUM_MEL_FILTERS;
    int end = (m + 1) * (FRAME_SIZE/2) / NUM_MEL_FILTERS;
    for (int k = start; k < end; k++) {
      melFilters[m][k] = 1.0f;
    }
  }
}

void initDCTMatrix() {
  for (int k = 0; k < NUM_MFCC; k++) {
    for (int m = 0; m < NUM_MEL_FILTERS; m++) {
      dctMatrix[k][m] = cosf(PI * k * (m + 0.5f) / NUM_MEL_FILTERS);
    }
  }
}

void setup() {
  Serial.begin(115200);
  while (!Serial);
  
  // Initialize Hamming window
  for (int i = 0; i < FRAME_SIZE; i++) {
    hammingWindow[i] = 0.54f - 0.46f * cosf(2 * PI * i / (FRAME_SIZE - 1));
  }

  // Initialize FFT instances
  arm_rfft_init_f32(&fftInstance, &fftRadixInstance, FRAME_SIZE, 0, 1);
  
  initMelFilters();
  initDCTMatrix();

  // Configure ADC
  PMC->PMC_PCER1 |= PMC_PCER1_PID37;
  ADC->ADC_MR = ADC_MR_PRESCAL(1) | ADC_MR_STARTUP_SUT8 | 
               ADC_MR_TRACKTIM(15) | ADC_MR_SETTLING_AST3;
  ADC->ADC_CHER = ADC_CHER_CH0;
  
  // Configure timer for 32kHz sampling
  pmc_set_writeprotect(false);
  pmc_enable_periph_clk(ID_TC0);
  TC_Configure(TC0, 0, TC_CMR_TCCLKS_TIMER_CLOCK1 | TC_CMR_WAVE | TC_CMR_WAVSEL_UP_RC);
  TC_SetRC(TC0, 0, 656);
  TC_Start(TC0, 0);
  
  TC0->TC_CHANNEL[0].TC_IER = TC_IER_CPCS;
  NVIC_EnableIRQ(TC0_IRQn);

  Serial.println("MFCC Processor Ready");
}

void processFrame() {
  static uint16_t frameCount = 0;
  float windowedFrame[FRAME_SIZE];
  float fftOutput[FRAME_SIZE * 2]; // Real FFT needs 2*N space
  
  // 1. Apply Hamming window
  for (int i = 0; i < FRAME_SIZE; i++) {
    windowedFrame[i] = audioBuffer[i] * hammingWindow[i];
  }

  // 2. Pre-emphasis
  static float prevSample = 0;
  for (int i = 0; i < FRAME_SIZE; i++) {
    float current = windowedFrame[i];
    windowedFrame[i] = current - 0.97f * prevSample;
    prevSample = current;
  }

  // 3. Compute FFT
  arm_rfft_f32(&fftInstance, windowedFrame, fftOutput);

  // 4. Power spectrum
  float powerSpectrum[FRAME_SIZE/2];
  for (int i = 0; i < FRAME_SIZE/2; i++) {
    float real = fftOutput[2*i];
    float imag = fftOutput[2*i+1];
    powerSpectrum[i] = real*real + imag*imag;
  }

  // 5. Mel filterbank
  float melBands[NUM_MEL_FILTERS] = {0};
  for (int m = 0; m < NUM_MEL_FILTERS; m++) {
    for (int k = 0; k < FRAME_SIZE/2; k++) {
      melBands[m] += powerSpectrum[k] * melFilters[m][k];
    }
    melBands[m] = logf(melBands[m] + 1e-6f);
  }

  // 6. DCT
  for (int k = 0; k < NUM_MFCC; k++) {
    mfccCoefficients[k] = 0;
    for (int m = 0; m < NUM_MEL_FILTERS; m++) {
      mfccCoefficients[k] += melBands[m] * dctMatrix[k][m];
    }
  }

  // Output MFCCs
  if (frameCount++ % 4 == 0) {
    Serial.print("MFCC:");
    for (int i = 0; i < NUM_MFCC; i++) {
      Serial.print(static_cast<int>(mfccCoefficients[i] * 100));
      if (i < NUM_MFCC-1) Serial.print(",");
    }
    Serial.println();
  }
}

int16_t applyFIRFilter() {
  float output = 0;
  int idx;
  
  for (int i = 0; i < 15; i++) {
    idx = (bufferIndex - i + BUFFER_SIZE) % BUFFER_SIZE;
    output += firCoeffs[i] * sampleBuffer[idx];
  }
  return static_cast<int16_t>(output);
}

void TC0_Handler() {
  TC_GetStatus(TC0, 0);
  
  ADC->ADC_CR = ADC_CR_START;
  while (!(ADC->ADC_ISR & ADC_ISR_DRDY));
  int16_t sample = static_cast<int16_t>(ADC->ADC_CDR[0] - 2048);
  
  sampleBuffer[bufferIndex] = sample;
  bufferIndex = (bufferIndex + 1) % BUFFER_SIZE;
  
  if (++downsampleCounter >= 4) {
    downsampleCounter = 0;
    
    int16_t filtered = applyFIRFilter();
    if (audioBufferIndex < FRAME_SIZE + FRAME_OVERLAP) {
      audioBuffer[audioBufferIndex++] = filtered;
      
      if (audioBufferIndex == FRAME_SIZE + FRAME_OVERLAP) {
        frameReady = true;
      }
    }
  }
}

void loop() {
  if (frameReady) {
    processFrame();
    frameReady = false;
    
    memmove(audioBuffer, &audioBuffer[FRAME_SIZE - FRAME_OVERLAP], FRAME_OVERLAP * sizeof(int16_t));
    audioBufferIndex = FRAME_OVERLAP;
  }
}
