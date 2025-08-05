#include <Arduino.h>
#include "arduinoMFCC.h"

#define LED_PIN 2
#define BUFFER_SIZE 33
#define RECORD_DURATION_MS 1000
#define SAMPLE_RATE 8000
#define SAMPLE_INTERVAL_DIV 4
#define NUM_SAMPLES (SAMPLE_RATE * RECORD_DURATION_MS / 1000)
#define FRAME_SIZE 256
#define NUM_FRAMES 48
#define HOP_SIZE 164

#define MFCC_SIZE 13
#define DCT_MFCC_SIZE 6
#define NUM_RECORDINGS 55
#define PAUSE_BETWEEN_RECORDINGS_MS 500

volatile bool isRecording = false;
volatile uint16_t recordIndex = 0;
volatile int16_t audioBuffer[NUM_SAMPLES];
float frames[NUM_FRAMES][FRAME_SIZE];
float all_mfccs[NUM_FRAMES][MFCC_SIZE];

volatile float firBuffer[BUFFER_SIZE];
volatile int bufferIndex = 0;
volatile int downsampleCounter = 0;

const float firCoeffs[BUFFER_SIZE] = {
  -0.0012, -0.0023, -0.0036, -0.0047, -0.0051, -0.0039, 0.0000, 0.0073,
   0.0176, 0.0301, 0.0434, 0.0553, 0.0635, 0.0669, 0.0650, 0.0584,
   0.0485, 0.0368, 0.0248, 0.0136, 0.0041, -0.0027, -0.0065, -0.0074,
  -0.0062, -0.0039, -0.0016, -0.0003, 0.0002, 0.0001, 0.0000, 0.0000, 0.0000
};

arduinoMFCC mymfcc(MFCC_SIZE, DCT_MFCC_SIZE, FRAME_SIZE, SAMPLE_RATE);
float mfcc[MFCC_SIZE];

void setup() {
  Serial.begin(115200);
  pinMode(LED_PIN, OUTPUT);
  analogReadResolution(12);

  //mymfcc.pre_emphasis();
  mymfcc.create_hamming_window();
  //mymfcc.apply_fft();
  mymfcc.create_mel_filter_bank();
  //mymfcc.create_dct_matrix();

  pmc_set_writeprotect(false);
  pmc_enable_periph_clk(ID_TC0);
  TC_Configure(TC0, 0,
               TC_CMR_TCCLKS_TIMER_CLOCK1 |
               TC_CMR_WAVE |
               TC_CMR_WAVSEL_UP_RC);
  TC_SetRC(TC0, 0, 1312); // 32kHz
  TC_Start(TC0, 0);

  TC0->TC_CHANNEL[0].TC_IER = TC_IER_CPCS;
  NVIC_EnableIRQ(TC0_IRQn);

  delay(3000); // Attente initiale
  Serial.println("Go");
}

void loop() {
  for (int r = 0; r < NUM_RECORDINGS; r++) {
    // Allume la LED et démarre l'enregistrement
    digitalWrite(LED_PIN, HIGH);
    isRecording = true;
    recordIndex = 0;

    // Attente que l'enregistrement se termine (1s)
    while (isRecording);

    // Éteint la LED après enregistrement
    digitalWrite(LED_PIN, LOW);

    // Calcul et affichage des MFCC
    for (int f = 0; f < NUM_FRAMES; f++) {
      int start = f * HOP_SIZE;
      for (int i = 0; i < FRAME_SIZE; i++) {
        frames[f][i] = (float)audioBuffer[start + i];
      }

      mymfcc.compute(frames[f], mfcc);

      for (int m = 0; m < MFCC_SIZE; m++) {
        all_mfccs[f][m] = mfcc[m];
      }
    }

    //Serial.print("=== Enregistrement ");
    //Serial.print(r + 1);
    //Serial.println(" ===");

    for (int f = 0; f < NUM_FRAMES; f++) {
      for (int m = 0; m < MFCC_SIZE; m++) {
        Serial.println(all_mfccs[f][m], 2);
      }
    }

    //delay(PAUSE_BETWEEN_RECORDINGS_MS); // Pause entre chaque enregistrement
  }

  while (1); // Fin du programme après 10 enregistrements
}

void TC0_Handler() {
  TC_GetStatus(TC0, 0);
  uint16_t rawSample = analogRead(A0);
  firBuffer[bufferIndex] = rawSample;
  bufferIndex = (bufferIndex + 1) % BUFFER_SIZE;

  downsampleCounter++;
  if (downsampleCounter >= SAMPLE_INTERVAL_DIV) {
    downsampleCounter = 0;

    float filtered = 0;
    int idx = bufferIndex;
    for (int i = 0; i < BUFFER_SIZE; i++) {
      idx = (idx != 0) ? idx - 1 : BUFFER_SIZE - 1;
      filtered += firBuffer[idx] * firCoeffs[i];
    }

    if (isRecording && recordIndex < NUM_SAMPLES) {
      audioBuffer[recordIndex++] = (int16_t)filtered;
      if (recordIndex >= NUM_SAMPLES) {
        isRecording = false;
      }
    }
  }
}
