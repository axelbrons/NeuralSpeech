#include <Arduino.h>

#define FILTER_ORDER 31
#define BUFFER_SIZE 8000  // 1 seconde à 8kHz
#define DOWNSAMPLE_RATIO 4

volatile uint16_t sample = 0;
volatile bool recording = false;
volatile bool dataReady = false;

const int micPin = A0;
const int buttonPin = 2;

// Coefficients FIR passe-bas (générés pour Fc = 4kHz, Fs = 32kHz)
const float firCoeffs[FILTER_ORDER] = {
  -0.0010, -0.0021, -0.0028, -0.0019, 0.0010, 0.0057, 0.0111, 0.0157,
  0.0178, 0.0163, 0.0104, 0.0006, -0.0121, -0.0252, -0.0361, -0.0420,
  -0.0403, -0.0296, -0.0108, 0.0141, 0.0410, 0.0648, 0.0814, 0.0873,
  0.0814, 0.0648, 0.0410, 0.0141, -0.0108, -0.0296, -0.0403
};

volatile float firBuffer[FILTER_ORDER] = {0};
volatile int firIndex = 0;
volatile int downsampleCount = 0;
volatile int bufferIndex = 0;

volatile uint16_t recordedData[BUFFER_SIZE]; // stocke 1 seconde d’audio à 8kHz

void setup() {
  analogReadResolution(12);
  Serial.begin(115200);
  analogWriteResolution(12);

  pinMode(buttonPin, INPUT); // bouton avec pull-down externe

  pmc_set_writeprotect(false);
  pmc_enable_periph_clk(ID_TC0);

  TC_Configure(TC0, 0,
               TC_CMR_TCCLKS_TIMER_CLOCK1 |
               TC_CMR_WAVE |
               TC_CMR_WAVSEL_UP_RC);

  TC_SetRC(TC0, 0, 656); // 32kHz
  TC_Start(TC0, 0);

  TC0->TC_CHANNEL[0].TC_IER = TC_IER_CPCS;
  NVIC_EnableIRQ(TC0_IRQn);
}

void loop() {
  // Appui sur le bouton pour démarrer l'enregistrement
  if (digitalRead(buttonPin) == HIGH && !recording && !dataReady) {
    bufferIndex = 0;
    firIndex = 0;
    downsampleCount = 0;
    recording = true;
    Serial.println("Enregistrement démarré !");
    delay(300); // anti-rebond
  }

  // Quand 1s d'enregistrement est terminée → envoi série
  if (dataReady) {
    Serial.println("Envoi des données filtrées :");

    for (int i = 0; i < BUFFER_SIZE; i++) {
      Serial.println(recordedData[i]);  // Envoi des échantillons
    }

    Serial.println("Transmission terminée !");
    dataReady = false;
  }
}

void TC0_Handler() {
  TC_GetStatus(TC0, 0);

  // Lire micro
  float newSample = analogRead(micPin);

  // Mise à jour buffer circulaire FIR
  firBuffer[firIndex] = newSample;
  firIndex = (firIndex + 1) % FILTER_ORDER;

  // Appliquer le filtre FIR
  float filtered = 0;
  int idx = firIndex;
  for (int i = 0; i < FILTER_ORDER; i++) {
    idx = (idx - 1 + FILTER_ORDER) % FILTER_ORDER;
    filtered += firBuffer[idx] * firCoeffs[i];
  }

  // Down-sampling (1/4 des samples à 32kHz → 8kHz)
  downsampleCount++;
  if (downsampleCount >= DOWNSAMPLE_RATIO) {
    downsampleCount = 0;

    if (recording && bufferIndex < BUFFER_SIZE) {
      recordedData[bufferIndex++] = constrain(filtered, 0, 4095); // Clamp 12 bits

      if (bufferIndex >= BUFFER_SIZE) {
        recording = false;
        dataReady = true;
      }
    }
  }

  // Sortie DAC (facultatif, utile pour l'oscillo)
  //analogWrite(DAC0, constrain(filtered, 0, 4095));
}
