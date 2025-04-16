#include <Arduino.h>

// --- PARAMÈTRES ---
#define SAMPLE_RATE     32000          // Fréquence d'échantillonnage de l'ADC
#define DOWNSAMPLE_RATE 8000           // Fréquence après downsampling
#define BUFFER_SIZE     32000          // Buffer circulaire pour 1 seconde à 32kHz
#define DOWNSAMPLE_FACTOR 4            // 32kHz / 4 = 8kHz
#define BUTTON_PIN      7              // Pin du bouton pour démarrer l'enregistrement

// --- VARIABLES ---
volatile bool recording = false;
volatile uint16_t downsampleCounter = 0;
volatile uint16_t bufferIndex = 0;
volatile uint16_t readIndex = 0;
volatile int16_t audioBuffer[BUFFER_SIZE];  // Buffer circulaire
volatile bool recordingDone = false;        // Flag pour signaler la fin de l'enregistrement

// Coefficients d'un filtre passe-bas FIR (5 points, Hamming)
const float firCoeffs[5] = {0.1, 0.15, 0.5, 0.15, 0.1};
volatile int16_t firBuffer[5] = {0};

// --- SETUP ---
void setup() {
  Serial.begin(115200);
  pinMode(BUTTON_PIN, INPUT_PULLUP);  // Bouton avec pull-up interne

  analogReadResolution(12);  // Résolution de l'ADC
  analogWriteResolution(12); // Résolution de l'DAC

  // Timer pour échantillonnage à 32kHz
  pmc_set_writeprotect(false);
  pmc_enable_periph_clk(ID_TC0);
  TC_Configure(TC0, 0,
               TC_CMR_TCCLKS_TIMER_CLOCK1 |
               TC_CMR_WAVE |
               TC_CMR_WAVSEL_UP_RC);
  TC_SetRC(TC0, 0, 656);  // 21 MHz / 656 ≈ 32kHz
  TC_Start(TC0, 0);
  TC0->TC_CHANNEL[0].TC_IER = TC_IER_CPCS;
  NVIC_EnableIRQ(TC0_IRQn);

  Serial.println("P");
}

// --- LOOP ---
void loop() {
  // Attendre que le bouton soit pressé pour démarrer l'enregistrement
  Serial.println("loop");
  if (!recording && digitalRead(BUTTON_PIN) == LOW) {
    delay(50);  // Anti-rebond
    if (digitalRead(BUTTON_PIN) == LOW) {
      recording = true;
      bufferIndex = 0;
      readIndex = 0;
      Serial.println("Enreg");
    }
  }

  // Si l'enregistrement est terminé, afficher le message
  if (recordingDone) {
    recordingDone = false;
    recording = false;
    Serial.println("termin");
    
    // Afficher les premiers échantillons pour vérifier le signal
    for (int i = 0; i < BUFFER_SIZE / 4; i++) {
      Serial.println(audioBuffer[i]);  // Affichage de chaque échantillon sous-échantillonné
    }
  }
}

// --- INTERRUPTION : 32kHz ---
void TC0_Handler() {
  TC_GetStatus(TC0, 0);  // Effacer l'interruption

  int16_t raw = analogRead(A0);  // Lire la valeur brute du microphone
  raw -= 2048;                   // Centrage autour de 0

  // --- FILTRAGE FIR ---
  for (int i = 4; i > 0; i--) {
    firBuffer[i] = firBuffer[i - 1];  // Décalage des valeurs dans le buffer FIR
  }
  firBuffer[0] = raw;

  // Calcul du signal filtré
  float filtered = 0;
  for (int i = 0; i < 5; i++) {
    filtered += firBuffer[i] * firCoeffs[i];
  }

  // --- DOWNSAMPLING : 1/4 (32kHz -> 8kHz) ---
  downsampleCounter++;
  if (downsampleCounter >= DOWNSAMPLE_FACTOR) {
    downsampleCounter = 0;

    // Enregistrement des échantillons dans le buffer circulaire
    if (recording) {
      audioBuffer[bufferIndex] = (int16_t)filtered;
      bufferIndex = (bufferIndex + 1) % BUFFER_SIZE;  // Si le buffer est plein, on écrase les anciennes données
    }
  }

  // Condition de fin d'enregistrement
  if (bufferIndex == readIndex) {
    recordingDone = true;
  }
}
