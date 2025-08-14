# Résumé technique

## 1. Traitement du signal
- **Échantillonnage** : 32 kHz (théorème de Nyquist-Shannon)
- **Filtre RIF** : Atténuation des fréquences > 4 kHz
- **MFCC** : 48 frames × 13 coefficients

## 2. Réseau de neurones
- **Architecture** : {624, 4} (précision : 98.46%)
- **Entraînement** : 750 enregistrements, 75 enregistrements de test

## 3. Tests
- Validation du filtre RIF via oscilloscope
- Visualisation des MFCC avec Python
- Test réseau de neurones sur données tests
- Test de plusieurs architectures de réseau de neurones en ajoutant des couche de convolutions
