#include "MCC.h"
#include <Arduino.h>

MCC::MCC(uint8_t noMoteur, uint8_t freq) : _moteur(noMoteur, freq) {}

void MCC::bouger(int vitesse) {
  if (vitesse == 0)
    _moteur.run(RELEASE);
  if (vitesse > 0) {
    _moteur.setSpeed(vitesse);
    _moteur.run(FORWARD);
  }
  else {
    _moteur.setSpeed(-vitesse);
    _moteur.run(BACKWARD);
  }
}
