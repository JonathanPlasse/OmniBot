#ifndef MCC_H
#define MCC_H
#include <AFMotor.h>
#include <inttypes.h>

class MCC{    
  public :
    MCC(uint8_t noMoteur, uint8_t freq = DC_MOTOR_PWM_RATE);
    void bouger(int vitesse);

  private :
    AF_DCMotor _moteur;
};

#endif
