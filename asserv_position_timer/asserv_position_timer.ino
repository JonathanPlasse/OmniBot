#include <UsSensor.h>
#include "TimerFive.h"
#include "PID_v1.h"
#include "MCC.h"

#define N 5

/*Interruption*/
//Pins des encodeurs des moteurs
const unsigned char pinA1 = 19;
const unsigned char pinA2 = 20;
const unsigned char pinA3 = 21;
const unsigned char pinB1 = 22;
const unsigned char pinB2 = 23;
const unsigned char pinB3 = 24;
//Compteurs de pas des moteurs
volatile long compteur1 = 0, compteur2 = 0, compteur3 = 0, tmpCompteur1 = 0, tmpCompteur2 = 0, tmpCompteur3 = 0;
volatile long dernierCompteur1 = 0, dernierCompteur2 = 0, dernierCompteur3 = 0;

/*Capteur Ultrason*/
/*
  UsSensor us1(52, 53), us2(50, 51), us3(48, 50); //us.distance() renvoie la distance en cm
  short distanceSecu = 15;
*/

/*Moteur*/
volatile float vitesse1 = 0, vitesse2 = 0, vitesse3 = 0;//en tours/seconde
volatile float dernieresVitesse1[N] = {0}, dernieresVitesse2[N] = {0}, dernieresVitesse3[N] = {0};
volatile float angle1 = 0, angle2 = 0, angle3 = 0;//en rad/seconde
volatile const float tour = 2343.75;//408.25 pas par tour
volatile const float tau = 6.2431;

float tensionMesure = 12;

unsigned char tensionMax = 12 / tensionMesure * 255;
float vitesseMax = 1;
MCC m1(2, MOTOR12_64KHZ), m2(3, MOTOR34_64KHZ), m3(4, MOTOR34_64KHZ);

float vx = 0, vy = 3;

/*PID*/
const unsigned short echantillonnage = 10; //l'échantillonnage est l'intervalle de temps entre chaque calcul de la commande, exprimé en milliseconde
const float coeffVitesse = 1000 / tour / echantillonnage;
volatile float consignePos1 = 0, consignePos2 = 0, consignePos3 = 0; //en radian
volatile float consigneVit1 = 3, consigneVit2 = 3/*-0.866 * vx - 0.5 * vy*/, consigneVit3 = 3/*0.866 * vx - 0.5 * vy*/; //la consigne donne la vitesse voulue du moteur en tours/seconde
volatile float commande1 = 0, commande2 = 0, commande3 = 0; //la commande est le pwm envoyé sur le moteur

//Réglage des coefficient des PID position
const float kpPos = 0;
const float kiPos = 0;
const float kdPos = 0;

//Réglage des coefficient des PID vitesse
const float kpVit = 100;
const float kiVit = 100;
const float kdVit = 0;

PID monPIDvit1(&vitesse1, &commande1, &consigneVit1, kpVit, kiVit, kdVit, DIRECT);
PID monPIDvit2(&vitesse2, &commande2, &consigneVit2, kpVit, kiVit, kdVit, DIRECT);
PID monPIDvit3(&vitesse3, &commande3, &consigneVit3, kpVit, kiVit, kdVit, DIRECT);

PID monPIDpos1(&angle1, &consigneVit1, &consignePos1, kpPos, kiPos, kdPos, DIRECT);
PID monPIDpos2(&angle2, &consigneVit2, &consignePos2, kpPos, kiPos, kdPos, DIRECT);
PID monPIDpos3(&angle3, &consigneVit3, &consignePos3, kpPos, kiPos, kdPos, DIRECT);

void increment1() {
  if (PINA & (1 << 0))//(digitalRead(pinB1) == HIGH)
    compteur1++;
  else
    compteur1--;
}

void increment2() {
  if (PINA & (1 << 1))//(digitalRead(pinB2) == HIGH)
    compteur2++;
  else
    compteur2--;
}

void increment3() {
  if (PINA & (1 << 2))//(digitalRead(pinB3) == HIGH)
    compteur3++;
  else
    compteur3--;
}

void getVitesse1() {
  vitesse1 = (compteur1 - dernierCompteur1) * coeffVitesse;
  dernierCompteur1 = compteur1;
}

void getVitesse2() {
  vitesse2 = (compteur2 - dernierCompteur2) * coeffVitesse;
  dernierCompteur2 = compteur2;
}

void getVitesse3() {
  vitesse3 = (compteur3 - dernierCompteur3) * coeffVitesse;
  dernierCompteur3 = compteur3;
}

void asservissement() {
  //Calcul et application de la commande
  angle1 = compteur1 / tour * tau;
  angle2 = compteur2 / tour * tau;
  angle3 = compteur3 / tour * tau;
  getVitesse1();
  getVitesse2();
  getVitesse3();
  monPIDpos1.Compute();
  monPIDpos2.Compute();
  monPIDpos3.Compute();
  monPIDvit1.Compute();
  monPIDvit2.Compute();
  monPIDvit3.Compute();
  m1.bouger((int)commande1);
  m2.bouger((int)commande2);
  m3.bouger((int)commande3);
}

//Retourne true si arret du robot a cause d'un obstacle sur le chemin
bool arretObstacle()
{
  //À compléter
  return false;
}

void affichage() {
  //Affichage liaison série
  Serial.print(vitesse1);
  Serial.print(' ');
  Serial.print(consigneVit1);
  Serial.print(' ');
  Serial.println(5);
}

void setup() {
  /*Initialisation de la liaison série*/
  Serial.begin(9600);

  //Mise en place des interruptions
  pinMode(pinB1, INPUT);
  attachInterrupt(digitalPinToInterrupt(pinA1), increment1, RISING);
  pinMode(pinB2, INPUT);
  attachInterrupt(digitalPinToInterrupt(pinA2), increment2, RISING);
  pinMode(pinB3, INPUT);
  attachInterrupt(digitalPinToInterrupt(pinA3), increment3, RISING);

  //Initialisation PID
  monPIDvit1.SetSampleTime(echantillonnage);
  monPIDvit1.SetOutputLimits(-tensionMax, tensionMax);
  monPIDvit1.SetMode(AUTOMATIC);

  monPIDvit2.SetSampleTime(echantillonnage);
  monPIDvit2.SetOutputLimits(-tensionMax, tensionMax);
  monPIDvit2.SetMode(AUTOMATIC);

  monPIDvit3.SetSampleTime(echantillonnage);
  monPIDvit3.SetOutputLimits(-tensionMax, tensionMax);
  monPIDvit3.SetMode(AUTOMATIC);

  monPIDpos1.SetSampleTime(echantillonnage);
  monPIDpos1.SetOutputLimits(-vitesseMax, vitesseMax);
  //monPIDpos1.SetMode(AUTOMATIC);

  monPIDpos2.SetSampleTime(echantillonnage);
  monPIDpos2.SetOutputLimits(-vitesseMax, vitesseMax);
  //monPIDpos2.SetMode(AUTOMATIC);

  monPIDpos3.SetSampleTime(echantillonnage);
  monPIDpos3.SetOutputLimits(-vitesseMax, vitesseMax);
  //monPIDpos3.SetMode(AUTOMATIC);

  //Exécute la fonction asservissement toutes les "echantillonnage" ms
  Timer5.attachInterrupt(asservissement, echantillonnage * 1000);
}

void loop() {
  //Arret si obstacle
  //arretObstacle();
  affichage();
  /*
    if (tmpCompteur1 != compteur1 || tmpCompteur2 != compteur2 || tmpCompteur3 != compteur3)
    {
    affichage();
    tmpCompteur1 = compteur1;
    tmpCompteur2 = compteur2;
    tmpCompteur3 = compteur3;
    }
  */
}
