#include "PID_v1.h"
#include "MCC.h"

/*Interruption*/
//Pins des encodeurs des moteurs
const unsigned char pinA1 = 19;
const unsigned char pinA2 = 20;
const unsigned char pinA3 = 21;
const unsigned char pinB1 = 22;
const unsigned char pinB2 = 23;
const unsigned char pinB3 = 24;
//Compteurs de pas des moteurs
volatile long compteur1 = 0, compteur2 = 0, compteur3 = 0;
volatile long dernierCompteur1 = 0, dernierCompteur2 = 0, dernierCompteur3 = 0;

/*Asservissement*/
unsigned long maintenant = 0, dernierTemps = 0;//en ms
float deltaT = 0;//en ms

/*Moteur*/
float pwmMax = 255;
float vitesseMax = 2;//en cm/seconde

volatile const float rayonRoue = 3, rayonRobot = 19.5;//en cm
volatile const float tour = 9375;//nombre de pas par tour, déterminer sur 100 tours
volatile const float tau = 6.28319;//2*pi
volatile const float c1 = 1, c2 = 1, c3 = 1;//en rad/seconde/volt

volatile const float coeffVitesse = tau*rayonRoue/tour;

MCC m1(2, MOTOR12_64KHZ), m2(3, MOTOR34_64KHZ), m3(4, MOTOR34_64KHZ);

/*PID*/
const unsigned short echantillonnage = 10; //l'échantillonnage est l'intervalle de temps entre chaque calcul de la commande, exprimé en milliseconde
volatile float consigneX = 0, consigneY = 0, consigneTheta = 0; //les consigne de position en cm, cm, radian
volatile float x = 0, y = 0, theta = 0; //les position en cm, cm, radian
volatile float consigneVx = 0, consigneVy = 20, consigneW = 1.5; //les consigne de vitesse en cm/seconde, cm/seconde, radian/seconde
volatile float vx = 0, vy = 0, vr = 0, vtheta = 0, w = 0;//les mesure de vitesse en cm/seconde, cm/seconde, radian/seconde
volatile float commandeVx = 0, commandeVy = 0, commandeVr = 0, commandeVtheta = 0, commandeW = 0; //les commande de vitesse en cm/seconde, cm/seconde, radian/seconde

volatile int u1 = 0, u2 = 0, u3 = 0;//commande pwm
volatile float p1 = 0, p2 = 0, p3 = 0;//nombre de pas/mseconde

volatile float cosTheta, sinTheta;

//Réglage des coefficient des PID position
const float kpPos = 0;
const float kiPos = 0;
const float kdPos = 0;

//Réglage des coefficient des PID vitesse
const float kpVit = 60;
const float kiVit = 0;
const float kdVit = 0;

PID monPIDvit1(&vx, &commandeVx, &consigneVx, kpVit, kiVit, kdVit, DIRECT);
PID monPIDvit2(&vy, &commandeVy, &consigneVy, kpVit, kiVit, kdVit, DIRECT);
PID monPIDvit3(&w, &commandeW, &consigneW, kpVit, kiVit, kdVit, DIRECT);

PID monPIDpos1(&x, &consigneVx, &consigneX, kpPos, kiPos, kdPos, DIRECT);
PID monPIDpos2(&y, &consigneVy, &consigneY, kpPos, kiPos, kdPos, DIRECT);
PID monPIDpos3(&theta, &consigneW, &consigneTheta, kpPos, kiPos, kdPos, DIRECT);

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

void updateP1() {
  p1 = (compteur1 - dernierCompteur1);
  dernierCompteur1 = compteur1;
}

void updateP2() {
  p2 = (compteur2 - dernierCompteur2);
  dernierCompteur2 = compteur2;
}

void updateP3() {
  p3 = (compteur3 - dernierCompteur3);
  dernierCompteur3 = compteur3;
}

void asservissement() {
  //Calcul position
  x += vx * deltaT / 1000;
  y += vy * deltaT / 1000;
  theta += w * deltaT / 1000;
  //Calcul nombre de pas fait
  updateP1();
  updateP2();
  updateP3();
  //Calcul de vr,vtheta,w
  vr = coeffVitesse * (-p2 * 0.5774 + p3 * 0.5774) / deltaT * 1000;
  vtheta = coeffVitesse * (p1 * 0.6667 - p2 * 0.3333 - p3 * 0.3333) / deltaT * 1000;
  w = coeffVitesse / rayonRobot * (p1 * 0.3333 + p2 * 0.3333 + p3 * 0.3333) / deltaT * 1000;
  //Calcul de vx, vy
  cosTheta = cos(theta);
  sinTheta = sin(theta);
  vx = vr * cosTheta - vtheta * sinTheta;
  vy = vr * sinTheta + vtheta * cosTheta;
  //Mis à jour des commande du PID
  monPIDpos1.Compute();
  monPIDpos2.Compute();
  monPIDpos3.Compute();
  monPIDvit1.Compute();
  monPIDvit2.Compute();
  monPIDvit3.Compute();
  //Calcul de vr, vtheta
  commandeVr = commandeVx * cosTheta + commandeVy * sinTheta;
  commandeVtheta = -commandeVx * sinTheta + commandeVy * cosTheta;
  //À faire calcul u1, u2, u3
  u1 = 1 / rayonRoue * (commandeVtheta / c1 + rayonRobot * w / c1);
  u2 = 1 / rayonRoue * (-commandeVr * 0.866 / c2 - commandeVtheta / 2 / c2 + rayonRobot * commandeW / c2);
  u3 = 1 / rayonRoue * (commandeVr * 0.866 / c3 - commandeVtheta / 2 / c3 + rayonRobot * commandeW / c3);
  
  m1.bouger(u1);
  m2.bouger(u2);
  m3.bouger(u3);
}

void affichage() {
  //Affichage liaison série
  Serial.print(vx);
  Serial.print(" ");
  Serial.print(vy);
  Serial.print(" ");
  Serial.println(w);
}

void setup() {
  /*Initialisation de la liaison série*/
  Serial.begin(115200);

  //Mise en place des interruptions
  pinMode(pinB1, INPUT);
  attachInterrupt(digitalPinToInterrupt(pinA1), increment1, RISING);
  pinMode(pinB2, INPUT);
  attachInterrupt(digitalPinToInterrupt(pinA2), increment2, RISING);
  pinMode(pinB3, INPUT);
  attachInterrupt(digitalPinToInterrupt(pinA3), increment3, RISING);

  //Initialisation PID
  monPIDvit1.SetSampleTime(echantillonnage);
  monPIDvit1.SetOutputLimits(-pwmMax, pwmMax);
  monPIDvit1.SetMode(AUTOMATIC);

  monPIDvit2.SetSampleTime(echantillonnage);
  monPIDvit2.SetOutputLimits(-pwmMax, pwmMax);
  monPIDvit2.SetMode(AUTOMATIC);

  monPIDvit3.SetSampleTime(echantillonnage);
  monPIDvit3.SetOutputLimits(-pwmMax, pwmMax);
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
  dernierTemps = millis();
}

void loop() {
  
  maintenant = millis();
  deltaT = maintenant - dernierTemps;
  if(deltaT >= echantillonnage) {
    asservissement();
    dernierTemps = maintenant;
    affichage();
  }
  
  /*
  m1.bouger(255*c1);
  delay(10000);
  m1.bouger(0);
  Serial.print("m1:");
  Serial.println(compteur1/tour);
  m2.bouger(200*c2);
  delay(10000);
  m2.bouger(0);
  Serial.print("m2:");
  Serial.println(compteur2/tour);
  m3.bouger(200*c3);
  delay(10000);
  m3.bouger(0);
  Serial.print("m3:");
  Serial.println(compteur3/tour);
  */
}
