#include <MeAuriga.h>
#include <Wire.h>
#include <math.h>
#include <string.h>

const int vitesseAvance = 30; // tr/min
const int vitessePivot = 25;
const int anglePivot = 30;   // degres du robot
const unsigned long dureeAvance = 200000;
const unsigned long tempsMaxPivot = 3000;
const unsigned long periodeMessages = 250;
const int ecartMaxAvance = 20;
const int seuilPwmArret = 120;
const float kpDirection = 0.8;
const float kdDirection = 0.4;
const float correctionMax = 5; // tr/min, reglages d'essai.
const unsigned long periodeCorrection = 40;

const byte adresseGyro = 0x69;
MeGyro gyro(0, adresseGyro);
MeEncoderOnBoard encoderLeft(SLOT2);
MeEncoderOnBoard encoderRight(SLOT1);

enum Etat {ATTENTE, AVANCE, PIVOT, TERMINE};
Etat appState = ATTENTE;
bool firstTime = true;
bool gyroPret = false;
unsigned long currentTime = 0;
unsigned long debutEssai = 0;
float angleDepart = 0;
unsigned long derniereCorrection = 0;
float erreurPrecedente = 0;
float correctionDirection = 0;

void interruptionGauche() {
  if (digitalRead(encoderLeft.getPortB()) == LOW) {
    encoderLeft.pulsePosMinus();
  } else {
    encoderLeft.pulsePosPlus();
  }
}

void interruptionDroite() {
  if (digitalRead(encoderRight.getPortB()) == LOW) {
    encoderRight.pulsePosMinus();
  } else {
    encoderRight.pulsePosPlus();
  }
}

void configurerEncodeurs() {
  attachInterrupt(encoderLeft.getIntNum(), interruptionGauche, RISING);
  attachInterrupt(encoderRight.getIntNum(), interruptionDroite, RISING);
  encoderLeft.setPulse(9);
  encoderRight.setPulse(9);
  encoderLeft.setRatio(39.267);
  encoderRight.setRatio(39.267);
  encoderLeft.setSpeedPid(0.18, 0, 0);
  encoderRight.setSpeedPid(0.18, 0, 0);
  TCCR1A = _BV(WGM10);
  TCCR1B = _BV(CS11) | _BV(WGM12);
  TCCR2A = _BV(WGM21) | _BV(WGM20);
  TCCR2B = _BV(CS21);
}

void arreterMoteurs() {
  encoderLeft.setTarPWM(0);
  encoderRight.setTarPWM(0);
  encoderLeft.setMotorPwm(0);
  encoderRight.setMotorPwm(0);
  correctionDirection = 0;
}

void terminerEssai(const char* raison) {
  arreterMoteurs();
  appState = TERMINE;
  Serial.print("Arret : ");
  Serial.println(raison);
}

float angleRelatif() {
  float angle = gyro.getAngleZ() - angleDepart;
  // Le gyro peut passer de +180 a -180 sans changer de sens.
  if (angle > 180) angle -= 360;
  else if (angle < -180) angle += 360;
  return angle;
}

void corrigerDirection(unsigned long temps) {
  unsigned long ecoule = temps - derniereCorrection;
  if (ecoule >= periodeCorrection) {
    float erreur = angleRelatif();
    float pas = ecoule / (float)periodeCorrection;
    float variation = (erreur - erreurPrecedente) / pas;
    correctionDirection = kpDirection * erreur + kdDirection * variation;
    correctionDirection = constrain(correctionDirection, -correctionMax, correctionMax);
    erreurPrecedente = erreur;
    derniereCorrection = temps;
  }
  // Meme moyenne de 30 tr/min, mais une petite difference entre les roues.
  encoderLeft.runSpeed(vitesseAvance - correctionDirection);
  encoderRight.runSpeed(-vitesseAvance - correctionDirection);
}

void traiterCommande(const char* commande) {
  if (strcmp(commande, "stop") == 0) {
    terminerEssai("stop");
    return;
  }

  bool demandeAvance = strcmp(commande, "avance") == 0;
  bool demandePivot = strcmp(commande, "pivot") == 0;
  if (!demandeAvance && !demandePivot) return;
  if (!gyroPret) {
    Serial.println("Gyroscope indisponible");
  } else if (appState != ATTENTE) {
    Serial.println("Un essai par Reset");
  } else if (demandeAvance) {
    appState = AVANCE;
  } else {
    appState = PIVOT;
  }
}

void commandeTask() {
  static char commande[8];
  static byte longueur = 0;
  static bool invalide = false;

  for (int i = 0; i < 32 && Serial.available() > 0; i++) {
    char caractere = Serial.read();
    if (caractere == '\n' || caractere == '\r') {
      if (longueur > 0 && !invalide) {
        commande[longueur] = '\0';
        traiterCommande(commande);
      }
      longueur = 0;
      invalide = false;
    } else if (!invalide && longueur < sizeof(commande) - 1 && caractere >= 'a' && caractere <= 'z') {
      commande[longueur++] = caractere;
    } else {
      invalide = true;
    }
  }
}

void etatMouvement(unsigned long temps) {
  if (!gyroPret || !isfinite(gyro.getAngleZ())) {
    terminerEssai("angle invalide");
    return;
  }

  bool enPivot = appState == PIVOT;
  if (firstTime) {
    firstTime = false;
    debutEssai = temps;
    angleDepart = gyro.getAngleZ();
    if (enPivot) {
      encoderLeft.runSpeed(vitessePivot);
      encoderRight.runSpeed(vitessePivot);
      Serial.println("Etat : PIVOT");
    } else {
      derniereCorrection = temps;
      erreurPrecedente = 0;
      correctionDirection = 0;
      encoderLeft.runSpeed(vitesseAvance);
      encoderRight.runSpeed(-vitesseAvance);
      Serial.println("Etat : AVANCE");
    }
  }

  float angle = fabs(angleRelatif());
  unsigned long tempsEcoule = temps - debutEssai;
  bool tropEffort = abs(encoderLeft.getCurPwm()) >= seuilPwmArret || abs(encoderRight.getCurPwm()) >= seuilPwmArret;
  if (tropEffort) {
    terminerEssai("effort eleve : verifier les roues et les encodeurs");
  } else if (enPivot && angle >= anglePivot) {
    terminerEssai("angle atteint");
  } else if (enPivot && tempsEcoule >= tempsMaxPivot) {
    terminerEssai("temps maximum du pivot");
  } else if (!enPivot && angle >= ecartMaxAvance) {
    terminerEssai("avance : deviation importante");
  } else if (!enPivot && tempsEcoule >= dureeAvance) {
    terminerEssai("fin de l'avance");
  } else if (!enPivot) {
    corrigerDirection(temps);
  }
}

void stateManager(unsigned long temps) {
  switch (appState) {
    case AVANCE:
    case PIVOT: etatMouvement(temps); break;
    case ATTENTE:
    case TERMINE: break;
  }
}

void afficherTask(unsigned long temps) {
  static unsigned long dernierMessage = 0;
  if (!gyroPret || temps - dernierMessage < periodeMessages) return;
  dernierMessage = temps;

  Serial.print("AngleZ:"); Serial.print(gyro.getAngleZ(), 1);
  Serial.print("\tdZ:"); Serial.print(angleRelatif(), 1);
  Serial.print("\tG_deg:"); Serial.print(encoderLeft.getCurPos());
  Serial.print("\tD_deg:"); Serial.print(encoderRight.getCurPos());
  Serial.print("\tPWM_G:"); Serial.print(encoderLeft.getCurPwm());
  Serial.print("\tPWM_D:"); Serial.print(encoderRight.getCurPwm());
  Serial.print("\tCorrection:"); Serial.println(correctionDirection, 1);
}

void setup() {
  Serial.begin(115200);
  arreterMoteurs();
  configurerEncodeurs();
  Wire.begin();
  Wire.setWireTimeout(25000, true);
  Wire.beginTransmission(adresseGyro);
  gyroPret = Wire.endTransmission() == 0;
  if (gyroPret) gyro.begin();
  if (Wire.getWireTimeoutFlag()) gyroPret = false;
  if (gyroPret) Serial.println("Etat : ATTENTE | avance, pivot ou stop");
  else Serial.println("Gyroscope indisponible");
}

void loop() {
  commandeTask();
  if (gyroPret) {
    gyro.update();
    if (Wire.getWireTimeoutFlag()) {
      gyroPret = false;
      terminerEssai("communication gyroscope");
    }
  }
  currentTime = millis();
  stateManager(currentTime);
  encoderLeft.loop();
  encoderRight.loop();
  afficherTask(currentTime);
}
