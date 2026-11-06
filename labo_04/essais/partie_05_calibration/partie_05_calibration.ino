#include <MeAuriga.h>
#include <Adafruit_seesaw.h>
#include <Wire.h>
#include <math.h>
#include <string.h>

const int nombreCapteurs = 5;
const byte adresseCapteur = 0x49;
const byte adresseGyro = 0x69;
const int vitesseCalibration = 25; // tr/min, comme le pivot de la partie 4.
const int angleCalibration = 360;
const int contrasteMinimum = 50; // Reglage d'essai, a verifier sur la piste.
const int seuilPwmArret = 120;
const unsigned long tempsMaxCalibration = 20000;
const unsigned long periodeLecture = 20;
const unsigned long periodeMessages = 250;

MeGyro gyro(0, adresseGyro);
MeEncoderOnBoard encoderLeft(SLOT2);
MeEncoderOnBoard encoderRight(SLOT1);
Adafruit_seesaw ss;

struct Capteur {
  int valeurMin;
  int valeurMax;
  int valeurLue;
};

Capteur capteurs[nombreCapteurs];
enum Etat {ATTENTE, CALIB};
Etat appState = ATTENTE;
bool firstTime = true;
bool capteurPret = false;
bool gyroPret = false;
bool estCalibre = false;
unsigned long currentTime = 0;
unsigned long debutCalibration = 0;
unsigned long derniereLecture = 0;
float anglePrecedent = 0;
float angleParcouru = 0;

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
}

void changerEtat(Etat prochain) {
  arreterMoteurs();
  if (appState != prochain) {
    appState = prochain;
    firstTime = true;
  }
}

void interrompreCalibration(const char* raison) {
  estCalibre = false;
  changerEtat(ATTENTE);
  Serial.print("Arret : ");
  Serial.println(raison);
}

void traiterCommande(const char* commande) {
  Serial.print("cmd recu : ");
  Serial.println(commande);
  if (strcmp(commande, "stop") == 0) {
    if (appState == CALIB) estCalibre = false;
    changerEtat(ATTENTE);
    Serial.println("Arret : stop");
  } else if (strcmp(commande, "cal") == 0) {
    if (!gyroPret || !capteurPret) {
      Serial.println("Capteur ou gyroscope indisponible : verifier puis Reset");
    } else if (appState == ATTENTE) {
      changerEtat(CALIB);
    }
  } else {
    Serial.println("cmd INCONNUE");
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
      } else if (invalide) {
        Serial.println("cmd INCONNUE");
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

bool lireCapteurs() {
  Wire.clearWireTimeoutFlag();
  Wire.beginTransmission(adresseCapteur);
  if (!capteurPret || Wire.endTransmission() != 0 || Wire.getWireTimeoutFlag()) return false;
  for (int i = 0; i < nombreCapteurs; i++) {
    unsigned int valeur = ss.analogRead(i);
    if (valeur > 1023 || Wire.getWireTimeoutFlag()) return false;
    capteurs[i].valeurLue = valeur;
    if (capteurs[i].valeurLue < capteurs[i].valeurMin) capteurs[i].valeurMin = capteurs[i].valeurLue;
    if (capteurs[i].valeurLue > capteurs[i].valeurMax) capteurs[i].valeurMax = capteurs[i].valeurLue;
  }
  return true;
}

bool calibrationValide() {
  for (int i = 0; i < nombreCapteurs; i++) {
    if (capteurs[i].valeurMax - capteurs[i].valeurMin < contrasteMinimum) return false;
  }
  return true;
}

void afficherBilan() {
  for (int i = 0; i < nombreCapteurs; i++) {
    Serial.print("IR"); Serial.print(i);
    Serial.print(" min:"); Serial.print(capteurs[i].valeurMin);
    Serial.print(" max:"); Serial.println(capteurs[i].valeurMax);
  }
  Serial.print("estCalibre:"); Serial.println(estCalibre);
}

void etatCalib(unsigned long temps) {
  if (firstTime) {
    firstTime = false;
    Serial.println("Etat : CALIB");
    estCalibre = false;
    debutCalibration = temps;
    derniereLecture = temps;
    anglePrecedent = gyro.getAngleZ();
    angleParcouru = 0;
    for (int i = 0; i < nombreCapteurs; i++) {
      capteurs[i].valeurMin = 1023;
      capteurs[i].valeurMax = 0;
    }
    if (!lireCapteurs()) {
      interrompreCalibration("lecture LyneTracker invalide");
      return;
    }
  }

  float angle = gyro.getAngleZ();
  if (!gyroPret || !isfinite(angle)) {
    interrompreCalibration("angle invalide");
    return;
  }
  float variation = angle - anglePrecedent;
  // Additionner les petits angles permet de mesurer un tour complet.
  if (variation > 180) variation -= 360;
  else if (variation < -180) variation += 360;
  angleParcouru += variation;
  anglePrecedent = angle;

  bool transitionTemps = temps - debutCalibration >= tempsMaxCalibration;
  bool transitionEffort = abs(encoderLeft.getCurPwm()) >= seuilPwmArret || abs(encoderRight.getCurPwm()) >= seuilPwmArret;
  bool transitionFin = fabs(angleParcouru) >= angleCalibration;
  if (transitionTemps || transitionEffort) {
    interrompreCalibration("rotation interrompue : verifier duree, roues et encodeurs");
  } else if (transitionFin) {
    changerEtat(ATTENTE);
    estCalibre = calibrationValide();
    afficherBilan();
    if (estCalibre) Serial.println("Calibration terminee");
    else Serial.println("Calibration insuffisante : les cinq capteurs doivent voir noir et blanc");
  } else {
    if (temps - derniereLecture >= periodeLecture) {
      derniereLecture = temps;
      if (!lireCapteurs()) {
        interrompreCalibration("lecture LyneTracker invalide");
        return;
      }
    }
    encoderLeft.runSpeed(vitesseCalibration);
    encoderRight.runSpeed(vitesseCalibration);
  }
}

void stateManager(unsigned long temps) {
  switch (appState) {
    case ATTENTE:
      if (firstTime) {
        firstTime = false;
        Serial.println("Etat : ATTENTE | cal ou stop");
      }
      break;
    case CALIB: etatCalib(temps); break;
  }
}

void afficherTask(unsigned long temps) {
  static unsigned long dernierMessage = 0;
  if (appState != CALIB || temps - dernierMessage < periodeMessages) return;
  dernierMessage = temps;
  Serial.print("Tour:"); Serial.print(fabs(angleParcouru), 1);
  Serial.print("\tPWM_G:"); Serial.print(encoderLeft.getCurPwm());
  Serial.print("\tPWM_D:"); Serial.print(encoderRight.getCurPwm());
  for (int i = 0; i < nombreCapteurs; i++) {
    Serial.print("\tIR"); Serial.print(i);
    Serial.print(":"); Serial.print(capteurs[i].valeurLue);
  }
  Serial.println("");
}

void setup() {
  Serial.begin(115200);
  arreterMoteurs();
  configurerEncodeurs();
  Wire.begin();
  Wire.setWireTimeout(25000, true);
  capteurPret = ss.begin(adresseCapteur);
  Wire.setWireTimeout(25000, true);
  if (Wire.getWireTimeoutFlag()) capteurPret = false;
  Wire.clearWireTimeoutFlag();
  Wire.beginTransmission(adresseGyro);
  gyroPret = Wire.endTransmission() == 0;
  if (gyroPret) gyro.begin();
  if (Wire.getWireTimeoutFlag()) gyroPret = false;
  if (!capteurPret) Serial.println("LyneTracker indisponible");
  if (!gyroPret) Serial.println("Gyroscope indisponible");
}

void loop() {
  commandeTask();
  if (gyroPret) {
    Wire.clearWireTimeoutFlag();
    Wire.beginTransmission(adresseGyro);
    if (Wire.endTransmission() == 0) gyro.update();
    else gyroPret = false;
    if (Wire.getWireTimeoutFlag()) gyroPret = false;
  }
  currentTime = millis();
  if (appState == CALIB && !gyroPret) interrompreCalibration("communication gyroscope");
  stateManager(currentTime);
  encoderLeft.loop();
  encoderRight.loop();
  afficherTask(currentTime);
}
