#include <MeAuriga.h>
#include <Adafruit_seesaw.h>
#include <Wire.h>
#include <math.h>
#include <string.h>

const int nombreCapteurs = 5;
const byte adresseCapteur = 0x49;
const byte adresseGyro = 0x69;
const int vitesseCalibration = 25;
const int angleCalibration = 360;
const int contrasteMinimum = 50;
const int seuilNoir = 200;
const int zoneCentre = 250;
const int seuilPwmArret = 120;
const unsigned long tempsMaxCalibration = 20000;
const unsigned long periodeLecture = 20;
const unsigned long periodeMessages = 250;
const unsigned long confirmationPerteLigne = 100;
const unsigned long tempsMaxRecherche = 2000;
const unsigned long tempsMaxDemiTour = 8000;
const unsigned long pauseDemiTour = 200;
const unsigned long tempsSansRotation = 1500;

// Reglages d'essai : vitesses en tr/min.
const int vitesseLigne = 30;
const int vitesseRecherche = 20;
const int vitesseDemiTour = 20;
const int vitesseFinDemiTour = 15;
const int angleDemiTour = 180;
const int toleranceDemiTour = 3;
const float kpLigne = 8;
const float kiLigne = 0.02;
const float kdLigne = 2;
const float limiteIntegrale = 20;
const float limiteCorrection = 15;
const unsigned long periodePid = 40;

MeGyro gyro(0, adresseGyro);
MeEncoderOnBoard encoderLeft(SLOT2);
MeEncoderOnBoard encoderRight(SLOT1);
Adafruit_seesaw ss;

struct Capteur {
  int valeurMin;
  int valeurMax;
  int valeurLue;
  int valeurNormalisee;
};

Capteur capteurs[nombreCapteurs];
enum Etat {ATTENTE, CALIB, SUIVRE_LIGNE, RECHERCHE_LIGNE, DEMI_TOUR};
Etat appState = ATTENTE;
bool firstTime = true;
bool capteurPret = false;
bool gyroPret = false;
bool estCalibre = false;
bool lecturePrete = false;
bool lignePresente = false;
bool perteLigneEnCours = false;
bool ligneVue = false;
bool anglePerteValide = false;
long positionLigne = 0;
int dernierCoteLigne = 0;
unsigned long currentTime = 0;
unsigned long debutCalibration = 0;
unsigned long derniereLecture = 0;
unsigned long derniereCorrection = 0;
unsigned long debutPerteLigne = 0;
float anglePrecedent = 0;
float angleParcouru = 0;
float integraleLigne = 0;
float erreurPrecedenteLigne = 0;
float correctionLigne = 0;
float angleDerniereLecture = 0;
float angleDepuisPerte = 0;

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
  integraleLigne = 0;
  erreurPrecedenteLigne = 0;
  correctionLigne = 0;
  perteLigneEnCours = false;
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
  lecturePrete = false;
  lignePresente = false;
  changerEtat(ATTENTE);
  Serial.print("Arret : ");
  Serial.println(raison);
}

void terminerSuivi(const char* raison) {
  changerEtat(ATTENTE);
  Serial.print("Arret : ");
  Serial.println(raison);
}

bool estMessageService(const char* commande) {
  return strcmp(commande, "+ERROR:2") == 0 || strcmp(commande, "+READY") == 0;
}

void traiterCommande(const char* commande) {
  // Ce message deja observe n'est pas une commande du robot.
  if (estMessageService(commande)) {
    static bool dejaAffiche = false;
    if (!dejaAffiche) {
      Serial.print("Info ignoree : ");
      Serial.println(commande);
    }
    dejaAffiche = true;
    return;
  }
  Serial.print("cmd recu : ");
  Serial.println(commande);
  if (strcmp(commande, "stop") == 0) {
    if (appState == CALIB) estCalibre = false;
    terminerSuivi("stop");
  } else if (strcmp(commande, "cal") == 0) {
    if (!gyroPret || !capteurPret) {
      Serial.println("Capteur ou gyroscope indisponible : verifier puis Reset");
    } else if (appState == ATTENTE) {
      changerEtat(CALIB);
    }
  } else if (strcmp(commande, "ligne") == 0) {
    if (!estCalibre) {
      Serial.println("Calibration requise : envoyer cal");
    } else if (appState == ATTENTE) {
      dernierCoteLigne = 0;
      ligneVue = false;
      anglePerteValide = false;
      angleDepuisPerte = 0;
      changerEtat(SUIVRE_LIGNE);
    }
  } else {
    changerEtat(ATTENTE);
    Serial.println("cmd INCONNUE");
  }
}

void commandeTask() {
  static char commande[32];
  static byte longueur = 0;
  static byte enteteBluetooth = 0;
  static bool invalide = false;
  static bool tropLongue = false;
  static bool expiree = false;
  static int octetInvalide = -1;
  static unsigned long dernierCaractere = 0;
  unsigned long temps = millis();

  if ((longueur > 0 || invalide || enteteBluetooth != 0) && !expiree && temps - dernierCaractere >= 1000) {
    commande[longueur] = '\0';
    if (!invalide && estMessageService(commande)) {
      traiterCommande(commande);
      longueur = 0;
      enteteBluetooth = 0;
    } else {
      changerEtat(ATTENTE);
      Serial.println("cmd INCONNUE : message incomplet");
      invalide = true;
      expiree = true;
    }
  }

  for (int i = 0; i < 32 && Serial.available() > 0; i++) {
    byte caractere = Serial.read();
    dernierCaractere = temps;

    // Retirer FF 55 seulement s'il est present au debut.
    if (longueur == 0 && !invalide) {
      if (enteteBluetooth == 0 && caractere == 0xFF) {
        enteteBluetooth = 1;
        continue;
      }
      if (enteteBluetooth == 1) {
        enteteBluetooth = 2;
        if (caractere == 0x55) continue;
        invalide = true;
        octetInvalide = 0xFF;
      }
    }

    if (caractere == '\n' || caractere == '\r') {
      commande[longueur] = '\0';
      byte debut = 0;
      while (debut < longueur && (commande[debut] == ' ' || commande[debut] == '\t')) debut++;
      while (longueur > debut && (commande[longueur - 1] == ' ' || commande[longueur - 1] == '\t')) commande[--longueur] = '\0';

      if (invalide && !expiree) {
        changerEtat(ATTENTE);
        if (tropLongue) {
          Serial.println("cmd INCONNUE : message trop long");
        } else {
          Serial.print("cmd INCONNUE : octet invalide ");
          Serial.println(octetInvalide);
        }
      } else if (!invalide && longueur > debut) {
        traiterCommande(commande + debut);
      }
      longueur = 0;
      enteteBluetooth = 0;
      invalide = false;
      tropLongue = false;
      expiree = false;
      octetInvalide = -1;
    } else if (!invalide) {
      if ((caractere >= 32 && caractere <= 126) || caractere == '\t') {
        if (longueur < sizeof(commande) - 1) {
          commande[longueur++] = caractere;
        } else {
          invalide = true;
          tropLongue = true;
        }
      } else {
        invalide = true;
        octetInvalide = caractere;
      }
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
    // Les min/max restent fixes apres la calibration.
    if (appState == CALIB) {
      if (capteurs[i].valeurLue < capteurs[i].valeurMin) capteurs[i].valeurMin = capteurs[i].valeurLue;
      if (capteurs[i].valeurLue > capteurs[i].valeurMax) capteurs[i].valeurMax = capteurs[i].valeurLue;
    }
  }
  return true;
}

bool calibrationValide() {
  for (int i = 0; i < nombreCapteurs; i++) {
    if (capteurs[i].valeurMax - capteurs[i].valeurMin < contrasteMinimum) return false;
  }
  return true;
}

bool calculerPosition() {
  long numerateur = 0;
  long denominateur = 0;
  lignePresente = false;
  positionLigne = 0;
  for (int i = 0; i < nombreCapteurs; i++) {
    int plage = capteurs[i].valeurMax - capteurs[i].valeurMin;
    if (plage < contrasteMinimum) return false;
    long noir = (capteurs[i].valeurMax - capteurs[i].valeurLue) * 1000L / plage;
    noir = constrain(noir, 0L, 1000L);
    capteurs[i].valeurNormalisee = noir;
    if (noir >= seuilNoir) lignePresente = true;
    numerateur += noir * (i - 2);
    denominateur += noir;
  }
  if (lignePresente && denominateur > 0) positionLigne = numerateur * 1000L / denominateur;
  return true;
}

void positionTask(unsigned long temps) {
  if (!estCalibre || temps - derniereLecture < periodeLecture) return;
  derniereLecture = temps;
  lecturePrete = false;
  if (!lireCapteurs()) {
    interrompreCalibration("lecture LyneTracker invalide");
    return;
  }
  if (!calculerPosition()) {
    interrompreCalibration("calibration insuffisante");
    return;
  }
  lecturePrete = true;
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
    lecturePrete = false;
    lignePresente = false;
    debutCalibration = temps;
    derniereLecture = temps;
    anglePrecedent = gyro.getAngleZ();
    angleParcouru = 0;
    for (int i = 0; i < nombreCapteurs; i++) {
      capteurs[i].valeurMin = 1023;
      capteurs[i].valeurMax = 0;
      capteurs[i].valeurNormalisee = 0;
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
    if (estCalibre) Serial.println("Calibration terminee : moteurs arretes, envoyer ligne");
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

bool suivreAnglePerte() {
  float angle = gyro.getAngleZ();
  if (!anglePerteValide || !gyroPret || !isfinite(angle)) {
    anglePerteValide = false;
    return false;
  }
  float variation = angle - angleDerniereLecture;
  if (variation > 180) variation -= 360;
  else if (variation < -180) variation += 360;
  if (!isfinite(variation) || fabs(variation) > 90) {
    anglePerteValide = false;
    return false;
  }
  angleDepuisPerte += variation;
  angleDerniereLecture = angle;
  return true;
}

void etatSuivreLigne(unsigned long temps) {
  if (firstTime) {
    firstTime = false;
    lecturePrete = false;
    perteLigneEnCours = false;
    derniereLecture = temps - periodeLecture;
    derniereCorrection = temps - periodePid;
    Serial.println("Etat : SUIVRE_LIGNE");
    return;
  }
  if (!estCalibre) {
    terminerSuivi("calibration requise");
    return;
  }

  positionTask(temps);
  if (appState != SUIVRE_LIGNE || !lecturePrete) return;
  bool transitionPerte = !lignePresente;
  bool transitionEffort = abs(encoderLeft.getCurPwm()) >= seuilPwmArret || abs(encoderRight.getCurPwm()) >= seuilPwmArret;
  if (transitionEffort) {
    terminerSuivi("effort eleve : verifier les roues et les encodeurs");
    return;
  }
  if (transitionPerte) {
    if (!perteLigneEnCours) {
      perteLigneEnCours = true;
      debutPerteLigne = temps;
      angleDerniereLecture = gyro.getAngleZ();
      angleDepuisPerte = 0;
      anglePerteValide = gyroPret && isfinite(angleDerniereLecture);
      Serial.println("Ligne absente : verification");
    }
    if (anglePerteValide) suivreAnglePerte();
    // Garder la derniere commande pendant cette courte confirmation.
    if (temps - debutPerteLigne >= confirmationPerteLigne) {
      if (dernierCoteLigne != 0) {
        changerEtat(RECHERCHE_LIGNE);
      } else if (ligneVue) {
        changerEtat(DEMI_TOUR);
      } else {
        terminerSuivi("ligne perdue : replacer le robot puis envoyer ligne");
      }
    }
    return;
  }
  if (perteLigneEnCours) Serial.println("Ligne retrouvee");
  perteLigneEnCours = false;
  ligneVue = true;
  if (positionLigne < -zoneCentre) dernierCoteLigne = -1;
  else if (positionLigne > zoneCentre) dernierCoteLigne = 1;
  if (temps - derniereCorrection < periodePid) return;
  derniereCorrection = temps;

  float erreur = positionLigne / 1000.0;
  integraleLigne = constrain(integraleLigne + erreur, -limiteIntegrale, limiteIntegrale);
  float variation = erreur - erreurPrecedenteLigne;
  correctionLigne = kpLigne * erreur + kiLigne * integraleLigne + kdLigne * variation;
  correctionLigne = constrain(correctionLigne, -limiteCorrection, limiteCorrection);
  erreurPrecedenteLigne = erreur;

  // Tourner en ralentissant la roue interieure, sans accelerer l'autre.
  float vitesseGauche = vitesseLigne;
  float vitesseDroite = vitesseLigne;
  if (correctionLigne > 0) {
    vitesseDroite -= 2 * correctionLigne;
  } else {
    vitesseGauche += 2 * correctionLigne;
  }
  vitesseGauche = constrain(vitesseGauche, 0.0f, (float)vitesseLigne);
  vitesseDroite = constrain(vitesseDroite, 0.0f, (float)vitesseLigne);
  encoderLeft.runSpeed(vitesseGauche);
  encoderRight.runSpeed(-vitesseDroite);
}

void etatRechercherLigne(unsigned long temps) {
  static unsigned long debutRecherche = 0;
  if (firstTime) {
    firstTime = false;
    debutRecherche = temps;
    derniereLecture = temps - periodeLecture;
    Serial.print("Etat : RECHERCHE_LIGNE ");
    if (dernierCoteLigne < 0) Serial.println("gauche");
    else Serial.println("droite");
  }
  if (!estCalibre) {
    terminerSuivi("calibration requise");
    return;
  }

  positionTask(temps);
  if (appState != RECHERCHE_LIGNE || !lecturePrete) return;
  if (anglePerteValide) suivreAnglePerte();
  bool transitionEffort = abs(encoderLeft.getCurPwm()) >= seuilPwmArret || abs(encoderRight.getCurPwm()) >= seuilPwmArret;
  bool transitionTemps = temps - debutRecherche >= tempsMaxRecherche;
  if (transitionEffort) {
    terminerSuivi("effort eleve : verifier les roues et les encodeurs");
    return;
  }
  if (dernierCoteLigne == 0) {
    terminerSuivi("ligne non retrouvee : verifier la bande puis envoyer ligne");
    return;
  }
  if (lignePresente) {
    Serial.println("Ligne retrouvee : reprise du suivi");
    changerEtat(SUIVRE_LIGNE);
    return;
  }
  if (transitionTemps || (anglePerteValide && fabs(angleDepuisPerte) >= angleDemiTour - toleranceDemiTour)) {
    if (ligneVue) changerEtat(DEMI_TOUR);
    else terminerSuivi("ligne non retrouvee : verifier la bande puis envoyer ligne");
    return;
  }

  // Pivoter doucement du dernier cote ou la ligne etait visible.
  int vitessePivot = dernierCoteLigne * vitesseRecherche;
  encoderLeft.runSpeed(vitessePivot);
  encoderRight.runSpeed(vitessePivot);
}

void etatDemiTour(unsigned long temps) {
  static unsigned long debutDemiTour = 0;
  static unsigned long debutPause = 0;
  static unsigned long derniereRotation = 0;
  static float angleControle = 0;
  static int sensPivot = 1;
  static bool enPause = false;
  if (firstTime) {
    firstTime = false;
    debutDemiTour = temps;
    derniereRotation = temps;
    angleControle = angleDepuisPerte;
    enPause = false;
    sensPivot = 1;
    if (dernierCoteLigne < 0) sensPivot = -1;
    derniereLecture = temps - periodeLecture;
    Serial.println("Etat : DEMI_TOUR");
  }
  if (!estCalibre || !ligneVue) {
    terminerSuivi("ligne et calibration requises");
    return;
  }
  if (!suivreAnglePerte()) {
    terminerSuivi("gyroscope indisponible pendant le demi-tour");
    return;
  }
  positionTask(temps);
  if (appState != DEMI_TOUR || !lecturePrete) return;
  bool transitionEffort = abs(encoderLeft.getCurPwm()) >= seuilPwmArret || abs(encoderRight.getCurPwm()) >= seuilPwmArret;
  bool transitionTemps = temps - debutDemiTour >= tempsMaxDemiTour;
  if (transitionEffort || transitionTemps) {
    terminerSuivi("demi-tour interrompu : verifier roues, gyroscope et duree");
    return;
  }

  // L'angle comprend deja le pivot de recherche de la ligne.
  float erreurAngle = angleDemiTour - fabs(angleDepuisPerte);
  if (fabs(angleDepuisPerte - angleControle) >= 1) {
    derniereRotation = temps;
    angleControle = angleDepuisPerte;
  }
  if (fabs(erreurAngle) <= toleranceDemiTour) {
    arreterMoteurs();
    if (!enPause) {
      enPause = true;
      debutPause = temps;
    }
    if (temps - debutPause >= pauseDemiTour) {
      Serial.print("Demi-tour termine : "); Serial.println(fabs(angleDepuisPerte), 1);
      ligneVue = false;
      dernierCoteLigne = 0;
      anglePerteValide = false;
      changerEtat(SUIVRE_LIGNE);
    }
    return;
  }
  enPause = false;
  if (temps - derniereRotation >= tempsSansRotation) {
    terminerSuivi("rotation absente : verifier gyroscope et moteurs");
    return;
  }
  int vitessePivot = vitesseDemiTour;
  if (fabs(erreurAngle) <= 30) vitessePivot = vitesseFinDemiTour;
  vitessePivot *= sensPivot;
  if (erreurAngle < 0) vitessePivot = -vitessePivot;
  encoderLeft.runSpeed(vitessePivot);
  encoderRight.runSpeed(vitessePivot);
}

void stateManager(unsigned long temps) {
  switch (appState) {
    case ATTENTE:
      if (firstTime) {
        firstTime = false;
        Serial.println("Etat : ATTENTE | cal, ligne ou stop");
      }
      positionTask(temps);
      break;
    case CALIB: etatCalib(temps); break;
    case SUIVRE_LIGNE: etatSuivreLigne(temps); break;
    case RECHERCHE_LIGNE: etatRechercherLigne(temps); break;
    case DEMI_TOUR: etatDemiTour(temps); break;
  }
}

void afficherTask(unsigned long temps) {
  static unsigned long dernierMessage = 0;
  if (appState != CALIB && (!estCalibre || !lecturePrete)) return;
  if (temps - dernierMessage < periodeMessages) return;
  dernierMessage = temps;
  if (appState == CALIB) {
    Serial.print("Tour:"); Serial.print(fabs(angleParcouru), 1);
    for (int i = 0; i < nombreCapteurs; i++) {
      Serial.print("\tIR"); Serial.print(i);
      Serial.print(":"); Serial.print(capteurs[i].valeurLue);
    }
  } else {
    for (int i = 0; i < nombreCapteurs; i++) {
      Serial.print("N"); Serial.print(i);
      Serial.print(":"); Serial.print(capteurs[i].valeurNormalisee);
      Serial.print("\t");
    }
    Serial.print("Position:");
    if (!lignePresente) {
      Serial.print("NON DETECTEE");
    } else {
      Serial.print(positionLigne);
      if (positionLigne < -zoneCentre) Serial.print("\tGAUCHE");
      else if (positionLigne > zoneCentre) Serial.print("\tDROITE");
      else Serial.print("\tCENTRE");
    }
  }
  if (appState == RECHERCHE_LIGNE || appState == DEMI_TOUR) {
    Serial.print("\tAngleTour:"); Serial.print(fabs(angleDepuisPerte), 1);
  }
  Serial.print("\tCorrection:"); Serial.print(correctionLigne, 1);
  Serial.print("\tPWM_G:"); Serial.print(encoderLeft.getCurPwm());
  Serial.print("\tPWM_D:"); Serial.println(encoderRight.getCurPwm());
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
