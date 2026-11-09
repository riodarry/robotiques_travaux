#include <MeAuriga.h>
#include <string.h>

const byte intensiteLed = 10;
const unsigned long periodeMessages = 1000;

MeRGBLed led(PORT0, 12);
MeEncoderOnBoard encoderLeft(SLOT2);
MeEncoderOnBoard encoderRight(SLOT1);

enum Etat {ATTENTE, RECU_CAL, RECU_LIGNE, RECU_LAB3, CMD_INVALIDE};
Etat appState = ATTENTE;
bool firstTime = true;
unsigned long currentTime = 0;

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

bool estMessageService(const char* commande) {
  return strcmp(commande, "+ERROR:2") == 0 || strcmp(commande, "+READY") == 0;
}

void traiterCommande(const char* commande) {
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
  if (strcmp(commande, "stop") == 0) changerEtat(ATTENTE);
  else if (strcmp(commande, "cal") == 0) changerEtat(RECU_CAL);
  else if (strcmp(commande, "ligne") == 0) changerEtat(RECU_LIGNE);
  else if (strcmp(commande, "lab3") == 0) changerEtat(RECU_LAB3);
  else {
    changerEtat(CMD_INVALIDE);
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
      changerEtat(CMD_INVALIDE);
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
        changerEtat(CMD_INVALIDE);
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

void etatReception(const char* nomEtat, byte rouge, byte vert, byte bleu) {
  if (!firstTime) return;
  firstTime = false;
  led.setColor(0, 0, 0);
  led.setColor(1, rouge, vert, bleu);
  led.show();
  Serial.print("Etat : ");
  Serial.println(nomEtat);
}

void stateManager() {
  switch (appState) {
    case ATTENTE: etatReception("ATTENTE", 0, 0, 0); break;
    case RECU_CAL: etatReception("RECU_CAL", 0, intensiteLed, intensiteLed); break;
    case RECU_LIGNE: etatReception("RECU_LIGNE", 0, intensiteLed, 0); break;
    case RECU_LAB3: etatReception("RECU_LAB3", 0, 0, intensiteLed); break;
    case CMD_INVALIDE: etatReception("CMD_INVALIDE", intensiteLed, 0, 0); break;
  }
}

void messageTask(unsigned long temps) {
  static unsigned long dernierMessage = 0;
  if (temps - dernierMessage < periodeMessages) return;
  dernierMessage = temps;
  Serial.println("BT9 : moteurs arretes");
}

void setup() {
  Serial.begin(115200);
  arreterMoteurs();
  led.setpin(44);
  led.setColor(0, 0, 0);
  led.show();
  Serial.println("Essai Bluetooth : moteurs arretes");
}

void loop() {
  arreterMoteurs();
  currentTime = millis();
  commandeTask();
  stateManager();
  messageTask(currentTime);
}
