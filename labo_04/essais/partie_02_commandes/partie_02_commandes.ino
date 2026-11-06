#include <MeAuriga.h>

MeRGBLed led(PORT0, 12);
MeEncoderOnBoard encoderLeft(SLOT2);
MeEncoderOnBoard encoderRight(SLOT1);

const int longueurMax = 32;
String commande = "";
bool tropLongue = false;

void setup() {
  Serial.begin(115200);
  encoderLeft.setMotorPwm(0);
  encoderRight.setMotorPwm(0);
  led.setpin(44);
  led.setColor(0, 10, 0);
  led.show();
  Serial.println("Test commandes : ATTENTE");
}

void traiterCommande() {
  // Retirer FF 55 seulement si cet entete Bluetooth est present.
  if (commande.length() >= 2 && (byte)commande[0] == 0xFF && (byte)commande[1] == 0x55) commande.remove(0, 2);
  commande.trim();

  if (commande.length() > 0 || tropLongue) {
    Serial.print("cmd recu : ");
    Serial.println(commande);

    if (tropLongue) Serial.println("cmd INCONNUE : trop longue");
    else if (commande == "stop") {
      encoderLeft.setMotorPwm(0);
      encoderRight.setMotorPwm(0);
      Serial.println("STOP reconnu : ATTENTE");
    }
    else if (commande == "lab3") Serial.println("LAB3 reconnu : parcours non lance dans ce test");
    else if (commande == "cal") Serial.println("CAL reconnu : calibration non lancee dans ce test");
    else if (commande == "ligne") Serial.println("LIGNE reconnue : suivi non lance dans ce test");
    else Serial.println("cmd INCONNUE");
  }

  commande = "";
  tropLongue = false;
}

void loop() {
  while (Serial.available() > 0) {
    char caractere = Serial.read();
    if (caractere == '\n') {
      traiterCommande();
    } else if (caractere != '\r') {
      if (commande.length() < longueurMax) commande += caractere;
      else tropLongue = true;
    }
  }
}
