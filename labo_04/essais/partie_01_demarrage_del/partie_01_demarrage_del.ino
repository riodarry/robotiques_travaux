#include <MeAuriga.h>

const int nombreDel = 12;
const int brocheDel = 44;
const unsigned long dureeDemarrage = 3000;

MeRGBLed led(PORT0, nombreDel);
MeEncoderOnBoard encoderLeft(SLOT2);
MeEncoderOnBoard encoderRight(SLOT1);

enum Etat {BOOT, ATTENTE};
Etat appState = BOOT;
bool firstTime = true;
unsigned long currentTime = 0;
unsigned long debutEtat = 0;

void setup() {
  Serial.begin(115200);
  encoderLeft.setMotorPwm(0);
  encoderRight.setMotorPwm(0);
  led.setpin(brocheDel);
}

void stateManager(unsigned long temps) {
  switch (appState) {
    case BOOT: {
      if (firstTime) {
        firstTime = false;
        debutEtat = temps;
        Serial.println("Etat : BOOT");
        led.setColor(0, 0, 10);
        led.show();
      }
      bool transition = temps - debutEtat >= dureeDemarrage;
      if (transition) {
        appState = ATTENTE;
        firstTime = true;
      }
      break;
    }
    case ATTENTE:
      if (firstTime) {
        firstTime = false;
        Serial.println("Etat : ATTENTE");
        led.setColor(0, 10, 0);
        led.show();
      }
      break;
  }
}

void loop() {
  currentTime = millis();
  stateManager(currentTime);
}
