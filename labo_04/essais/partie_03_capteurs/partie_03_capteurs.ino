#include <MeAuriga.h>
#include <Adafruit_seesaw.h>
#include <Wire.h>

MeEncoderOnBoard encoderLeft(SLOT2);
MeEncoderOnBoard encoderRight(SLOT1);
Adafruit_seesaw ss;

const int nombreCapteurs = 5;
const byte adresseCapteur = 0x49;
const unsigned long periodeLecture = 250;
int valeurs[nombreCapteurs];
bool capteurPret = false;
unsigned long currentTime = 0;

void setup() {
  Serial.begin(115200);
  encoderLeft.setMotorPwm(0);
  encoderRight.setMotorPwm(0);
  Wire.begin();
  Wire.setWireTimeout(25000, true);
  capteurPret = ss.begin(adresseCapteur);
  Wire.setWireTimeout(25000, true);
  Serial.println("Test des cinq capteurs : aucun mouvement");
}

void capteursTask(unsigned long temps) {
  static unsigned long lastTime = 0;
  if (temps - lastTime < periodeLecture) return;
  lastTime = temps;

  if (!capteurPret) {
    Serial.println("LyneTracker absent : verifier I2C puis Reset");
    return;
  }
  Wire.clearWireTimeoutFlag();
  Wire.beginTransmission(adresseCapteur);
  if (Wire.endTransmission() != 0) {
    Serial.println("Connexion I2C perdue");
    return;
  }

  for (int i = 0; i < nombreCapteurs; i++) {
    unsigned int lecture = ss.analogRead(i);
    if (lecture > 1023 || Wire.getWireTimeoutFlag()) {
      Serial.println("Lecture invalide");
      return;
    }
    valeurs[i] = lecture;
  }
  for (int i = 0; i < nombreCapteurs; i++) {
    Serial.print("IR");
    Serial.print(i);
    Serial.print(":");
    Serial.print(valeurs[i]);
    Serial.print("\t");
  }
  Serial.println("");
}

void loop() {
  currentTime = millis();
  capteursTask(currentTime);
}
