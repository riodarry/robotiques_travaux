#include <MeAuriga.h>

// Labo 03 : integration 1, demarrage/FSM/DEL du prototype existant.
// DA 2409626 : variation paire. Sources et tests dans README.md.
// Version partielle : moteurs arretes, parcours non integre.

#define LEDNUM 12
#define LEDPIN 44

enum AppState { SETUP_STATE, READY_STATE };
AppState appState = SETUP_STATE;

MeRGBLed led(PORT0, LEDNUM);

const int LEFT_PWM_PIN = 11;
const int RIGHT_PWM_PIN = 10;
const int PROGRESS_LED_COUNT = 7;
const int START_MARKER_LED = 8;
const int FIRST_SEGMENT_MARKER_LED = 12;
const int LED_BRIGHTNESS = 10;

const unsigned long START_DELAY_MS = 3000;
const unsigned long SERIAL_RATE_MS = 250;
const bool DEBUG_ENABLED = true;
unsigned long currentTime = 0;

void stopMotors() {
  analogWrite(LEFT_PWM_PIN, 0);
  analogWrite(RIGHT_PWM_PIN, 0);
}

void setup() {
  Serial.begin(115200);
  pinMode(LEFT_PWM_PIN, OUTPUT);
  pinMode(RIGHT_PWM_PIN, OUTPUT);
  stopMotors();

  led.setpin(LEDPIN);
  led.setColor(0, 0, 0);
  led.show();
}

void loop() {
  currentTime = millis();
  stopMotors();
  stateManager(currentTime);
  serialTask(currentTime);
}

// Les transitions restent dans chaque etat.
void stateManager(unsigned long cT) {
  switch (appState) {
    case SETUP_STATE:
      setupState(cT);
      break;
    case READY_STATE:
      readyState();
      break;
  }
}

void showStartupProgress(unsigned long elapsedTime) {
  static int previousCount = -1;
  if (elapsedTime > START_DELAY_MS) {
    elapsedTime = START_DELAY_MS;
  }

  int count = elapsedTime * PROGRESS_LED_COUNT / START_DELAY_MS;
  if (count == previousCount) {
    return;
  }
  previousCount = count;

  led.setColor(0, 0, 0);
  for (int i = 1; i <= count; i++) {
    led.setColor(i, 0, 0, LED_BRIGHTNESS);
  }
  led.setColor(START_MARKER_LED, 0, LED_BRIGHTNESS, 0);
  led.show();
}

void setupState(unsigned long cT) {
  static bool firstTime = true;
  static unsigned long startTime = 0;

  if (firstTime) {
    firstTime = false;
    startTime = cT;
    stopMotors();
    showStartupProgress(0);
    if (DEBUG_ENABLED) {
      Serial.println("Entree : ATTENTE 3 SECONDES");
    }
    return;
  }

  unsigned long elapsedTime = cT - startTime;
  showStartupProgress(elapsedTime);

  bool transitionReady = elapsedTime >= START_DELAY_MS;
  if (transitionReady) {
    // Pas de mouvement dans cette etape.
    stopMotors();
    firstTime = true;
    appState = READY_STATE;
  }
}

void readyState() {
  static bool firstTime = true;
  if (firstTime) {
    firstTime = false;
    stopMotors();
    led.setColor(0, 0, 0);
    led.setColor(FIRST_SEGMENT_MARKER_LED, 0, LED_BRIGHTNESS, 0);
    led.show();
    if (DEBUG_ENABLED) {
      Serial.println("Entree : PRET - PARCOURS NON INTEGRE");
    }
    return;
  }

  stopMotors();
}

void serialTask(unsigned long cT) {
  static unsigned long lastTime = 0;
  if (!DEBUG_ENABLED || cT - lastTime < SERIAL_RATE_MS) {
    return;
  }
  lastTime = cT;

  // Trace toutes les 250 ms.
  Serial.print("Temps : ");
  Serial.print(cT);
  Serial.print(" ms | Etat : ");
  Serial.println((int)appState);
}
