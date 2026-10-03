#include <MeAuriga.h>

// Labo 03 : integration 2 du prototype existant.
// DA 2409626, derniers chiffres 26 : gauche puis droite (pivots a venir).
// Version partielle : arret apres le segment 1, sans sonar ni pivot.

#define LEDNUM 12
#define LEDPIN 44

enum AppState { SETUP_STATE, SEGMENT_1_STATE, READY_STATE, FAULT_STATE };
AppState appState = SETUP_STATE;

MeRGBLed led(PORT0, LEDNUM);
MeGyro gyro(0, 0x69);
MeEncoderOnBoard encoderRight(SLOT1);
MeEncoderOnBoard encoderLeft(SLOT2);

const int ENCODER_PULSES = 9;
const float ENCODER_RATIO = 39.267;
const float WHEEL_CIRCUMFERENCE_CM = 20.26;
const float SEGMENT_1_DISTANCE_CM = 100.0; // X a remplacer a l'evaluation.
const int STRAIGHT_PWM = 120;
const int APPROACH_PWM = 80;
const float SLOW_APPROACH_DISTANCE_CM = 15.0;
const double STRAIGHT_KP = 6.75;
const double STRAIGHT_KD = 1.0;
const double MAX_STRAIGHT_CORRECTION = 60.0;

const int PROGRESS_LED_COUNT = 7;
const int START_MARKER_LED = 8;
const int FIRST_SEGMENT_MARKER_LED = 12;
const int LED_BRIGHTNESS = 10;
const unsigned long START_DELAY_MS = 3000;
const unsigned long SERIAL_RATE_MS = 250;
const unsigned long SEGMENT_TIMEOUT_MS = 20000;
const bool DEBUG_ENABLED = true;
unsigned long currentTime = 0;
long segmentStartLeftDeg = 0;
long segmentStartRightDeg = 0;

void rightEncoderInterrupt() {
  if (digitalRead(encoderRight.getPortB()) == 0) {
    encoderRight.pulsePosMinus();
  } else {
    encoderRight.pulsePosPlus();
  }
}

void leftEncoderInterrupt() {
  if (digitalRead(encoderLeft.getPortB()) == 0) {
    encoderLeft.pulsePosMinus();
  } else {
    encoderLeft.pulsePosPlus();
  }
}

void encoderConfig() {
  attachInterrupt(encoderRight.getIntNum(), rightEncoderInterrupt, RISING);
  attachInterrupt(encoderLeft.getIntNum(), leftEncoderInterrupt, RISING);
  encoderRight.setPulse(ENCODER_PULSES);
  encoderLeft.setPulse(ENCODER_PULSES);
  encoderRight.setRatio(ENCODER_RATIO);
  encoderLeft.setRatio(ENCODER_RATIO);
  encoderRight.setPosPid(1.8, 0, 1.2);
  encoderLeft.setPosPid(1.8, 0, 1.2);
  encoderRight.setSpeedPid(0.18, 0, 0);
  encoderLeft.setSpeedPid(0.18, 0, 0);

  // Configuration PWM du professeur pour l'Auriga.
  TCCR1A = _BV(WGM10);
  TCCR1B = _BV(CS11) | _BV(WGM12);
  TCCR2A = _BV(WGM21) | _BV(WGM20);
  TCCR2B = _BV(CS21);
}

void stopMotors() {
  encoderLeft.setTarPWM(0);
  encoderRight.setTarPWM(0);
  encoderLeft.setMotorPwm(0);
  encoderRight.setMotorPwm(0);
}

void setup() {
  Serial.begin(115200);
  encoderConfig();
  stopMotors();
  gyro.begin(); // Robot immobile pendant la calibration.
  led.setpin(LEDPIN);
  led.setColor(0, 0, 0);
  led.show();
}

void loop() {
  gyro.update();
  encoderRight.loop();
  encoderLeft.loop();
  currentTime = millis();
  stateManager(currentTime);
  serialTask(currentTime);
}

// Les transitions restent dans chaque etat.
void stateManager(unsigned long cT) {
  switch (appState) {
    case SETUP_STATE:
      setupState(cT);
      break;
    case SEGMENT_1_STATE:
      segment1State(cT);
      break;
    case READY_STATE:
      readyState();
      break;
    case FAULT_STATE:
      faultState();
      break;
    default:
      stopMotors();
      break;
  }
}

void showProgress(float progress, int stageLed) {
  static int previousCount = -1;
  static int previousStageLed = -1;
  if (progress < 0.0) {
    progress = 0.0;
  } else if (progress > 1.0) {
    progress = 1.0;
  }
  int count = (int)(progress * PROGRESS_LED_COUNT);
  if (count == previousCount && stageLed == previousStageLed) {
    return;
  }
  previousCount = count;
  previousStageLed = stageLed;
  led.setColor(0, 0, 0);
  for (int i = 1; i <= count; i++) {
    led.setColor(i, 0, 0, LED_BRIGHTNESS);
  }
  led.setColor(stageLed, 0, LED_BRIGHTNESS, 0);
  led.show();
}

float getSegmentDistanceCm() {
  long leftDegrees = encoderLeft.getCurPos() - segmentStartLeftDeg;
  long rightDegrees = encoderRight.getCurPos() - segmentStartRightDeg;
  if (leftDegrees < 0) {
    leftDegrees = -leftDegrees;
  }
  if (rightDegrees < 0) {
    rightDegrees = -rightDegrees;
  }
  float averageDegrees = (leftDegrees + rightDegrees) / 2.0;
  // Conversion inverse de moveForward(), en cm.
  return averageDegrees * WHEEL_CIRCUMFERENCE_CM / 360.0;
}

void goStraight(int speed, bool firstRun) {
  static double targetAngleZ = 0.0;
  static double previousError = 0.0;
  if (firstRun) {
    targetAngleZ = gyro.getAngleZ();
    previousError = 0.0;
    encoderLeft.setTarPWM(speed);
    encoderRight.setTarPWM(-speed);
    return;
  }
  double error = gyro.getAngleZ() - targetAngleZ;
  if (error > 180.0) {
    error -= 360.0;
  } else if (error < -180.0) {
    error += 360.0;
  }
  double correction = STRAIGHT_KP * error
                    + STRAIGHT_KD * (error - previousError);
  previousError = error;
  if (correction > MAX_STRAIGHT_CORRECTION) {
    correction = MAX_STRAIGHT_CORRECTION;
  } else if (correction < -MAX_STRAIGHT_CORRECTION) {
    correction = -MAX_STRAIGHT_CORRECTION;
  }
  encoderLeft.setTarPWM(speed - correction);
  encoderRight.setTarPWM(-speed - correction);
}

void setupState(unsigned long cT) {
  static bool firstTime = true;
  static unsigned long startTime = 0;
  if (firstTime) {
    firstTime = false;
    startTime = cT;
    stopMotors();
    showProgress(0.0, START_MARKER_LED);
    if (DEBUG_ENABLED) {
      Serial.println("Entree : ATTENTE 3 SECONDES");
    }
    return;
  }
  unsigned long elapsedTime = cT - startTime;
  showProgress(elapsedTime / (float)START_DELAY_MS, START_MARKER_LED);
  bool transitionSegment1 = elapsedTime >= START_DELAY_MS;
  if (transitionSegment1) {
    stopMotors();
    firstTime = true;
    appState = SEGMENT_1_STATE;
  }
}

void segment1State(unsigned long cT) {
  static bool firstTime = true;
  static unsigned long startTime = 0;
  if (firstTime) {
    firstTime = false;
    startTime = cT;
    segmentStartLeftDeg = encoderLeft.getCurPos();
    segmentStartRightDeg = encoderRight.getCurPos();
    showProgress(0.0, FIRST_SEGMENT_MARKER_LED);
    goStraight(STRAIGHT_PWM, true);
    if (DEBUG_ENABLED) {
      Serial.println("Entree : SEGMENT 1");
    }
    return;
  }
  float travelledCm = getSegmentDistanceCm();
  showProgress(travelledCm / SEGMENT_1_DISTANCE_CM, FIRST_SEGMENT_MARKER_LED);
  bool transitionTimeout = cT - startTime >= SEGMENT_TIMEOUT_MS;
  bool transitionReady = travelledCm >= SEGMENT_1_DISTANCE_CM;
  if (transitionTimeout) {
    stopMotors();
    firstTime = true;
    appState = FAULT_STATE;
    return;
  }
  if (transitionReady) {
    stopMotors();
    firstTime = true;
    appState = READY_STATE;
    return;
  }
  int speed = STRAIGHT_PWM;
  if (SEGMENT_1_DISTANCE_CM - travelledCm <= SLOW_APPROACH_DISTANCE_CM) {
    speed = APPROACH_PWM;
  }
  goStraight(speed, false);
}

void readyState() {
  static bool firstTime = true;
  if (firstTime) {
    firstTime = false;
    stopMotors();
    showProgress(1.0, FIRST_SEGMENT_MARKER_LED);
    if (DEBUG_ENABLED) {
      Serial.println("Entree : SEGMENT 1 TERMINE - PIVOT NON INTEGRE");
    }
    return;
  }
  stopMotors();
}

void faultState() {
  static bool firstTime = true;
  if (firstTime) {
    firstTime = false;
    stopMotors();
    if (DEBUG_ENABLED) {
      Serial.println("Entree : ARRET - TEMPS MAXIMUM SEGMENT 1");
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
  Serial.print("Temps : ");
  Serial.print(cT);
  Serial.print(" ms | Etat : ");
  Serial.print((int)appState);
  Serial.print(" | Distance : ");
  float travelledCm = 0.0;
  if (appState != SETUP_STATE) {
    travelledCm = getSegmentDistanceCm();
  }
  Serial.print(travelledCm, 1);
  Serial.print(" cm | Angle Z : ");
  Serial.println(gyro.getAngleZ(), 1);
}
