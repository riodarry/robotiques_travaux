#include <MeAuriga.h>

// Labo 03 : integration 5 du prototype existant.
// DA 2409626, derniers chiffres 26 : gauche puis droite.
// Version partielle : arret apres le segment 3, sans pivot final.

#define LEDNUM 12
#define LEDPIN 44

enum AppState { SETUP_STATE, SEGMENT_1_STATE, PIVOT_1_STATE, SEGMENT_2_STATE,
                PIVOT_2_STATE, SEGMENT_3_STATE, READY_STATE, FAULT_STATE };
AppState appState = SETUP_STATE;
enum FaultReason { SEGMENT_1_TIMEOUT, PIVOT_1_TIMEOUT, SEGMENT_2_TIMEOUT,
                   SONAR_CHANGE, OBSTACLE_TOO_CLOSE, PIVOT_2_TIMEOUT,
                   SEGMENT_3_TIMEOUT };
FaultReason faultReason = SEGMENT_1_TIMEOUT;

MeRGBLed led(PORT0, LEDNUM);
MeGyro gyro(0, 0x69);
MeEncoderOnBoard encoderRight(SLOT1);
MeEncoderOnBoard encoderLeft(SLOT2);
MeUltrasonicSensor sonar(PORT_10);

const int ENCODER_PULSES = 9;
const float ENCODER_RATIO = 39.267;
const float WHEEL_CIRCUMFERENCE_CM = 20.26;
const float FULL_SPIN_CIRCUMFERENCE_CM = 47.44;
const int FIRST_PIVOT_DEG = -90;
const int SECOND_PIVOT_DEG = 90;
const int PIVOT_SPEED_RPM = 60;
const float SEGMENT_1_DISTANCE_CM = 100.0; // X a remplacer a l'evaluation.
const float SEGMENT_3_DISTANCE_CM = 100.0; // X a remplacer a l'evaluation.
const int STRAIGHT_PWM = 120;
const int APPROACH_PWM = 80;
const float SLOW_APPROACH_DISTANCE_CM = 15.0;
const double STRAIGHT_KP = 6.75;
const double STRAIGHT_KD = 1.0;
const double MAX_STRAIGHT_CORRECTION = 60.0;
const float OBSTACLE_STOP_DISTANCE_CM = 30.0;
const float STOP_TOLERANCE_CM = 2.0;
const int ARRIVAL_READINGS = 3;
const float SONAR_MIN_CM = 2.0;
const float SONAR_MAX_CM = 400.0;
const float MAX_SONAR_CHANGE_CM = 20.0; // Controle conservateur, non calibre.
const unsigned long SONAR_RATE_MS = 100;
const unsigned long SONAR_TIMEOUT_US = 25000;

const int PROGRESS_LED_COUNT = 7;
const int START_MARKER_LED = 8;
const int FIRST_SEGMENT_MARKER_LED = 12;
const int SECOND_SEGMENT_MARKER_LED = 11;
const int THIRD_SEGMENT_MARKER_LED = 10;
const int LED_BRIGHTNESS = 10;
const unsigned long START_DELAY_MS = 3000;
const unsigned long SERIAL_RATE_MS = 250;
const unsigned long SEGMENT_TIMEOUT_MS = 20000;
const unsigned long PIVOT_TIMEOUT_MS = 6000;
const unsigned long PIVOT_SETTLE_MS = 200;
const bool DEBUG_ENABLED = true;
unsigned long currentTime = 0;
long segmentStartLeftDeg = 0;
long segmentStartRightDeg = 0;
float segment1DistanceCm = 0.0;
bool pivotSettling = false;
unsigned long pivotReachedTime = 0;
float segment2DistanceCm = 0.0;
float segment3DistanceCm = 0.0;
float segment2ExpectedDistanceCm = 0.0;
float distanceCm = -1.0;
float previousSonarCm = 0.0;
unsigned long sonarPrevious = 0;
bool sonarReady = false;
bool sonarFresh = false;
bool sonarHasPrevious = false;
bool sonarChangeDetected = false;
int arrivalCount = 0;

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
  currentTime = millis();
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
    case PIVOT_1_STATE:
      pivot1State(cT);
      break;
    case SEGMENT_2_STATE:
      segment2State(cT);
      break;
    case PIVOT_2_STATE:
      pivot2State(cT);
      break;
    case SEGMENT_3_STATE:
      segment3State(cT);
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
  if (stageLed == SECOND_SEGMENT_MARKER_LED || stageLed == THIRD_SEGMENT_MARKER_LED) {
    led.setColor(FIRST_SEGMENT_MARKER_LED, 0, LED_BRIGHTNESS, 0);
  }
  if (stageLed == THIRD_SEGMENT_MARKER_LED) {
    led.setColor(SECOND_SEGMENT_MARKER_LED, 0, LED_BRIGHTNESS, 0);
  }
  led.show();
}

void sonarTask(unsigned long cT) {
  sonarFresh = false;
  if (cT - sonarPrevious < SONAR_RATE_MS) {
    return;
  }
  sonarPrevious = cT;
  sonarFresh = true;
  sonarReady = false;
  distanceCm = -1.0;
  // Impulsion du cours; signal unique du sonar MakeBlock.
  sonar.dWrite2(LOW);
  delayMicroseconds(2);
  sonar.dWrite2(HIGH);
  delayMicroseconds(10);
  sonar.dWrite2(LOW);
  pinMode(sonar.pin2(), INPUT);
  unsigned long duration = pulseIn(sonar.pin2(), HIGH, SONAR_TIMEOUT_US);
  float measuredCm = duration / 58.0; // Conversion MakeBlock.
  if (duration == 0 || measuredCm < SONAR_MIN_CM || measuredCm >= SONAR_MAX_CM) {
    return;
  }
  distanceCm = measuredCm;
  // Une mesure trop proche reste prioritaire sur le filtre.
  if (sonarHasPrevious && measuredCm >= OBSTACLE_STOP_DISTANCE_CM - STOP_TOLERANCE_CM) {
    float change = measuredCm - previousSonarCm;
    if (change > MAX_SONAR_CHANGE_CM || change < -MAX_SONAR_CHANGE_CM) {
      sonarChangeDetected = true;
      return;
    }
  }
  previousSonarCm = measuredCm;
  sonarHasPrevious = true;
  sonarReady = true;
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

// Calcul de spin() du professeur, en cm.
void spin(int goal) {
  stopMotors();
  pivotSettling = false;
  float ratioSpin = (goal * 1.0) / 360.0;
  float dist = ratioSpin * FULL_SPIN_CIRCUMFERENCE_CM;
  float nbRots = dist / WHEEL_CIRCUMFERENCE_CM;
  long angleLeft = nbRots * 360.0;
  long angleRight = angleLeft;
  encoderLeft.move(angleLeft, PIVOT_SPEED_RPM);
  encoderRight.move(angleRight, PIVOT_SPEED_RPM);
}

bool pivotReached(unsigned long cT) {
  bool motorsReached = encoderLeft.isTarPosReached()
                    && encoderRight.isTarPosReached();
  if (!motorsReached) {
    pivotSettling = false;
    return false;
  }
  if (!pivotSettling) {
    pivotSettling = true;
    pivotReachedTime = cT;
    return false;
  }
  return cT - pivotReachedTime >= PIVOT_SETTLE_MS;
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
    segment1DistanceCm = 0.0;
    showProgress(0.0, FIRST_SEGMENT_MARKER_LED);
    goStraight(STRAIGHT_PWM, true);
    if (DEBUG_ENABLED) {
      Serial.println("Entree : SEGMENT 1");
    }
    return;
  }
  float travelledCm = getSegmentDistanceCm();
  segment1DistanceCm = travelledCm;
  showProgress(travelledCm / SEGMENT_1_DISTANCE_CM, FIRST_SEGMENT_MARKER_LED);
  bool transitionTimeout = cT - startTime >= SEGMENT_TIMEOUT_MS;
  bool transitionPivot1 = travelledCm >= SEGMENT_1_DISTANCE_CM;
  if (transitionTimeout) {
    stopMotors();
    faultReason = SEGMENT_1_TIMEOUT;
    firstTime = true;
    appState = FAULT_STATE;
    return;
  }
  if (transitionPivot1) {
    stopMotors();
    firstTime = true;
    appState = PIVOT_1_STATE;
    return;
  }
  int speed = STRAIGHT_PWM;
  if (SEGMENT_1_DISTANCE_CM - travelledCm <= SLOW_APPROACH_DISTANCE_CM) {
    speed = APPROACH_PWM;
  }
  goStraight(speed, false);
}

void pivot1State(unsigned long cT) {
  static bool firstTime = true;
  static unsigned long startTime = 0;
  if (firstTime) {
    firstTime = false;
    startTime = cT;
    spin(FIRST_PIVOT_DEG);
    if (DEBUG_ENABLED) {
      Serial.println("Entree : PIVOT 1 - GAUCHE 90 DEGRES");
    }
    return;
  }
  bool transitionTimeout = cT - startTime >= PIVOT_TIMEOUT_MS;
  bool transitionReady = pivotReached(cT);
  // Le delai maximum reste prioritaire.
  if (transitionTimeout) {
    stopMotors();
    faultReason = PIVOT_1_TIMEOUT;
    firstTime = true;
    appState = FAULT_STATE;
    return;
  }
  if (transitionReady) {
    stopMotors();
    firstTime = true;
    appState = SEGMENT_2_STATE;
  }
}

void segment2State(unsigned long cT) {
  static bool firstTime = true;
  static bool segmentStarted = false;
  static unsigned long startTime = 0;
  if (firstTime) {
    firstTime = false;
    startTime = cT;
    segmentStarted = false;
    segment2DistanceCm = 0.0;
    segment2ExpectedDistanceCm = 0.0;
    distanceCm = -1.0;
    sonarPrevious = cT;
    sonarReady = false;
    sonarFresh = false;
    sonarHasPrevious = false;
    sonarChangeDetected = false;
    arrivalCount = 0;
    stopMotors();
    showProgress(0.0, SECOND_SEGMENT_MARKER_LED);
    if (DEBUG_ENABLED) {
      Serial.println("Entree : SEGMENT 2");
    }
    return;
  }
  if (cT - startTime < SEGMENT_TIMEOUT_MS) {
    sonarTask(cT);
  }
  cT = millis();
  bool transitionTimeout = cT - startTime >= SEGMENT_TIMEOUT_MS;
  bool transitionTooClose = sonarReady
      && distanceCm < OBSTACLE_STOP_DISTANCE_CM - STOP_TOLERANCE_CM;
  bool transitionBadSonar = sonarChangeDetected;
  if (transitionTimeout || transitionTooClose || transitionBadSonar) {
    stopMotors();
    faultReason = SEGMENT_2_TIMEOUT;
    if (transitionTooClose) {
      faultReason = OBSTACLE_TOO_CLOSE;
    } else if (transitionBadSonar) {
      faultReason = SONAR_CHANGE;
    }
    firstTime = true;
    appState = FAULT_STATE;
    return;
  }
  if (!sonarReady) {
    arrivalCount = 0;
    stopMotors();
    return;
  }
  if (segmentStarted) {
    segment2DistanceCm = getSegmentDistanceCm();
    showProgress(segment2DistanceCm / segment2ExpectedDistanceCm, SECOND_SEGMENT_MARKER_LED);
  }
  if (sonarFresh) {
    bool inConfirmationZone = distanceCm <= OBSTACLE_STOP_DISTANCE_CM + STOP_TOLERANCE_CM;
    if (distanceCm <= OBSTACLE_STOP_DISTANCE_CM
        || (arrivalCount > 0 && inConfirmationZone)) {
      arrivalCount++;
    } else {
      arrivalCount = 0;
    }
  }
  bool transitionPivot2 = arrivalCount >= ARRIVAL_READINGS;
  if (arrivalCount > 0) {
    stopMotors();
    if (transitionPivot2) {
      showProgress(1.0, SECOND_SEGMENT_MARKER_LED);
      firstTime = true;
      appState = PIVOT_2_STATE;
    }
    return;
  }
  int speed = STRAIGHT_PWM;
  if (distanceCm - OBSTACLE_STOP_DISTANCE_CM <= SLOW_APPROACH_DISTANCE_CM) {
    speed = APPROACH_PWM;
  }
  if (!segmentStarted) {
    segmentStarted = true;
    segmentStartLeftDeg = encoderLeft.getCurPos();
    segmentStartRightDeg = encoderRight.getCurPos();
    segment2ExpectedDistanceCm = distanceCm - OBSTACLE_STOP_DISTANCE_CM;
    goStraight(speed, true);
    return;
  }
  goStraight(speed, false);
}

void pivot2State(unsigned long cT) {
  static bool firstTime = true;
  static unsigned long startTime = 0;
  if (firstTime) {
    firstTime = false;
    startTime = cT;
    sonarReady = false;
    sonarFresh = false;
    spin(SECOND_PIVOT_DEG);
    if (DEBUG_ENABLED) {
      Serial.println("Entree : PIVOT 2 - DROITE 90 DEGRES");
    }
    return;
  }
  bool transitionTimeout = cT - startTime >= PIVOT_TIMEOUT_MS;
  bool transitionSegment3 = pivotReached(cT);
  if (transitionTimeout) {
    stopMotors();
    faultReason = PIVOT_2_TIMEOUT;
    firstTime = true;
    appState = FAULT_STATE;
    return;
  }
  if (transitionSegment3) {
    stopMotors();
    firstTime = true;
    appState = SEGMENT_3_STATE;
  }
}

void segment3State(unsigned long cT) {
  static bool firstTime = true;
  static unsigned long startTime = 0;
  if (firstTime) {
    firstTime = false;
    startTime = cT;
    segmentStartLeftDeg = encoderLeft.getCurPos();
    segmentStartRightDeg = encoderRight.getCurPos();
    segment3DistanceCm = 0.0;
    showProgress(0.0, THIRD_SEGMENT_MARKER_LED);
    goStraight(STRAIGHT_PWM, true);
    if (DEBUG_ENABLED) {
      Serial.println("Entree : SEGMENT 3");
    }
    return;
  }
  float travelledCm = getSegmentDistanceCm();
  segment3DistanceCm = travelledCm;
  showProgress(travelledCm / SEGMENT_3_DISTANCE_CM, THIRD_SEGMENT_MARKER_LED);
  bool transitionTimeout = cT - startTime >= SEGMENT_TIMEOUT_MS;
  bool transitionReady = travelledCm >= SEGMENT_3_DISTANCE_CM;
  if (transitionTimeout) {
    stopMotors();
    faultReason = SEGMENT_3_TIMEOUT;
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
  if (SEGMENT_3_DISTANCE_CM - travelledCm <= SLOW_APPROACH_DISTANCE_CM) {
    speed = APPROACH_PWM;
  }
  goStraight(speed, false);
}

void readyState() {
  static bool firstTime = true;
  if (firstTime) {
    firstTime = false;
    stopMotors();
    showProgress(1.0, THIRD_SEGMENT_MARKER_LED);
    if (DEBUG_ENABLED) {
      Serial.println("Entree : SEGMENT 3 TERMINE - PIVOT FINAL NON INTEGRE");
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
      switch (faultReason) {
        case PIVOT_1_TIMEOUT:
          Serial.println("Entree : ARRET - TEMPS MAXIMUM PIVOT 1");
          break;
        case SEGMENT_2_TIMEOUT:
          Serial.println("Entree : ARRET - TEMPS MAXIMUM SEGMENT 2");
          break;
        case PIVOT_2_TIMEOUT:
          Serial.println("Entree : ARRET - TEMPS MAXIMUM PIVOT 2");
          break;
        case SEGMENT_3_TIMEOUT:
          Serial.println("Entree : ARRET - TEMPS MAXIMUM SEGMENT 3");
          break;
        case SONAR_CHANGE:
          Serial.println("Entree : ARRET - VARIATION SONAR ABERRANTE");
          break;
        case OBSTACLE_TOO_CLOSE:
          Serial.println("Entree : ARRET - OBSTACLE TROP PROCHE");
          break;
        default:
          Serial.println("Entree : ARRET - TEMPS MAXIMUM SEGMENT 1");
          break;
      }
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
  if (appState == SEGMENT_3_STATE || appState == READY_STATE
      || (appState == FAULT_STATE && faultReason == SEGMENT_3_TIMEOUT)) {
    Serial.print(segment3DistanceCm, 1);
  } else if (appState == SEGMENT_2_STATE || appState == PIVOT_2_STATE
      || (appState == FAULT_STATE && (faultReason == SEGMENT_2_TIMEOUT
          || faultReason == SONAR_CHANGE || faultReason == OBSTACLE_TOO_CLOSE
          || faultReason == PIVOT_2_TIMEOUT))) {
    Serial.print(segment2DistanceCm, 1);
  } else {
    Serial.print(segment1DistanceCm, 1);
  }
  Serial.print(" cm | Sonar : ");
  if (sonarReady) {
    Serial.print(distanceCm, 1);
  } else {
    Serial.print("INVALIDE");
  }
  Serial.print(" cm | Angle Z : ");
  Serial.println(gyro.getAngleZ(), 1);
}
