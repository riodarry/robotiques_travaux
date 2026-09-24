#include <MeAuriga.h>

const int LED_COUNT = 12;
const int LED_PIN = 44;
const unsigned long SERIAL_BAUD_RATE = 115200;

const int MAX_DISTANCE_CM = 400;
const int SLOW_DISTANCE_CM = 100;
const int STOP_DISTANCE_CM = 30;

const unsigned long DISTANCE_INTERVAL = 100;
const unsigned long PRINT_INTERVAL = 250;
const unsigned long STOP_DURATION = 2000;
const unsigned long REVERSE_DURATION = 1000;
const unsigned long PIVOT_DURATION = 850;
const unsigned long CLEAR_DISTANCE_DURATION = 200;

const int CRUISE_SPEED = 180;
const int SLOW_SPEED = 90;
const int REVERSE_SPEED = 125;
const int PIVOT_SPEED = 150;

const int LED_BRIGHTNESS = 10;
const int FRONT_LED_START = 1;
const int FRONT_LED_END = 6;
const int REAR_LED_START = 7;
const int REAR_LED_END = 12;

MeRGBLed led(PORT0, LED_COUNT);
MeUltrasonicSensor sonar(PORT_10);

enum State { MARCHE, LENT, ARRET, RECULE, PIVOTE, MAX_STATE };

State currentState = MARCHE;

unsigned long currentTime = 0;
unsigned long statePrevious = 0;
int distanceCm = MAX_DISTANCE_CM;
bool nouvelleDistanceRequise = true;


// Moteur gauche
const int m1_pwm = 11;
const int m1_in1 = 48;
const int m1_in2 = 49;

// Moteur droit
const int m2_pwm = 10;
const int m2_in1 = 47;
const int m2_in2 = 46;

void setup() {
  Serial.begin(SERIAL_BAUD_RATE);

  led.setpin(LED_PIN);
  led.setColor(0, 0, 0);
  led.show();

  pinMode(m1_pwm, OUTPUT);
  pinMode(m1_in1, OUTPUT);
  pinMode(m1_in2, OUTPUT);

  pinMode(m2_pwm, OUTPUT);
  pinMode(m2_in1, OUTPUT);
  pinMode(m2_in2, OUTPUT);

  Stop();
  statePrevious = millis();
}

void loop() {
  currentTime = millis();

  distanceCm = distanceTask(currentTime);
  printDistanceTask(currentTime, distanceCm);
  stateManager();
  ledTask(currentState);
  printStateTask(currentState);
}

int distanceTask(unsigned long ct) {
  static unsigned long lastTime = 0;
  static int lastResult = MAX_DISTANCE_CM;

  if (ct - lastTime < DISTANCE_INTERVAL) {
    return lastResult;
  }

  lastTime = ct;
  lastResult = sonar.distanceCm();
  nouvelleDistanceRequise = false;

  return lastResult;
}

void printDistanceTask(unsigned long ct, int distance) {
  static unsigned long lastTime = 0;

  if (ct - lastTime < PRINT_INTERVAL) {
    return;
  }

  lastTime = ct;

  Serial.print("Distance : ");
  Serial.print(distance);
  Serial.println(" cm");
}

void stateManager() {
  static unsigned long clearDistancePrevious = 0;

  switch (currentState) {
    case MARCHE:
      clearDistancePrevious = 0;

      if (nouvelleDistanceRequise) {
        Stop();
      } else if (distanceCm < SLOW_DISTANCE_CM) {
        currentState = LENT;
      } else {
        Forward(CRUISE_SPEED);  // Environ 70 % de 255
      }
      break;

    case LENT:
      if (distanceCm < STOP_DISTANCE_CM) {
        clearDistancePrevious = 0;
        currentState = ARRET;
        statePrevious = currentTime;
        Stop();
      } else if (distanceCm >= SLOW_DISTANCE_CM) {
        if (clearDistancePrevious == 0) {
          clearDistancePrevious = currentTime;
        }

        if (currentTime - clearDistancePrevious >= CLEAR_DISTANCE_DURATION) {
          currentState = MARCHE;
          clearDistancePrevious = 0;
        } else {
          Forward(SLOW_SPEED);
        }
      } else {
        clearDistancePrevious = 0;
        Forward(SLOW_SPEED);  // 50 % de la vitesse de croisiere avec la division
      }
      break;

    case ARRET:
      Stop();

      if (currentTime - statePrevious >= STOP_DURATION) {
        currentState = RECULE;
        statePrevious = currentTime;
      }
      break;

    case RECULE:
      Backward(REVERSE_SPEED);

      if (currentTime - statePrevious >= REVERSE_DURATION) {
        currentState = PIVOTE;
        statePrevious = currentTime;
      }
      break;

    case PIVOTE:
      TurnRight(PIVOT_SPEED);  // Numero etudiant pair : sens horaire

      // Duree calibree sur le robot pour obtenir environ 180 degres.
      if (currentTime - statePrevious >= PIVOT_DURATION) {
        currentState = MARCHE;
        statePrevious = currentTime;
        nouvelleDistanceRequise = true;
        Stop();
      }
      break;

    default:
      currentState = ARRET;
      statePrevious = currentTime;
  }
}

void printStateTask(State state) {
  static State previousState = MAX_STATE;

  if (state == previousState) {
    return;
  }

  previousState = state;
  Serial.print("Etat : ");

  switch (state) {
    case MARCHE:
      Serial.println("MARCHE");
      break;
    case LENT:
      Serial.println("LENT");
      break;
    case ARRET:
      Serial.println("ARRET");
      break;
    case RECULE:
      Serial.println("RECULE");
      break;
    case PIVOTE:
      Serial.println("PIVOTE");
      break;
    default:
      Serial.println("INCONNU");
  }
}

void ledTask(State state) {
  static State previousState = MAX_STATE;

  if (state == previousState) {
    return;
  }

  previousState = state;
  led.setColor(0, 0, 0);

  if (state == MARCHE) {
    // Numero pair : moitie avant verte.
    for (int i = FRONT_LED_START; i <= FRONT_LED_END; i++) {
      led.setColor(i, 0, LED_BRIGHTNESS, 0);
    }
  } else if (state == LENT) {
    // Avant-dernier chiffre pair : moitie arriere jaune.
    for (int i = REAR_LED_START; i <= REAR_LED_END; i++) {
      led.setColor(i, LED_BRIGHTNESS, LED_BRIGHTNESS, 0);
    }

  } else {
    // ARRET, RECULE et PIVOTE : anneau entier rouge.
    for (int i = 1; i <= LED_COUNT; i++) {
      led.setColor(i, LED_BRIGHTNESS, 0, 0);
    }
  }

  led.show();
}

void Forward(int speed) {
  digitalWrite(m1_in2, LOW);
  digitalWrite(m1_in1, HIGH);
  analogWrite(m1_pwm, speed);

  digitalWrite(m2_in2, LOW);
  digitalWrite(m2_in1, HIGH);
  analogWrite(m2_pwm, speed);
}

void Backward(int speed) {
  digitalWrite(m1_in2, HIGH);
  digitalWrite(m1_in1, LOW);
  analogWrite(m1_pwm, speed);

  digitalWrite(m2_in2, HIGH);
  digitalWrite(m2_in1, LOW);
  analogWrite(m2_pwm, speed);
}

void Stop() {
  analogWrite(m1_pwm, 0);
  analogWrite(m2_pwm, 0);
}

void TurnRight(int speed) {
  digitalWrite(m1_in2, HIGH);
  digitalWrite(m1_in1, LOW);
  analogWrite(m1_pwm, speed);

  digitalWrite(m2_in2, LOW);
  digitalWrite(m2_in1, HIGH);
  analogWrite(m2_pwm, speed);
}
