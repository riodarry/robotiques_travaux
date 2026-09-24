#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

const int SCREEN_WIDTH = 128;
const int SCREEN_HEIGHT = 64;
const int ECHO_PIN = 11;
const int TRIG_PIN = 12;

const char NOM_FAMILLE[] = "NGUEUDAM";


const int LARGEUR_VOITURE = 24;

const int DISTANCE_INVALIDE = -1;
const unsigned long INTERVALLE_DISTANCE_MS = 250;
const int DISTANCE_ANIMATION_MIN_CM = 10;
const int DISTANCE_ANIMATION_MAX_CM = 50;

static const unsigned char PROGMEM soleil_bmp[] = {
  B00100000,
  B10101000,
  B01110000,
  B11111000,
  B01110000,
  B10101000,
  B00100000,
  B00000000
};

static const unsigned char PROGMEM voiture_bmp[] = {
  B00000000, B00000000, B00000000,
  B00000011, B11111100, B00000000,
  B00000111, B11111110, B00000000,
  B00001110, B00000111, B00000000,
  B00011111, B11111111, B10000000,
  B00111111, B11111111, B11000000,
  B11111111, B11111111, B11110000,
  B11111111, B11111111, B11111100,
  B11111111, B11111111, B11111100,
  B00111100, B00000000, B11110000,
  B00111100, B00000000, B11110000,
  B00011000, B00000000, B01100000
};

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

unsigned long currentTime = 0;
int distanceCm = DISTANCE_INVALIDE;

void setup() {
  Serial.begin(115200);
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println(F("Echec d'allocation SSD1306"));
    for (;;) {}
  }

  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.display();
}

void loop() {
  currentTime = millis();

  distanceCm = distanceTask(currentTime);
  printDistanceTask(currentTime, distanceCm);
  displayTask(currentTime, distanceCm);
}

int distanceTask(unsigned long ct) {
  static unsigned long lastTime = 0;
  static int lastResult = DISTANCE_INVALIDE;

  if (ct - lastTime < INTERVALLE_DISTANCE_MS) {
    return lastResult;
  }
  lastTime = ct;

  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  unsigned long duration = pulseIn(ECHO_PIN, HIGH);
  int result = duration * 0.034 / 2;

  if (result >= 2 && result <= 400) {
    lastResult = result;
  }

  return lastResult;
}

void printDistanceTask(unsigned long ct, int distance) {
  static unsigned long lastTime = 0;

  if (ct - lastTime < INTERVALLE_DISTANCE_MS) {
    return;
  }
  lastTime = ct;

  if (distance != DISTANCE_INVALIDE) {
    Serial.print(F("Distance : "));
    Serial.print(distance);
    Serial.println(F(" cm"));
  }
}

void drawName() {
  display.setTextSize(1);
  display.setCursor(0, SCREEN_HEIGHT - 8);
  display.print(NOM_FAMILLE);
}

void displayTask(unsigned long ct, int distance) {
  static unsigned long lastTime = 0;

  if (ct - lastTime < 100) {
    return;
  }
  lastTime = ct;

  int carX = (SCREEN_WIDTH - LARGEUR_VOITURE) / 2;
  if (distance != DISTANCE_INVALIDE) {
    int limitedDistance = constrain(distance, DISTANCE_ANIMATION_MIN_CM,
                                    DISTANCE_ANIMATION_MAX_CM);
    carX = map(limitedDistance, DISTANCE_ANIMATION_MIN_CM,
               DISTANCE_ANIMATION_MAX_CM,
               SCREEN_WIDTH - LARGEUR_VOITURE, 0);
  }

  display.clearDisplay();
  display.drawBitmap(0, 0, soleil_bmp, 5, 8, SSD1306_WHITE);
  drawName();
  display.drawBitmap(carX, 40, voiture_bmp, LARGEUR_VOITURE, 12,
                     SSD1306_WHITE);
  display.display();
}
