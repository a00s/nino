#include <math.h>
#include <ZxTFT_ILI9488.h> // Hardware-specific library
#include <XPT2046_Touchscreen.h>
#include <SPI.h>

#define TFT_CS 10
#define TFT_DC 9
#define TFT_RST 8
#define CS_PIN 7

ZxTFT_ILI9488 tft(TFT_CS, TFT_DC, TFT_RST); // my epd connection shield for Arduino Due
XPT2046_Touchscreen ts(CS_PIN);

const int thermistorPin = A0; 
const int seriesResistor = 10000; 
const float nominalResistance = 100000; 
const float nominalTemperature = 25; 
const float bCoefficient = 3950; 
const float supplyVoltage = 5.0; 
const int adcMax = 1023;

const int fanControlPin = 3;
const int fanTachPin = 2; 
const int relayPin = 5; 

volatile int fanTachCounter = 0;
unsigned long previousMillis = 0;
const unsigned long interval = 1000; 

int numberOfCycles = 5; 
float denaturationTemp = 95.0; 
float annealingTemp = 55.0; 
float extensionTemp = 72.0; 
unsigned long denaturationTime = 30000; 
unsigned long annealingTime = 30000; 
unsigned long extensionTime = 30000;

float slowDownRange = 12.0; 

bool pcrRunning = false;
bool pcrAborted = false;

int startButtonX = 40, startButtonY = 10, startButtonW = 120, startButtonH = 50;
int stopButtonX = 180, stopButtonY = 10, stopButtonW = 120, stopButtonH = 50;

const int maxPoints = 100; 
float temperatureHistory[maxPoints];
float targetHistory[maxPoints];
int currentIndex = 0;

float currentTemp = 0.0; 
float previousTemp = 0.0; 

float tempChangeRate = 0.0;
unsigned long lastTempUpdateTime = 0;

void setup() {
  Serial.begin(9600); 

  pinMode(fanControlPin, OUTPUT); 
  pinMode(fanTachPin, INPUT_PULLUP); 
  pinMode(relayPin, OUTPUT); 
  attachInterrupt(digitalPinToInterrupt(fanTachPin), fanTachISR, FALLING);

  analogWrite(fanControlPin, 0);

  pinMode(TFT_CS, OUTPUT);
  digitalWrite(TFT_CS, HIGH);
  tft.init();
  ts.begin();
  tft.setRotation(2);

  tft.fillScreen(0x0000); 
  tft.setTextColor(0xFFFF);
  tft.setTextSize(2);

  drawButtons();

  for (int i = 0; i < maxPoints; i++) {
    temperatureHistory[i] = 0;
    targetHistory[i] = 0;
  }
}

void loop() {
  currentTemp = readTemperature();
  unsigned long currentTime = millis();

  if (lastTempUpdateTime != 0) {
    float timeDifference = (currentTime - lastTempUpdateTime) / 1000.0;
    tempChangeRate = (currentTemp - previousTemp) / timeDifference;
  }

  lastTempUpdateTime = currentTime;
  previousTemp = currentTemp;

  displayTemperatures(currentTemp, pcrRunning ? denaturationTemp : 0, tempChangeRate);

  if (pcrRunning) {
    updateChart(currentTemp, denaturationTemp);
  }

  if (ts.touched()) {
    TS_Point p = ts.getPoint();
    int16_t x = p.x;
    int16_t y = p.y;

    if (x > 357 && x < 665 && y > 1960 && y < 3330 && !pcrRunning) {
      Serial.println("Iniciando PCR...");
      pcrRunning = true;
      pcrAborted = false;
      runPCR();
    }

    if (x > 357 && x < 665 && y > 380 && y < 1743 && pcrRunning) {
      Serial.println("Abortando PCR...");
      pcrAborted = true;
      pcrRunning = false;
      stopAllSystems();
    }
  }

  delay(100);
}

void drawButtons() {
  tft.fillRect(startButtonX, startButtonY, startButtonW, startButtonH, 0x07E0); 
  tft.setTextColor(0x0000); 
  tft.setCursor(startButtonX + 10, startButtonY + 20);
  tft.print("  START");

  tft.fillRect(stopButtonX, stopButtonY, stopButtonW, stopButtonH, 0xF800); 
  tft.setTextColor(0xFFFF); 
  tft.setCursor(stopButtonX + 10, stopButtonY + 20);
  tft.print("  STOP");
}

void runPCR() {
  for (int cycle = 0; cycle < numberOfCycles; cycle++) {
    if (pcrAborted) break;

    Serial.print("Cycle: ");
    Serial.println(cycle + 1);

    controlTemperature(denaturationTemp, denaturationTime);
    if (pcrAborted) break;

    controlTemperature(annealingTemp, annealingTime);
    if (pcrAborted) break;

    controlTemperature(extensionTemp, extensionTime);
    if (pcrAborted) break;
  }

  if (!pcrAborted) {
    Serial.println("PCR Complete");
  }
  stopAllSystems();
  pcrRunning = false;
}

void controlTemperature(float targetTemp, unsigned long duration) {
  unsigned long startMillis = millis();

  while (!pcrAborted && (millis() - startMillis < duration || fabs(currentTemp - targetTemp) > 0.5)) {
    currentTemp = readTemperature();

    if (currentTemp < targetTemp - slowDownRange) {
      analogWrite(relayPin, 255);
      analogWrite(fanControlPin, 0);
    } else if (currentTemp < targetTemp) {
      analogWrite(relayPin, 100);
      analogWrite(fanControlPin, 0);
    } else if (currentTemp > targetTemp) {
      analogWrite(relayPin, 0);
      analogWrite(fanControlPin, 255);
    }

    updateChart(currentTemp, targetTemp);
    displayTemperatures(currentTemp, targetTemp, tempChangeRate);

    if (ts.touched()) {
      TS_Point p = ts.getPoint();
      int16_t x = p.x;
      int16_t y = p.y;
      if (x > 357 && x < 665 && y > 380 && y < 1743) {
        Serial.println("Abortando PCR...");
        pcrAborted = true;
        break;
      }
    }

    delay(500);
  }
}

float readTemperature() {
  int adcValue = analogRead(thermistorPin);
  float voltage = adcValue * (supplyVoltage / adcMax); 
  float resistance = seriesResistor * (supplyVoltage / voltage - 1.0); 

  float steinhart;
  steinhart = resistance / nominalResistance;  
  steinhart = log(steinhart);  
  steinhart /= bCoefficient;  
  steinhart += 1.0 / (nominalTemperature + 273.15);  
  steinhart = 1.0 / steinhart;  
  float temperatureC = steinhart - 273.15;  
  return temperatureC;
}

void updateChart(float currentTemp, float targetTemp) {
  temperatureHistory[currentIndex] = currentTemp;
  targetHistory[currentIndex] = targetTemp;

  currentIndex++;
  if (currentIndex >= maxPoints) {
    currentIndex = 0;
  }

  drawChart();
}

void displayTemperatures(float currentTemp, float targetTemp, float tempChangeRate) {
  tft.fillRect(0, 70, tft.width(), 40, 0x0000); 

  tft.setCursor(10, 70); 
  tft.setTextColor(0xFFFF);
  tft.setTextSize(2);
  tft.print("Temp: ");
  tft.print(currentTemp, 1);
  tft.print(" C");

  if (pcrRunning) {
    tft.print("  Obj: ");
    tft.print(targetTemp, 1);
    tft.print(" C");
  }

  tft.setCursor(10, 90); 
  tft.print("Vel: ");
  tft.print(tempChangeRate, 2);  
  tft.print(" C/s");
}

void drawChart() {
  tft.fillRect(0, 110, tft.width(), tft.height() - 110, 0x0000);

  for (int i = 1; i < maxPoints; i++) {
    int x1 = map(i - 1, 0, maxPoints - 1, 0, tft.width());
    int y1Current = map(temperatureHistory[i - 1], 20, 100, tft.height() - 10, 110);
    int y1Target = map(targetHistory[i - 1], 20, 100, tft.height() - 10, 110);

    int x2 = map(i, 0, maxPoints - 1, 0, tft.width());
    int y2Current = map(temperatureHistory[i], 20, 100, tft.height() - 10, 110);
    int y2Target = map(targetHistory[i], 20, 100, tft.height() - 10, 110);

    tft.drawLine(x1, y1Current, x2, y2Current, 0x001F);
    tft.drawLine(x1, y1Target, x2, y2Target, 0xF800);
  }
}

void stopAllSystems() {
  pcrRunning = false;
  digitalWrite(relayPin, LOW);
  analogWrite(fanControlPin, 0);
  Serial.println("All systems stopped.");
}

void fanTachISR() {
  fanTachCounter++; 
}
