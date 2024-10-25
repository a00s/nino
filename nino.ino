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
const int platePin = 5;
const int topHeaterPin = 6;

volatile int fanTachCounter = 0;

int numberOfCycles = 5;
float denaturationTemp = 95.0;
float annealingTemp = 55.0;
float extensionTemp = 72.0;
unsigned long denaturationTime = 10000; // 10 segundos para testes
unsigned long annealingTime = 10000; // 10 segundos para testes
unsigned long extensionTime = 10000; // 10 segundos para testes

float slowDownRange = 12.0;
bool pcrRunning = false;
bool pcrAborted = false;
bool topHeaterOn = false;

int startButtonX = 10, startButtonY = 10, startButtonW = 100, startButtonH = 50;
int stopButtonX = 120, stopButtonY = 10, stopButtonW = 100, stopButtonH = 50;
int topHeaterButtonX = 230, topHeaterButtonY = 10, topHeaterButtonW = 80, topHeaterButtonH = 50;

const int maxPoints = 100;
float temperatureHistory[maxPoints];
float targetHistory[maxPoints];
int currentIndex = maxPoints - 1;

float currentTemp = 0.0;
float previousTemp = 0.0;

float tempChangeRate = 0.0;
unsigned long lastTempUpdateTime = 0;

void setup() {
  Serial.begin(9600);

  pinMode(fanControlPin, OUTPUT);
  pinMode(fanTachPin, INPUT_PULLUP);
  pinMode(platePin, OUTPUT);
  pinMode(topHeaterPin, OUTPUT);
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
  updateTemperature();

  if (pcrRunning) {
    updateChart(currentTemp, denaturationTemp);
  }

  if (ts.touched()) {
    TS_Point p = ts.getPoint();
    int16_t x = p.x;
    int16_t y = p.y;
    Serial.print("x: ");
    Serial.print(x);
    Serial.print(", y: ");
    Serial.println(y);
    // x = map(x, 200, 3800, 0, 320); // Ajuste do mapeamento do toque
    // y = map(y, 200, 3800, 0, 480);
    // Serial.print(" x: ");
    // Serial.print(x);
    // Serial.print(", y: ");
    // Serial.println(y);
    // if (x > startButtonX && x < (startButtonX + startButtonW) && y > startButtonY && y < (startButtonY + startButtonH) && !pcrRunning) {
    if ((x > 320 && x < 688 && y < 3760 & y > 2700) && !pcrRunning) {
      Serial.println("Iniciando PCR...");
      pcrRunning = true;
      pcrAborted = false;
      runPCR();
    }

    if (x > stopButtonX && x < (stopButtonX + stopButtonW) && y > stopButtonY && y < (stopButtonY + stopButtonH) && pcrRunning) {
      Serial.println("Abortando PCR...");
      pcrAborted = true;
      pcrRunning = false;
      stopAllSystems();
    }
    if (x > 320 && x < 688 && y < 1160 & y > 300){
    // if (x > topHeaterButtonX && x < (topHeaterButtonX + topHeaterButtonW) && y > topHeaterButtonY && y < (topHeaterButtonY + topHeaterButtonH)) {
      Serial.println("Heater ON/OFF");
      topHeaterOn = !topHeaterOn;
      analogWrite(topHeaterPin, topHeaterOn ? 100 : 0);
      drawTopHeaterButton();
    }
  }

  delay(100);
}

void updateTemperature() {
  currentTemp = readTemperature();
  unsigned long currentTime = millis();

  if (lastTempUpdateTime != 0) {
    float timeDifference = (currentTime - lastTempUpdateTime) / 1000.0;
    tempChangeRate = (currentTemp - previousTemp) / timeDifference;
  }

  lastTempUpdateTime = currentTime;
  previousTemp = currentTemp;

  displayTemperatures(currentTemp, pcrRunning ? denaturationTemp : 0, tempChangeRate);
}

void drawButtons() {
  tft.fillRect(startButtonX, startButtonY, startButtonW, startButtonH, 0x07E0);
  tft.setTextColor(0x0000);
  tft.setCursor(startButtonX + 22, startButtonY + 20);
  tft.print("START");

  tft.fillRect(stopButtonX, stopButtonY, stopButtonW, stopButtonH, 0xF800);
  tft.setTextColor(0xFFFF);
  tft.setCursor(stopButtonX + 30, stopButtonY + 20);
  tft.print("STOP");

  drawTopHeaterButton();
}

void drawTopHeaterButton() {
  tft.fillRect(topHeaterButtonX, topHeaterButtonY, topHeaterButtonW, topHeaterButtonH, topHeaterOn ? 0xFDA0 : 0x001F); // Laranja para ON, Azul para OFF
  tft.setTextColor(0xFFFF);
  tft.setCursor(topHeaterButtonX + 35, topHeaterButtonY + 20);
  tft.print("H");
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
  bool reachedTargetTemp = false;
  unsigned long timeAtTargetTemp = 0;

  while (!pcrAborted) {
    updateTemperature();

    float tempDifference = targetTemp - currentTemp;

    // Controlar aquecimento ou resfriamento
    if (tempDifference > 0.5) {
      int power = map(tempDifference, 0, slowDownRange, 0, 255);
      power = constrain(power, 0, 255);
      analogWrite(platePin, power);
      analogWrite(fanControlPin, 0);
      reachedTargetTemp = false;
    } else if (tempDifference < -0.5) {
      analogWrite(platePin, 0);
      analogWrite(fanControlPin, 255);
      reachedTargetTemp = false;
    } else {
      // Se a temperatura está dentro da faixa alvo, tente manter
      analogWrite(platePin, 0);
      analogWrite(fanControlPin, 0);
      
      // Verifique se já atingiu a temperatura alvo e inicie o timer
      if (!reachedTargetTemp) {
        timeAtTargetTemp = millis(); // Começa a contar quando atinge a temperatura
        reachedTargetTemp = true;
      }
      
      // Verifica se o tempo desejado foi mantido na temperatura
      if (reachedTargetTemp && (millis() - timeAtTargetTemp >= duration)) {
        break; // Avance para a próxima fase
      }
    }

    updateChart(currentTemp, targetTemp);

    // Verifica se o botão de "stop" foi pressionado
    if (ts.touched()) {
      TS_Point p = ts.getPoint();
      int16_t x = p.x;
      int16_t y = p.y;
      Serial.print("x: ");
      Serial.print(x);
      Serial.print(", y: ");
      Serial.println(y);
      // x = map(x, 200, 3800, 0, 320); // Ajuste do mapeamento do toque
      // y = map(y, 200, 3800, 0, 480);
      if (x > 320 && x < 688 && y < 2400 & y > 1345) {
      // if (x > stopButtonX && x < (stopButtonX + stopButtonW) && y > stopButtonY && y < (stopButtonY + stopButtonH)) {
        Serial.println("Abortando PCR...");
        pcrAborted = true;
        stopAllSystems();
        break;
      }
    }

    delay(100); // Pequeno atraso para evitar sobrecarregar o sistema
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
  // Atualiza o gráfico rolando da direita para a esquerda
  if (currentIndex > 0) {
    currentIndex--;
  } else {
    currentIndex = maxPoints - 1;
  }

  temperatureHistory[currentIndex] = currentTemp;
  targetHistory[currentIndex] = targetTemp;

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
    tft.print(" Obj: ");
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

  for (int i = 0; i < maxPoints - 1; i++) {
    int x1 = map(i, 0, maxPoints - 1, tft.width(), 0);
    int y1Current = map(temperatureHistory[(currentIndex + i) % maxPoints], 20, 100, tft.height() - 10, 110);
    int y1Target = map(targetHistory[(currentIndex + i) % maxPoints], 20, 100, tft.height() - 10, 110);

    int x2 = map(i + 1, 0, maxPoints - 1, tft.width(), 0);
    int y2Current = map(temperatureHistory[(currentIndex + i + 1) % maxPoints], 20, 100, tft.height() - 10, 110);
    int y2Target = map(targetHistory[(currentIndex + i + 1) % maxPoints], 20, 100, tft.height() - 10, 110);

    for (int j = -1; j <= 1; j++) {
      tft.drawLine(x1, y1Current + j, x2, y2Current + j, 0xFFFF);
      tft.drawLine(x1, y1Target + j, x2, y2Target + j, 0xF800);
    }
  }
}

void stopAllSystems() {
  pcrRunning = false;
  analogWrite(platePin, 0);
  analogWrite(topHeaterPin, 0);
  topHeaterOn = false;
  analogWrite(topHeaterPin, 0);
  drawTopHeaterButton();

  Serial.println("All systems stopped.");

  // Liga o ventilador até a temperatura ficar abaixo de 40 graus
  while (readTemperature() > 40) {
    analogWrite(fanControlPin, 255);
    delay(500);
    updateTemperature();
  }

  analogWrite(fanControlPin, 0);
}

void fanTachISR() {
  fanTachCounter++;
}
