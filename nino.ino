#include <math.h>

const int thermistorPin = A1;  // Pin connected to the thermistor (Analog)
const int seriesResistor = 10000;  // Value of the series resistor (10K ohms)
const float nominalResistance = 100000;  // Resistance of the thermistor at 25 degrees C (100K ohms)
const float nominalTemperature = 25;  // Nominal temperature (25 degrees C)
const float bCoefficient = 3950;  // Beta coefficient of the thermistor
const float supplyVoltage = 5.0;  // Supply voltage (5V)
const int adcMax = 1023;  // Maximum ADC value

const int fanControlPin = 9;  // PWM control pin for the fan (Digital, PWM-capable)
const int fanTachPin = 2;  // Tachometer pin for the fan (Digital, Interrupt-capable)
const int relayPin = 8;  // Pin connected to the relay for heating (Digital)
const int startButtonPin = 4;  // Pin connected to the start button (Digital)
const int abortButtonPin = 5;  // Pin connected to the abort button (Digital)

volatile int fanTachCounter = 0;
unsigned long previousMillis = 0;
const unsigned long interval = 1000;  // Interval for fan speed calculation (1 second)

int numberOfCycles = 5;  // Number of PCR cycles
float denaturationTemp = 95.0;  // Denaturation temperature in Celsius
float annealingTemp = 55.0;  // Annealing temperature in Celsius
float extensionTemp = 72.0;  // Extension temperature in Celsius
unsigned long denaturationTime = 30000;  // Denaturation time in milliseconds (30 seconds)
unsigned long annealingTime = 30000;  // Annealing time in milliseconds (30 seconds)
unsigned long extensionTime = 30000;  // Extension time in milliseconds (30 seconds)

float slowDownRange = 12.0;  // Range in degrees before target to stop heating when increasing temperature

bool pcrRunning = false;
bool pcrAborted = false;

int contador = 0;
int contadorloop = 10;

// Variables to track temperature change rate
float previousTemperature = 0.0;
unsigned long previousTime = 0;

void setup() {
  Serial.begin(9600);  // Initialize serial communication

  pinMode(fanControlPin, OUTPUT);  // Set the fan control pin as output
  pinMode(fanTachPin, INPUT_PULLUP);  // Set the fan tachometer pin as input with pullup resistor
  pinMode(relayPin, OUTPUT);  // Set the relay pin as output
  pinMode(startButtonPin, INPUT_PULLUP);  // Set the start button pin as input with pullup resistor
  pinMode(abortButtonPin, INPUT_PULLUP);  // Set the abort button pin as input with pullup resistor
  attachInterrupt(digitalPinToInterrupt(fanTachPin), fanTachISR, FALLING);  // Attach an interrupt to the tachometer pin
  
  // Ensure the fan starts at its lowest speed
  analogWrite(fanControlPin, 0);
}

void loop() {
  if (digitalRead(startButtonPin) == LOW && !pcrRunning) {
    Serial.println("Botao de inicio ativado");
    pcrRunning = true;
    pcrAborted = false;
    runPCR();
  }

  if (digitalRead(abortButtonPin) == LOW && pcrRunning) {
    Serial.println("Botao de parada pressionado");
    pcrAborted = true;
    pcrRunning = false;
    stopAllSystems();
  }

  if (!pcrRunning) {
    analogWrite(fanControlPin, 0);  // Ensure the fan is at the lowest speed when not running
  }
}

void runPCR() {
  for (int cycle = 0; cycle < numberOfCycles; cycle++) {
    if (pcrAborted) {
      break;
    }

    Serial.print("Cycle: ");
    Serial.println(cycle + 1);

    // Denaturation phase
    controlTemperature(denaturationTemp, denaturationTime);

    // Annealing phase
    controlTemperature(annealingTemp, annealingTime);

    // Extension phase
    controlTemperature(extensionTemp, extensionTime);
  }

  // PCR complete
  Serial.println("PCR Complete");
  stopAllSystems();
  pcrRunning = false;
}

void controlTemperature(float targetTemp, unsigned long duration) {
  // Stabilize temperature
  while (true) {
    if (pcrAborted) {
      return;
    }

    float temperatureC = readTemperature();
   
    if (temperatureC < targetTemp - slowDownRange) {
      digitalWrite(relayPin, HIGH);  // Turn on heating at full power
      analogWrite(fanControlPin, 0);  // Turn off fan
    } else if (temperatureC < targetTemp) {
      digitalWrite(relayPin, LOW);  // Turn off heating
      analogWrite(fanControlPin, 0);  // Turn off fan
    } else if (temperatureC > targetTemp) {
      digitalWrite(relayPin, LOW);  // Turn off heating
      analogWrite(fanControlPin, 255);  // Turn on fan at full speed
    }

    contador++;
    if (contador >= contadorloop) {
      unsigned long currentTime = millis();
      float tempChangeRate = calculateTempChangeRate(temperatureC, currentTime);

      Serial.print("Temperature: ");
      Serial.print(temperatureC, 2);  // Print the temperature with 2 decimal places
      Serial.print(" | Target Temp: ");
      Serial.print(targetTemp);
      Serial.print(" | Temp Change Rate: ");
      Serial.print(tempChangeRate, 2);  // Print the temperature change rate
      Serial.println(" C/s");

      contador = 0;
    }
    // Verifique o botão de abortar a cada iteração do loop
    if (digitalRead(abortButtonPin) == LOW) {
      pcrAborted = true;
      stopAllSystems();
      return;
    }

    delay(100);  // Reduza o atraso para permitir a verificação mais frequente do botão de abortar

    // Exit loop when temperature is stable within a small range
    if (abs(temperatureC - targetTemp) < 0.5) {
      break;
    }
  }

  // Start counting time once the temperature is stabilized
  unsigned long startMillis = millis();

  while (millis() - startMillis < duration) {
    if (pcrAborted) {
      break;
    }

    float temperatureC = readTemperature();
    
    if (temperatureC < targetTemp - slowDownRange) {
      digitalWrite(relayPin, HIGH);  // Turn on heating at full power
      analogWrite(fanControlPin, 0);  // Turn off fan
    } else if (temperatureC < targetTemp) {
      digitalWrite(relayPin, LOW);  // Turn off heating
      analogWrite(fanControlPin, 0);  // Turn off fan
    } else if (temperatureC > targetTemp) {
      digitalWrite(relayPin, LOW);  // Turn off heating
      analogWrite(fanControlPin, 255);  // Turn on fan at full speed
    }

    contador++;
    if (contador >= contadorloop) {
      unsigned long currentTime = millis();
      float tempChangeRate = calculateTempChangeRate(temperatureC, currentTime);

      Serial.print("Temperature: ");
      Serial.print(temperatureC, 2);  // Print the temperature with 2 decimal places
      Serial.print(" | Target Temp: ");
      Serial.print(targetTemp);
      Serial.print(" | Temp Change Rate: ");
      Serial.print(tempChangeRate, 2);  // Print the temperature change rate
      Serial.println(" C/s");

      contador = 0;
    }
    // Verifique o botão de abortar a cada iteração do loop
    if (digitalRead(abortButtonPin) == LOW) {
      pcrAborted = true;
      stopAllSystems();
      break;
    }

    delay(100);  // Reduza o atraso para permitir a verificação mais frequente do botão de abortar
  }
}

float readTemperature() {
  int adcValue = analogRead(thermistorPin);  // Read the analog pin value
  float voltage = adcValue * (supplyVoltage / adcMax);  // Convert the ADC value to voltage
  float resistance = seriesResistor * (supplyVoltage / voltage - 1.0);  // Calculate the resistance of the thermistor

  // Calculate temperature in Kelvin using the Beta equation
  float steinhart;
  steinhart = resistance / nominalResistance;  // (R/R0)
  steinhart = log(steinhart);  // ln(R/R0)
  steinhart /= bCoefficient;  // 1/B * ln(R/R0)
  steinhart += 1.0 / (nominalTemperature + 273.15);  // + (1/T0)
  steinhart = 1.0 / steinhart;  // Invert
  float temperatureC = steinhart - 273.15;  // Convert to Celsius
  return temperatureC;
}

float calculateTempChangeRate(float currentTemperature, unsigned long currentTime) {
  float tempChangeRate = 0.0;

  if (previousTime != 0) {
    float deltaTemp = currentTemperature - previousTemperature;
    float deltaTime = (currentTime - previousTime) / 1000.0;  // Convert milliseconds to seconds
    tempChangeRate = deltaTemp / deltaTime;
  }

  // Update previous temperature and time
  previousTemperature = currentTemperature;
  previousTime = currentTime;

  return tempChangeRate;
}

void stopAllSystems() {
  // Turn off all systems
  digitalWrite(relayPin, LOW);
  analogWrite(fanControlPin, 0);
  Serial.println("All systems stopped.");
}

void fanTachISR() {
  fanTachCounter++;  // Incrementa o contador de tacômetro
}
