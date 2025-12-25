/**
 * ============================================================================
 * RPM READER - 4 MOTOR SPEED MONITOR (ARDUINO MEGA)
 * ============================================================================
 * 
 * This code reads and displays RPM from 4 JKBLD300 SPEED outputs.
 * Optimized for Arduino Mega with digital pins.
 * 
 * Wiring:
 *   Motor 1 SPEED → Arduino Pin 50 (Digital)
 *   Motor 2 SPEED → Arduino Pin 51 (Digital)
 *   Motor 3 SPEED → Arduino Pin 52 (Digital)
 *   Motor 4 SPEED → Arduino Pin 53 (Digital)
 *   All JKBLD300 GND → Arduino GND (CRITICAL!)
 * 
 * Commands: M=Status, V=Toggle, R=Reset, S=Stats, H=Help
 * ============================================================================
 */

#include <Arduino.h>

// ===== Pin Configuration =====
const int SPEED_FB[4] = {50, 51, 52, 53};
const int LED_PIN = 13;
const int NUM_MOTORS = 4;

// ===== Hardware Constants =====
const float POLE_PAIRS = 4.0f;
const float GEAR_RATIO = 16.0f;
const float DRIVER_CONSTANT = 20.0f;
const float MAX_WHEEL_RPM = 3000.0f / GEAR_RATIO;

// ===== Speed Measurement =====
volatile unsigned long speedPulseCount[4] = {0, 0, 0, 0};
unsigned long lastPulseCount[4] = {0, 0, 0, 0};
unsigned long lastCalcTime = 0;
float currentRPM[4] = {0.0f, 0.0f, 0.0f, 0.0f};

// ===== Statistics =====
float minRPM[4] = {9999.0f, 9999.0f, 9999.0f, 9999.0f};
float maxRPM[4] = {0.0f, 0.0f, 0.0f, 0.0f};
float sumRPM[4] = {0.0f, 0.0f, 0.0f, 0.0f};
unsigned int sampleCount[4] = {0, 0, 0, 0};

// ===== Timing =====
unsigned long lastPrintTime = 0;
unsigned long startTime = 0;

// ===== Settings =====
bool continuousPrint = true;

// ===== Pin State Tracking =====
int lastPinState[4] = {0, 0, 0, 0};
unsigned long lastPinChangeTime[4] = {0, 0, 0, 0};

// ===== RPM Calculation =====
void calculateRPM() {
  unsigned long now = millis();
  unsigned long dt = now - lastCalcTime;
  
  if (dt >= 100) {
    for (int i = 0; i < NUM_MOTORS; i++) {
      unsigned long count = speedPulseCount[i];
      unsigned long pulses = count - lastPulseCount[i];
      float freq = (float)pulses * 10.0f;  // dt=100ms, so *10 for Hz
      
      float motorRPM = (freq / POLE_PAIRS) * DRIVER_CONSTANT;
      currentRPM[i] = motorRPM / GEAR_RATIO;
      
      if (currentRPM[i] > 1.0f) {
        if (currentRPM[i] < minRPM[i]) minRPM[i] = currentRPM[i];
        if (currentRPM[i] > maxRPM[i]) maxRPM[i] = currentRPM[i];
        sumRPM[i] += currentRPM[i];
        sampleCount[i]++;
      }
      
      lastPulseCount[i] = count;
    }
    lastCalcTime = now;
  }
}

// ===== Print Functions =====
void printHeader() {
  Serial.println(F("\n=== RPM READER (4 Motors) ==="));
  Serial.print(F("Gearbox: ")); Serial.print(GEAR_RATIO, 0); Serial.println(F(":1"));
  Serial.print(F("Max RPM: ")); Serial.println((int)MAX_WHEEL_RPM);
  Serial.println(F("\nM1 RPM | M2 RPM | M3 RPM | M4 RPM | Time(s)"));
  Serial.println(F("-------|--------|--------|--------|--------"));
}

void printRPM() {
  for (int i = 0; i < NUM_MOTORS; i++) {
    Serial.print(currentRPM[i], 1);
    if (i < NUM_MOTORS - 1) Serial.print(F("   | "));
  }
  Serial.print(F("   | "));
  Serial.println((millis() - startTime) / 1000);
}

void printStatus() {
  Serial.println(F("\n=== STATUS ==="));
  for (int i = 0; i < NUM_MOTORS; i++) {
    Serial.print(F("Motor ")); Serial.print(i + 1); Serial.print(F(" RPM: "));
    Serial.println(currentRPM[i], 1);
  }
  Serial.println();
  for (int i = 0; i < NUM_MOTORS; i++) {
    Serial.print(F("M")); Serial.print(i + 1);
    Serial.print(F(" Pulses: ")); Serial.println(speedPulseCount[i]);
  }
  Serial.print(F("Time: ")); Serial.print((millis() - startTime) / 1000); Serial.println(F("s"));
  
  for (int i = 0; i < NUM_MOTORS; i++) {
    if (sampleCount[i] > 0) {
      Serial.print(F("\nM")); Serial.print(i + 1); Serial.println(F(" Stats:"));
      Serial.print(F("  Min: ")); Serial.println(minRPM[i], 1);
      Serial.print(F("  Max: ")); Serial.println(maxRPM[i], 1);
      Serial.print(F("  Avg: ")); Serial.println(sumRPM[i] / sampleCount[i], 1);
    }
  }
  Serial.println();
}

void printStats() {
  bool hasData = false;
  for (int i = 0; i < NUM_MOTORS; i++) {
    if (sampleCount[i] > 0) hasData = true;
  }
  
  if (!hasData) {
    Serial.println(F("\nNo data yet"));
    return;
  }
  
  Serial.println(F("\n=== STATISTICS ==="));
  for (int i = 0; i < NUM_MOTORS; i++) {
    if (sampleCount[i] > 0) {
      Serial.print(F("\nMotor ")); Serial.print(i + 1); Serial.println(F(":"));
      Serial.print(F("  Samples: ")); Serial.println(sampleCount[i]);
      Serial.print(F("  Min RPM: ")); Serial.println(minRPM[i], 1);
      Serial.print(F("  Max RPM: ")); Serial.println(maxRPM[i], 1);
      Serial.print(F("  Avg RPM: ")); Serial.println(sumRPM[i] / sampleCount[i], 1);
      Serial.print(F("  Range: ")); Serial.println(maxRPM[i] - minRPM[i], 1);
    }
  }
  Serial.println();
}

void resetStats() {
  for (int i = 0; i < NUM_MOTORS; i++) {
    speedPulseCount[i] = 0;
    lastPulseCount[i] = 0;
    minRPM[i] = 9999.0f;
    maxRPM[i] = 0.0f;
    sumRPM[i] = 0.0f;
    sampleCount[i] = 0;
  }
  startTime = millis();
  lastCalcTime = millis();
  Serial.println(F("\nReset!"));
}

void printHelp() {
  Serial.println(F("\n=== HELP ==="));
  Serial.println(F("M - Status"));
  Serial.println(F("V - Toggle print"));
  Serial.println(F("R - Reset"));
  Serial.println(F("S - Statistics"));
  Serial.println(F("C - Clear/header"));
  Serial.println(F("H - Help"));
  Serial.println(F("\nExpected max: ~187 RPM"));
  Serial.println();
}

// ===== Command Processing =====
void processCommand() {
  if (Serial.available()) {
    char cmd = Serial.read();
    while (Serial.available()) Serial.read();
    
    switch (cmd) {
      case 'm': case 'M':
        printStatus();
        break;
      case 'v': case 'V':
        continuousPrint = !continuousPrint;
        Serial.print(F("\nPrint: "));
        Serial.println(continuousPrint ? F("ON") : F("OFF"));
        if (continuousPrint) printHeader();
        break;
      case 'r': case 'R':
        resetStats();
        break;
      case 's': case 'S':
        printStats();
        break;
      case 'c': case 'C':
        printHeader();
        break;
      case 'h': case 'H':
        printHelp();
        break;
      case '\n': case '\r':
        break;
      default:
        Serial.println(F("\nPress H for help"));
        break;
    }
  }
}

// ===== Setup =====
void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 2000);
  
  for (int i = 0; i < NUM_MOTORS; i++) {
    pinMode(SPEED_FB[i], INPUT);
  }
  pinMode(LED_PIN, OUTPUT);
  
  startTime = millis();
  lastCalcTime = millis();
  lastPrintTime = millis();
  
  for (int i = 0; i < NUM_MOTORS; i++) {
    lastPinChangeTime[i] = millis();
  }
  
  printHeader();
  
  // LED blink
  for (int i = 0; i < 3; i++) {
    digitalWrite(LED_PIN, HIGH);
    delay(100);
    digitalWrite(LED_PIN, LOW);
    delay(100);
  }
  
  Serial.println(F("Ready!"));
}

// ===== Main Loop =====
void loop() {
  // Poll all 4 pins for rising edges
  for (int i = 0; i < NUM_MOTORS; i++) {
    int currentPinState = digitalRead(SPEED_FB[i]);
    if (currentPinState > lastPinState[i] && (millis() - lastPinChangeTime[i]) > 2) {
      speedPulseCount[i]++;
      lastPinChangeTime[i] = millis();
    }
    lastPinState[i] = currentPinState;
  }
  
  calculateRPM();
  processCommand();
  
  if (continuousPrint && (millis() - lastPrintTime >= 200)) {
    printRPM();
    lastPrintTime = millis();
  }
  
  // LED indicates if any motor is spinning
  bool anyMotorRunning = false;
  for (int i = 0; i < NUM_MOTORS; i++) {
    if (currentRPM[i] > 5.0f) {
      anyMotorRunning = true;
      break;
    }
  }
  digitalWrite(LED_PIN, anyMotorRunning ? HIGH : LOW);
}