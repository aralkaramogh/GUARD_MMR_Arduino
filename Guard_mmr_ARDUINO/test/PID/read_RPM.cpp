/**
 * ============================================================================
 * RPM READER ONLY - SPEED PIN MONITOR (OPTIMIZED FOR UNO)
 * ============================================================================
 * 
 * Memory-optimized version for Arduino Uno (2KB RAM limit)
 * This code ONLY reads and displays RPM from JKBLD300 SPEED output.
 * 
 * Wiring:
 *   JKBLD300 SPEED → Arduino Pin A0 (Analog 0)
 *   JKBLD300 GND → Arduino GND (CRITICAL!)
 *   Potentiometer → JKBLD300 (manual speed control)
 * 
 * Commands: M=Status, V=Toggle, R=Reset, S=Stats, H=Help
 * ============================================================================
 */

#include <Arduino.h>

// ===== Pin Configuration =====
const int SPEED_FB = A0;
const int LED_PIN = 13;

// ===== Hardware Constants =====
const float POLE_PAIRS = 4.0f;
const float GEAR_RATIO = 16.0f;
const float DRIVER_CONSTANT = 20.0f;
const float MAX_WHEEL_RPM = 3000.0f / GEAR_RATIO;

// ===== Speed Measurement =====
volatile unsigned long speedPulseCount = 0;
unsigned long lastPulseCount = 0;
unsigned long lastCalcTime = 0;
float currentRPM = 0.0f;

// ===== Statistics =====
float minRPM = 9999.0f;
float maxRPM = 0.0f;
float sumRPM = 0.0f;
unsigned int sampleCount = 0;

// ===== Timing =====
unsigned long lastPrintTime = 0;
unsigned long startTime = 0;

// ===== Settings =====
bool continuousPrint = true;

// ===== Pin State Tracking =====
int lastPinState = 0;
unsigned long lastPinChangeTime = 0;

// ===== ISR =====
void speedISR() {
  speedPulseCount++;
}

// ===== RPM Calculation =====
void calculateRPM() {
  unsigned long now = millis();
  unsigned long dt = now - lastCalcTime;
  
  if (dt >= 100) {
    noInterrupts();
    unsigned long count = speedPulseCount;
    interrupts();
    
    unsigned long pulses = count - lastPulseCount;
    float freq = (float)pulses * 10.0f;  // dt=100ms, so *10 for Hz
    
    float motorRPM = (freq / POLE_PAIRS) * DRIVER_CONSTANT;
    currentRPM = motorRPM / GEAR_RATIO;
    
    if (currentRPM > 1.0f) {
      if (currentRPM < minRPM) minRPM = currentRPM;
      if (currentRPM > maxRPM) maxRPM = currentRPM;
      sumRPM += currentRPM;
      sampleCount++;
    }
    
    lastPulseCount = count;
    lastCalcTime = now;
  }
}

// ===== Print Functions =====
void printHeader() {
  Serial.println(F("\n=== RPM READER ==="));
  Serial.print(F("Gearbox: ")); Serial.print(GEAR_RATIO,0); Serial.println(F(":1"));
  Serial.print(F("Max RPM: ")); Serial.println((int)MAX_WHEEL_RPM);
  Serial.println(F("\nRPM  | Pulses | Time"));
  Serial.println(F("-----+--------+------"));
}

void printRPM() {
  Serial.print(currentRPM, 1);
  Serial.print(F(" | "));
  Serial.print(speedPulseCount);
  Serial.print(F(" | "));
  Serial.println((millis() - startTime) / 1000);
}

void printStatus() {
  Serial.println(F("\n=== STATUS ==="));
  Serial.print(F("RPM: ")); Serial.println(currentRPM, 1);
  Serial.print(F("Pulses: ")); Serial.println(speedPulseCount);
  Serial.print(F("Time: ")); Serial.print((millis()-startTime)/1000); Serial.println(F("s"));
  
  if (sampleCount > 0) {
    Serial.println(F("\nStats:"));
    Serial.print(F("Min: ")); Serial.println(minRPM, 1);
    Serial.print(F("Max: ")); Serial.println(maxRPM, 1);
    Serial.print(F("Avg: ")); Serial.println(sumRPM/sampleCount, 1);
  }
  Serial.println();
}

void printStats() {
  if (sampleCount == 0) {
    Serial.println(F("\nNo data yet"));
    return;
  }
  
  Serial.println(F("\n=== STATISTICS ==="));
  Serial.print(F("Samples: ")); Serial.println(sampleCount);
  Serial.print(F("Min RPM: ")); Serial.println(minRPM, 1);
  Serial.print(F("Max RPM: ")); Serial.println(maxRPM, 1);
  Serial.print(F("Avg RPM: ")); Serial.println(sumRPM/sampleCount, 1);
  Serial.print(F("Range: ")); Serial.println(maxRPM - minRPM, 1);
  Serial.println();
}

void resetStats() {
  speedPulseCount = 0;
  lastPulseCount = 0;
  minRPM = 9999.0f;
  maxRPM = 0.0f;
  sumRPM = 0.0f;
  sampleCount = 0;
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
  
  pinMode(SPEED_FB, INPUT);
  pinMode(LED_PIN, OUTPUT);
  
  startTime = millis();
  lastCalcTime = millis();
  lastPrintTime = millis();
  lastPinChangeTime = millis();
  
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
  // Poll A0 pin for rising edges
  int currentPinState = digitalRead(SPEED_FB);
  if (currentPinState > lastPinState && (millis() - lastPinChangeTime) > 2) {
    speedPulseCount++;
    lastPinChangeTime = millis();
  }
  lastPinState = currentPinState;
  
  calculateRPM();
  processCommand();
  
  if (continuousPrint && (millis() - lastPrintTime >= 200)) {
    printRPM();
    lastPrintTime = millis();
  }
  
  digitalWrite(LED_PIN, (currentRPM > 5.0f) ? HIGH : LOW);
}