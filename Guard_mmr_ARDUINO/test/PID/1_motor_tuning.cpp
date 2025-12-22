/**
 * ============================================================================
 * SINGLE MOTOR PID TEST - MOTOR 0 (Right Front)
 * ============================================================================
 * 
 * Simplified version for testing and tuning ONE motor before deploying to 4WD.
 * Use this to verify wiring, tune PID gains, and understand system behavior.
 * 
 * Hardware:
 * - Arduino Uno
 * - 1x JKBLD300 V2 driver
 * - 1x 86BLF BLDC motor
 * - 16:1 Gearbox
 * 
 * Wiring (Motor 0):
 * - PWM: Arduino Pin 3 → JKBLD300 PWM input
 * - DIR: Arduino Pin 4 → JKBLD300 DIR input
 * - SPEED: JKBLD300 SPEED output → Arduino Pin 2 (INT0)
 * - GND: Common ground between Arduino and JKBLD300
 * 
 * Testing Procedure:
 * 1. Upload this code
 * 2. Open Serial Monitor (115200 baud)
 * 3. Press 'G' to enable CSV mode (optional, for Python GUI)
 * 4. Send T50 to test at 50 RPM
 * 5. Tune PID with P/I/D commands or Python GUI
 * 6. Once stable, transfer gains to 4-motor code
 * 
 * Commands:
 * - T<rpm>  : Set target RPM (e.g., T100)
 * - P<val>  : Set Kp (e.g., P1.5)
 * - I<val>  : Set Ki (e.g., I0.5)
 * - D<val>  : Set Kd (e.g., D0.05)
 * - F<val>  : Set feed-forward gain (e.g., F0.85)
 * - X/Space : Stop motor
 * - M       : Print status
 * - V       : Toggle continuous printing
 * - G       : Toggle CSV mode (for Python GUI)
 * - R       : Run test sequence (50, 100, 150 RPM)
 * 
 * ============================================================================
 */

#include <Arduino.h>

// ===== Pin Configuration =====
const int LED_PIN = 13;
const int MOTOR_PWM = 6;        // PWM output to JKBLD300
const int MOTOR_DIR = 8;        // Direction output to JKBLD300
const int SPEED_FB = A1;         // SPEED feedback from JKBLD300 (INT0)

// ===== Hardware Constants =====
const float POLE_PAIRS = 4.0f;           // 86BLF: 8 poles = 4 pole pairs
const float GEAR_RATIO = 16.0f;          // 16:1 gearbox
const float DRIVER_CONSTANT = 20.0f;     // JKBLD300 specific
const float MAX_MOTOR_RPM = 3000.0f;     // 86BLF rated speed
const float MAX_WHEEL_RPM = MAX_MOTOR_RPM / GEAR_RATIO;  // 187.5 RPM

// ===== PID Variables =====
float kp = 0.8f;                // Proportional gain
float ki = 0.5f;                // Integral gain
float kd = 0.05f;               // Derivative gain
float feedForwardGain = 0.85f;  // Feed-forward gain

float targetRPM = 0.0f;         // Target speed
float currentRPM = 0.0f;        // Measured speed
float integralSum = 0.0f;       // Integral accumulator
float lastError = 0.0f;         // Previous error
float pwmOutput = 0.0f;         // PWM output (0-255)

// ===== Speed Measurement =====
volatile unsigned long speedPulseCount = 0;
unsigned long lastPulseCount = 0;
unsigned long lastCalcTime = 0;

// ===== Timing =====
const unsigned long PID_INTERVAL = 50;     // 50ms = 20Hz
const unsigned long RPM_INTERVAL = 50;     // 50ms
const unsigned long PRINT_INTERVAL = 100;  // 100ms
unsigned long lastPIDTime = 0;
unsigned long lastPrintTime = 0;

// ===== Control Flags =====
bool pidEnabled = true;
bool continuousPrint = false;
bool csvMode = false;
int motorDirection = 1;  // 1 = forward, -1 = backward

// ===== Limits =====
const float INTEGRAL_LIMIT = 100.0f;
const int PWM_MIN = 0;
const int PWM_MAX = 255;

// ===== ISR for SPEED feedback =====
void speedISR() {
  speedPulseCount++;
}

// ===== Utility Functions =====
int limitPWM(float value) {
  if (value > PWM_MAX) return PWM_MAX;
  if (value < PWM_MIN) return PWM_MIN;
  return (int)value;
}

float limitFloat(float value, float minVal, float maxVal) {
  if (value > maxVal) return maxVal;
  if (value < minVal) return minVal;
  return value;
}

// ===== Motor Control =====
void setMotor(int pwm) {
  pwm = limitPWM(pwm);
  
  // Determine direction from target sign
  bool forward = (targetRPM >= 0);
  motorDirection = forward ? 1 : -1;
  
  digitalWrite(MOTOR_DIR, forward ? HIGH : LOW);
  analogWrite(MOTOR_PWM, pwm);
}

void stopMotor() {
  analogWrite(MOTOR_PWM, 0);
  targetRPM = 0;
  integralSum = 0;
  lastError = 0;
  if (!csvMode) Serial.println("Motor STOPPED");
}

// ===== RPM Calculation =====
void calculateRPM() {
  unsigned long currentTime = millis();
  unsigned long deltaTime = currentTime - lastCalcTime;
  
  if (deltaTime >= RPM_INTERVAL) {
    noInterrupts();
    unsigned long currentCount = speedPulseCount;
    interrupts();
    
    unsigned long deltaPulses = currentCount - lastPulseCount;
    float dt_sec = deltaTime / 1000.0f;
    
    // Calculate frequency (Hz)
    float frequency = (float)deltaPulses / dt_sec;
    
    // Motor RPM using JKBLD300 formula
    float motorRPM = (frequency / POLE_PAIRS) * DRIVER_CONSTANT;
    
    // Wheel RPM (after gearbox)
    float wheelRPM = motorRPM / GEAR_RATIO;
    
    // Apply direction sign
    currentRPM = wheelRPM * motorDirection;
    
    lastPulseCount = currentCount;
    lastCalcTime = currentTime;
  }
}

// ===== PID Control with Feed-Forward =====
void updatePID() {
  unsigned long currentTime = millis();
  
  if (currentTime - lastPIDTime >= PID_INTERVAL) {
    float dt = (currentTime - lastPIDTime) / 1000.0f;
    
    if (!pidEnabled) {
      // Direct PWM control (open-loop)
      int pwm = map(abs((int)targetRPM), 0, (int)MAX_WHEEL_RPM, 0, 255);
      pwmOutput = pwm;
      integralSum = 0;
      lastError = 0;
      setMotor(pwm);
      lastPIDTime = currentTime;
      return;
    }
    
    // Calculate error
    float error = targetRPM - currentRPM;
    
    // Proportional term
    float pTerm = kp * error;
    
    // Integral term with anti-windup
    integralSum += error * dt;
    integralSum = limitFloat(integralSum, -INTEGRAL_LIMIT, INTEGRAL_LIMIT);
    float iTerm = ki * integralSum;
    
    // Derivative term
    float dTerm = 0;
    if (dt > 0) {
      dTerm = kd * (error - lastError) / dt;
    }
    
    // PID result
    float pidResult = pTerm + iTerm + dTerm;
    
    // Feed-forward term
    float feedForward = abs(targetRPM) * feedForwardGain;
    
    // Combine
    float totalOutput = pidResult + feedForward;
    
    // Limit and apply
    int pwm = limitPWM(totalOutput);
    pwmOutput = pwm;
    
    setMotor(pwm);
    
    // Store for next iteration
    lastError = error;
    lastPIDTime = currentTime;
  }
}

// ===== Status Printing =====
void printStatus() {
  if (csvMode) {
    // CSV format: Target,Actual,PWM,Kp,Ki,Kd
    Serial.print(abs(targetRPM), 1); Serial.print(",");
    Serial.print(abs(currentRPM), 1); Serial.print(",");
    Serial.print(pwmOutput, 1); Serial.print(",");
    Serial.print(kp, 2); Serial.print(",");
    Serial.print(ki, 2); Serial.print(",");
    Serial.println(kd, 3);
  } else {
    Serial.println("\n=== Motor Status ===");
    Serial.print("PID: "); Serial.println(pidEnabled ? "ON" : "OFF");
    Serial.print("Target RPM: "); Serial.println(targetRPM, 1);
    Serial.print("Actual RPM: "); Serial.println(currentRPM, 1);
    Serial.print("Error: "); Serial.println(targetRPM - currentRPM, 1);
    Serial.print("PWM Output: "); Serial.println((int)pwmOutput);
    Serial.print("Pulse Count: "); Serial.println(speedPulseCount);
    Serial.println("\nPID Gains:");
    Serial.print("  Kp: "); Serial.println(kp, 2);
    Serial.print("  Ki: "); Serial.println(ki, 2);
    Serial.print("  Kd: "); Serial.println(kd, 3);
    Serial.print("  FF: "); Serial.println(feedForwardGain, 2);
    Serial.println();
  }
}

// ===== Test Sequence =====
void runTestSequence() {
  if (csvMode) {
    Serial.println("# Test sequence (disable CSV mode first)");
    return;
  }
  
  Serial.println("\n=== Running Test Sequence ===");
  Serial.println("Testing: 50 → 100 → 150 → 100 → 50 → 0 RPM");
  Serial.println("Each step: 5 seconds\n");
  
  int testSpeeds[] = {50, 100, 150, 100, 50, 0};
  int numSteps = 6;
  
  for (int i = 0; i < numSteps; i++) {
    targetRPM = testSpeeds[i];
    integralSum = 0;  // Reset integral
    
    Serial.print("Target: "); Serial.print(testSpeeds[i]); Serial.println(" RPM");
    
    unsigned long stepStart = millis();
    while (millis() - stepStart < 5000) {
      calculateRPM();
      updatePID();
      
      if (millis() - lastPrintTime >= 500) {
        Serial.print("  Actual: "); Serial.print(currentRPM, 1);
        Serial.print(" | Error: "); Serial.print(testSpeeds[i] - currentRPM, 1);
        Serial.print(" | PWM: "); Serial.println((int)pwmOutput);
        lastPrintTime = millis();
      }
    }
    Serial.println();
  }
  
  Serial.println("=== Test Complete ===\n");
}

// ===== Serial Command Processing =====
void processCommand() {
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    input.trim();
    
    if (input.length() == 0) return;
    
    // Multi-character commands (e.g., P1.5, T100)
    if (input.length() > 1) {
      char cmd = input.charAt(0);
      float val = input.substring(1).toFloat();
      
      switch(cmd) {
        case 'P': case 'p':
          kp = val;
          if (!csvMode) { Serial.print("Kp = "); Serial.println(kp, 2); }
          return;
        case 'I': case 'i':
          ki = val;
          if (!csvMode) { Serial.print("Ki = "); Serial.println(ki, 2); }
          return;
        case 'D': case 'd':
          kd = val;
          if (!csvMode) { Serial.print("Kd = "); Serial.println(kd, 3); }
          return;
        case 'T': case 't':
          targetRPM = val;
          integralSum = 0;
          if (!csvMode) { 
            Serial.print("Target = "); Serial.print(targetRPM, 1); 
            Serial.println(" RPM"); 
          }
          return;
        case 'F': case 'f':
          feedForwardGain = val;
          if (!csvMode) { Serial.print("FF Gain = "); Serial.println(feedForwardGain, 2); }
          return;
      }
    }
    
    // Single character commands
    char cmd = input.charAt(0);
    switch (cmd) {
      case 'x': case 'X': case ' ':
        stopMotor();
        break;
        
      case 'm': case 'M':
        printStatus();
        break;
        
      case 'v': case 'V':
        continuousPrint = !continuousPrint;
        if (!csvMode) {
          Serial.print("Continuous Print: ");
          Serial.println(continuousPrint ? "ON" : "OFF");
        }
        break;
        
      case 'g': case 'G':
        csvMode = !csvMode;
        continuousPrint = csvMode;
        Serial.print("CSV Mode: ");
        Serial.println(csvMode ? "ON" : "OFF");
        break;
        
      case 'r': case 'R':
        runTestSequence();
        break;
        
      case 'h': case 'H':
        stopMotor();
        kp = 0.8f;
        ki = 0.5f;
        kd = 0.05f;
        feedForwardGain = 0.85f;
        if (!csvMode) Serial.println("Reset to default values");
        break;
    }
  }
}

// ===== Setup =====
void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000);  // Wait up to 3s for serial
  
  // Configure pins
  pinMode(MOTOR_PWM, OUTPUT);
  pinMode(MOTOR_DIR, OUTPUT);
  pinMode(SPEED_FB, INPUT);
  pinMode(LED_PIN, OUTPUT);
  
  // Initialize motor (stopped)
  digitalWrite(MOTOR_DIR, HIGH);
  analogWrite(MOTOR_PWM, 0);
  digitalWrite(LED_PIN, LOW);
  
  // Attach interrupt
  attachInterrupt(digitalPinToInterrupt(SPEED_FB), speedISR, RISING);
  
  // Initialize timing
  lastCalcTime = millis();
  lastPIDTime = millis();
  lastPrintTime = millis();
  
  // Print header
  Serial.println("\n\n");
  Serial.println("============================================");
  Serial.println("  SINGLE MOTOR PID TEST - MOTOR 0");
  Serial.println("============================================");
  Serial.println();
  Serial.println("Hardware:");
  Serial.println("  - Arduino Uno (5V TTL)");
  Serial.println("  - JKBLD300 V2 Driver");
  Serial.println("  - 86BLF Motor (8-pole)");
  Serial.println("  - 16:1 Gearbox");
  Serial.println();
  Serial.println("Wiring:");
  Serial.println("  PWM:   Pin 3  → JKBLD300 PWM");
  Serial.println("  DIR:   Pin 4  → JKBLD300 DIR");
  Serial.println("  SPEED: Pin 2  ← JKBLD300 SPEED");
  Serial.println("  GND:   Common ground (CRITICAL!)");
  Serial.println();
  Serial.println("Commands:");
  Serial.println("  T<rpm>  - Set target (e.g., T100)");
  Serial.println("  P<val>  - Set Kp (e.g., P1.5)");
  Serial.println("  I<val>  - Set Ki (e.g., I0.5)");
  Serial.println("  D<val>  - Set Kd (e.g., D0.05)");
  Serial.println("  F<val>  - Set FF gain (e.g., F0.85)");
  Serial.println("  X/Space - Stop motor");
  Serial.println("  M       - Print status");
  Serial.println("  V       - Toggle continuous print");
  Serial.println("  G       - Toggle CSV mode (for GUI)");
  Serial.println("  R       - Run test sequence");
  Serial.println("  H       - Reset to defaults");
  Serial.println();
  Serial.println("Quick Start:");
  Serial.println("  1. Send: T50   (test at 50 RPM)");
  Serial.println("  2. Send: M     (check status)");
  Serial.println("  3. Send: R     (run full test)");
  Serial.println();
  Serial.println("Initial Settings:");
  Serial.print("  Kp: "); Serial.println(kp, 2);
  Serial.print("  Ki: "); Serial.println(ki, 2);
  Serial.print("  Kd: "); Serial.println(kd, 3);
  Serial.print("  FF: "); Serial.println(feedForwardGain, 2);
  Serial.print("  Max Wheel RPM: "); Serial.println((int)MAX_WHEEL_RPM);
  Serial.println();
  Serial.println("Ready! Send commands...");
  Serial.println("============================================\n");
  
  // Brief LED flash to indicate ready
  digitalWrite(LED_PIN, HIGH);
  delay(500);
  digitalWrite(LED_PIN, LOW);
}

// ===== Main Loop =====
void loop() {
  // Update calculations
  calculateRPM();
  updatePID();
  processCommand();
  
  // Continuous status printing
  if (continuousPrint && (millis() - lastPrintTime >= PRINT_INTERVAL)) {
    printStatus();
    lastPrintTime = millis();
  }
  
  // LED indicates motor running
  digitalWrite(LED_PIN, (abs(targetRPM) > 0) ? HIGH : LOW);
}