/**
 * ============================================================================
 * COMPREHENSIVE MOTOR DIAGNOSTIC TEST - FIXED WITH POLLING
 * ============================================================================
 * 
 * Tests each motor individually to identify control issues.
 * Uses POLLING (not interrupts) to read speed feedback from all 4 motors.
 * 
 * Wiring:
 *   PWM:   Pins 3, 5, 6, 9
 *   DIR:   Pins 4, 7, 8, 12
 *   SPEED: Pins 50, 51, 52, 53 (Digital input, no interrupt needed)
 * 
 * Commands:
 *   1-4     : Select motor (1=RF, 2=LF, 3=RB, 4=LB)
 *   W       : Forward (increase speed by 10 PWM)
 *   S       : Backward (decrease speed by 10 PWM)
 *   X       : Stop selected motor
 *   A       : Stop ALL motors
 *   M       : Print detailed status
 *   R       : Reset integral and error
 *   C       : Clear screen
 *   P/I/D   : Tune gains (format: P0.8)
 *   V       : Toggle continuous print
 *   H       : Help menu
 * 
 * ============================================================================
 */

#include <Arduino.h>

// ===== Pin Configuration =====
const int LED_PIN = 13;

// PWM Pins
const int MOTOR_PWM_1 = 3;
const int MOTOR_PWM_2 = 5;
const int MOTOR_PWM_3 = 6;
const int MOTOR_PWM_4 = 9;

// Direction Pins
const int MOTOR_DIR_1 = 4;
const int MOTOR_DIR_2 = 7;
const int MOTOR_DIR_3 = 8;
const int MOTOR_DIR_4 = 12;

// Speed Feedback Pins (POLLING MODE - NO INTERRUPTS)
const int SPEED_FB_1 = 50;
const int SPEED_FB_2 = 51;
const int SPEED_FB_3 = 52;
const int SPEED_FB_4 = 53;

// Grouping Arrays
const int PWM_PINS[] = { MOTOR_PWM_1, MOTOR_PWM_2, MOTOR_PWM_3, MOTOR_PWM_4 };
const int DIR_PINS[] = { MOTOR_DIR_1, MOTOR_DIR_2, MOTOR_DIR_3, MOTOR_DIR_4 };
const int SPEED_FB_PINS[] = { SPEED_FB_1, SPEED_FB_2, SPEED_FB_3, SPEED_FB_4 };

const int MOTOR_COUNT = 4;
const char* MOTOR_NAMES[] = {"RF (M1)", "LF (M2)", "RB (M3)", "LB (M4)"};
const int MOTOR_INVERT[] = { 0, 1, 0, 1 };

// ===== Hardware Constants =====
const float POLE_PAIRS = 4.0f;
const float GEAR_RATIO = 16.0f;
const float DRIVER_CONSTANT = 20.0f;
const float MAX_MOTOR_RPM = 3000.0f;
const float MAX_WHEEL_RPM = MAX_MOTOR_RPM / GEAR_RATIO;  // 187.5 RPM

// ===== PID Configuration =====
struct PIDController {
  float kp = 0.8f;
  float ki = 0.5f;
  float kd = 0.05f;
  float setpoint = 0;
  float integral = 0;
  float lastError = 0;
  float output = 0;
};

PIDController motorPID[MOTOR_COUNT];

// ===== Speed Measurement (POLLING MODE) =====
volatile unsigned long speedPulseCounts[MOTOR_COUNT] = {0, 0, 0, 0};
unsigned long lastSpeedPulseCounts[MOTOR_COUNT] = {0, 0, 0, 0};
float currentRPM[MOTOR_COUNT] = {0, 0, 0, 0};
int currentDirection[MOTOR_COUNT] = {1, 1, 1, 1};

// Pin state tracking for polling
int lastPinState[MOTOR_COUNT] = {0, 0, 0, 0};
unsigned long lastPinChangeTime[MOTOR_COUNT] = {0, 0, 0, 0};

// ===== Timing =====
const unsigned long PID_UPDATE_INTERVAL = 50;
const unsigned long RPM_CALC_INTERVAL = 100;  // 100ms for RPM calculation
const unsigned long PRINT_INTERVAL = 200;
unsigned long lastPIDUpdate = 0;
unsigned long lastRPMCalc = 0;
unsigned long lastPrint = 0;

// ===== Test Control =====
int selectedMotor = 0;
int currentPWM[MOTOR_COUNT] = {0, 0, 0, 0};
bool continuousPrint = true;
bool pidEnabled = false;

const float INTEGRAL_LIMIT = 100.0f;
const int PWM_MIN = 0;
const int PWM_MAX = 255;
const int DEBOUNCE_TIME = 2;  // 2ms debounce

// ===== POLLING: Detect rising edges on speed pins =====
void pollSpeedFeedback() {
  for (int i = 0; i < MOTOR_COUNT; i++) {
    int currentPinState = digitalRead(SPEED_FB_PINS[i]);
    
    // Detect rising edge with debounce
    if (currentPinState > lastPinState[i] && (millis() - lastPinChangeTime[i]) > DEBOUNCE_TIME) {
      speedPulseCounts[i]++;
      lastPinChangeTime[i] = millis();
    }
    lastPinState[i] = currentPinState;
  }
}

// ===== Utility Functions =====
int limitPWM(float value) {
  return constrain((int)value, PWM_MIN, PWM_MAX);
}

float limitFloat(float value, float minVal, float maxVal) {
  return constrain(value, minVal, maxVal);
}

// ===== Motor Control =====
void setMotorPWM(int motorIndex, int pwmValue) {
  if (motorIndex < 0 || motorIndex >= MOTOR_COUNT) return;
  
  pwmValue = limitPWM(pwmValue);
  currentPWM[motorIndex] = pwmValue;
  
  // Stop: PWM = 0
  if (pwmValue == 0) {
    analogWrite(PWM_PINS[motorIndex], 0);
    digitalWrite(DIR_PINS[motorIndex], LOW);
    return;
  }
  
  // Set direction based on setpoint sign
  bool forward = (motorPID[motorIndex].setpoint >= 0);
  
  // Apply inversion
  if (MOTOR_INVERT[motorIndex]) forward = !forward;
  
  digitalWrite(DIR_PINS[motorIndex], forward ? HIGH : LOW);
  analogWrite(PWM_PINS[motorIndex], abs(pwmValue));
}

// ===== RPM Calculation =====
void calculateRPM() {
  unsigned long now = millis();
  unsigned long dt = now - lastRPMCalc;
  
  if (dt >= RPM_CALC_INTERVAL) {
    float dt_sec = dt / 1000.0f;
    
    for (int i = 0; i < MOTOR_COUNT; i++) {
      unsigned long pulses = speedPulseCounts[i] - lastSpeedPulseCounts[i];
      float freq = (float)pulses / dt_sec;  // Frequency in Hz
      
      // Motor RPM using JKBLD300 formula
      float motorRPM = (freq / POLE_PAIRS) * DRIVER_CONSTANT;
      
      // Wheel RPM after gearbox
      float wheelRPM = motorRPM / GEAR_RATIO;
      
      // Apply direction
      currentRPM[i] = wheelRPM * currentDirection[i];
      
      lastSpeedPulseCounts[i] = speedPulseCounts[i];
    }
    lastRPMCalc = now;
  }
}

// ===== PID Control =====
void updatePIDControl() {
  unsigned long now = millis();
  
  if (now - lastPIDUpdate >= PID_UPDATE_INTERVAL) {
    float dt = (now - lastPIDUpdate) / 1000.0f;
    
    for (int i = 0; i < MOTOR_COUNT; i++) {
      if (!pidEnabled) {
        // Direct PWM control mode
        setMotorPWM(i, currentPWM[i]);
        motorPID[i].output = currentPWM[i];
        continue;
      }
      
      // PID mode
      float error = motorPID[i].setpoint - currentRPM[i];
      
      float pTerm = motorPID[i].kp * error;
      
      motorPID[i].integral += error * dt;
      motorPID[i].integral = limitFloat(motorPID[i].integral, -INTEGRAL_LIMIT, INTEGRAL_LIMIT);
      float iTerm = motorPID[i].ki * motorPID[i].integral;
      
      float dTerm = 0;
      if (dt > 0) {
        dTerm = motorPID[i].kd * (error - motorPID[i].lastError) / dt;
      }
      
      float pidOutput = pTerm + iTerm + dTerm;
      motorPID[i].output = pidOutput;
      motorPID[i].lastError = error;
      
      setMotorPWM(i, (int)pidOutput);
    }
    
    lastPIDUpdate = now;
  }
}

// ===== Status Printing =====
void printHeader() {
  Serial.println("\n====================================================");
  Serial.println("     MOTOR DIAGNOSTIC TEST - FIXED");
  Serial.println("====================================================\n");
}

void printSelectedMotor() {
  Serial.print("\n>>> SELECTED MOTOR: ");
  Serial.print(selectedMotor + 1);
  Serial.print(" (");
  Serial.print(MOTOR_NAMES[selectedMotor]);
  Serial.println(")");
  Serial.println("Commands: W(+10) S(-10) X(stop) M(status) R(reset) H(help)\n");
}

void printDetailedStatus() {
  Serial.println("\n========== DETAILED MOTOR STATUS ==========");
  Serial.println("Motor | Target | Actual | PWM | Pulses");
  Serial.println("------|--------|--------|-----|--------");
  
  for (int i = 0; i < MOTOR_COUNT; i++) {
    char marker = (i == selectedMotor) ? '*' : ' ';
    Serial.print(marker);
    Serial.print(" ");
    Serial.print(MOTOR_NAMES[i]);
    Serial.print(" | ");
    
    Serial.print(motorPID[i].setpoint, 1);
    Serial.print(" | ");
    Serial.print(currentRPM[i], 1);
    Serial.print(" | ");
    Serial.print(currentPWM[i]);
    Serial.print(" | ");
    Serial.println(speedPulseCounts[i]);
  }
  
  Serial.println("\n========== PID SETTINGS ==========");
  Serial.print("Mode: "); Serial.println(pidEnabled ? "PID CONTROL" : "DIRECT PWM");
  Serial.print("Kp="); Serial.print(motorPID[selectedMotor].kp, 2);
  Serial.print(" Ki="); Serial.print(motorPID[selectedMotor].ki, 2);
  Serial.print(" Kd="); Serial.println(motorPID[selectedMotor].kd, 3);
  Serial.print("Integral: "); Serial.println(motorPID[selectedMotor].integral, 3);
  Serial.print("Last Error: "); Serial.println(motorPID[selectedMotor].lastError, 2);
  Serial.println();
}

void printQuickStatus() {
  Serial.print("[M");
  Serial.print(selectedMotor + 1);
  Serial.print("] Tgt:");
  Serial.print(motorPID[selectedMotor].setpoint, 0);
  Serial.print(" Act:");
  Serial.print(currentRPM[selectedMotor], 1);
  Serial.print(" PWM:");
  Serial.print(currentPWM[selectedMotor]);
  Serial.print(" Pulses:");
  Serial.println(speedPulseCounts[selectedMotor]);
}

void printHelp() {
  Serial.println("\n========== COMMAND REFERENCE ==========");
  Serial.println("MOTOR SELECTION:");
  Serial.println("  1-4     Select motor 1-4");
  Serial.println("\nMOTOR CONTROL:");
  Serial.println("  W       Increase speed (+10 PWM)");
  Serial.println("  S       Decrease speed (-10 PWM)");
  Serial.println("  X       Stop selected motor");
  Serial.println("  A       Stop ALL motors");
  Serial.println("\nPID TUNING:");
  Serial.println("  P0.8    Set Kp=0.8");
  Serial.println("  I0.5    Set Ki=0.5");
  Serial.println("  D0.05   Set Kd=0.05");
  Serial.println("  T       Toggle PID mode");
  Serial.println("  R       Reset integral & error");
  Serial.println("\nDISPLAY:");
  Serial.println("  M       Print detailed status");
  Serial.println("  V       Toggle continuous print");
  Serial.println("  C       Clear screen");
  Serial.println("  H       Show this help");
  Serial.println("=========================================\n");
}

// ===== Command Processing =====
void processCommand() {
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    input.trim();
    
    if (input.length() == 0) return;
    
    // Commands with parameters
    if (input.length() > 1) {
      char cmd = input.charAt(0);
      float val = input.substring(1).toFloat();
      
      switch(cmd) {
        case 'P': case 'p':
          motorPID[selectedMotor].kp = val;
          Serial.print("M"); Serial.print(selectedMotor + 1);
          Serial.print(" Kp = "); Serial.println(val, 2);
          return;
        case 'I': case 'i':
          motorPID[selectedMotor].ki = val;
          Serial.print("M"); Serial.print(selectedMotor + 1);
          Serial.print(" Ki = "); Serial.println(val, 2);
          return;
        case 'D': case 'd':
          motorPID[selectedMotor].kd = val;
          Serial.print("M"); Serial.print(selectedMotor + 1);
          Serial.print(" Kd = "); Serial.println(val, 3);
          return;
      }
    }
    
    // Single character commands
    char cmd = input.charAt(0);
    switch(cmd) {
      case '1': case '2': case '3': case '4':
        selectedMotor = cmd - '1';
        printSelectedMotor();
        break;
        
      case 'w': case 'W':
        currentPWM[selectedMotor] = min(currentPWM[selectedMotor] + 10, 255);
        motorPID[selectedMotor].setpoint = currentPWM[selectedMotor];
        Serial.print("M"); Serial.print(selectedMotor + 1);
        Serial.print(" -> PWM "); Serial.println(currentPWM[selectedMotor]);
        break;
        
      case 's': case 'S':
        currentPWM[selectedMotor] = max(currentPWM[selectedMotor] - 10, 0);
        motorPID[selectedMotor].setpoint = currentPWM[selectedMotor];
        Serial.print("M"); Serial.print(selectedMotor + 1);
        Serial.print(" -> PWM "); Serial.println(currentPWM[selectedMotor]);
        break;
        
      case 'x': case 'X':
        currentPWM[selectedMotor] = 0;
        motorPID[selectedMotor].setpoint = 0;
        motorPID[selectedMotor].integral = 0;
        motorPID[selectedMotor].lastError = 0;
        Serial.print("M"); Serial.print(selectedMotor + 1);
        Serial.println(" STOPPED");
        break;
        
      case 'a': case 'A':
        for (int i = 0; i < MOTOR_COUNT; i++) {
          currentPWM[i] = 0;
          motorPID[i].setpoint = 0;
          motorPID[i].integral = 0;
          motorPID[i].lastError = 0;
        }
        Serial.println("ALL MOTORS STOPPED");
        break;
        
      case 'r': case 'R':
        motorPID[selectedMotor].integral = 0;
        motorPID[selectedMotor].lastError = 0;
        Serial.print("M"); Serial.print(selectedMotor + 1);
        Serial.println(" integral & error RESET");
        break;
        
      case 't': case 'T':
        pidEnabled = !pidEnabled;
        Serial.print("Mode switched to: ");
        Serial.println(pidEnabled ? "PID CONTROL" : "DIRECT PWM");
        if (!pidEnabled) {
          for (int i = 0; i < MOTOR_COUNT; i++) {
            motorPID[i].integral = 0;
            motorPID[i].lastError = 0;
          }
        }
        break;
        
      case 'm': case 'M':
        printDetailedStatus();
        break;
        
      case 'v': case 'V':
        continuousPrint = !continuousPrint;
        Serial.print("Continuous Print: ");
        Serial.println(continuousPrint ? "ON" : "OFF");
        break;
        
      case 'c': case 'C':
        for (int i = 0; i < 15; i++) Serial.println();
        printHeader();
        printSelectedMotor();
        break;
        
      case 'h': case 'H':
        printHelp();
        break;
        
      default:
        Serial.println("? Unknown command. Press H for help.");
        break;
    }
  }
}

// ===== Setup =====
void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 2000);
  
  // Initialize PID controllers
  for (int i = 0; i < MOTOR_COUNT; i++) {
    motorPID[i].kp = 0.8f;
    motorPID[i].ki = 0.5f;
    motorPID[i].kd = 0.05f;
  }
  
  // Configure motor pins
  for (int i = 0; i < MOTOR_COUNT; i++) {
    pinMode(PWM_PINS[i], OUTPUT);
    pinMode(DIR_PINS[i], OUTPUT);
    pinMode(SPEED_FB_PINS[i], INPUT);
    digitalWrite(PWM_PINS[i], LOW);
    digitalWrite(DIR_PINS[i], LOW);
  }
  
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);
  
  lastPIDUpdate = millis();
  lastRPMCalc = millis();
  lastPrint = millis();
  
  for (int i = 0; i < MOTOR_COUNT; i++) {
    lastPinChangeTime[i] = millis();
  }
  
  // Initial print
  Serial.println("\n\n====================================================");
  Serial.println("   MOTOR DIAGNOSTIC TEST - INITIALIZATION");
  Serial.println("====================================================");
  Serial.println("\nStatus:");
  Serial.println("  [OK] Arduino Mega 2560");
  Serial.println("  [OK] All motor pins configured");
  Serial.println("  [OK] Speed feedback POLLING enabled (pins 50-53)");
  Serial.println("  [OK] Mode: DIRECT PWM (no PID)");
  Serial.println("\nTo start:");
  Serial.println("  1-4 to select motor");
  Serial.println("  H for full command reference\n");
  
  printSelectedMotor();
}

// ===== Main Loop =====
void loop() {
  pollSpeedFeedback();    // Read all 4 pins with rising edge detection
  calculateRPM();         // Calculate RPM from pulse counts
  updatePIDControl();     // Update PWM values
  processCommand();       // Handle serial commands
  
  if (continuousPrint && (millis() - lastPrint >= PRINT_INTERVAL)) {
    printQuickStatus();
    lastPrint = millis();
  }
}