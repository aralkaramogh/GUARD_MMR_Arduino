/**
 * ============================================================================
 * PID-CONTROLLED 4WD AMR MOTOR CONTROLLER - ARDUINO MEGA
 * ============================================================================
 * 
 * Hardware Configuration:
 * - MCU: Arduino MEGA 2560 (5V TTL logic)
 * - Motor Drivers: 4x JKBLD300 V2
 * - Motors: 4x 86BLF series BLDC (8-pole, rated 3000 RPM)
 * - Gearbox: 16:1 reduction ratio
 * - Drive Type: Skid-steer (differential steering)
 * 
 * ============================================================================
 * WIRING CONNECTIONS (MEGA SPECIFIC)
 * ============================================================================
 * 
 * Motor Control Pins:
 * ┌─────────────┬──────────┬──────────┬────────────────────────┐
 * │ Motor       │ PWM Pin  │ DIR Pin  │ SPEED Feedback Pin     │
 * ├─────────────┼──────────┼──────────┼────────────────────────┤
 * │ Motor 1 (RF)│ Pin 3    │ Pin 4    │ Pin 21 (INT0)          │
 * │ Motor 2 (LF)│ Pin 5    │ Pin 7    │ Pin 20 (INT1)          │
 * │ Motor 3 (RB)│ Pin 6    │ Pin 8    │ Pin 19 (INT2)          │
 * │ Motor 4 (LB)│ Pin 9    │ Pin 12   │ Pin 18 (INT3)          │
 * └─────────────┴──────────┴──────────┴────────────────────────┘
 * 
 * JKBLD300 Driver Connections (per motor):
 * - Arduino PWM pin → JKBLD300 PWM input
 * - Arduino DIR pin → JKBLD300 DIR input
 * - JKBLD300 SPEED output → Arduino interrupt pin (21, 20, 19, 18)
 * - JKBLD300 GND → Arduino GND (CRITICAL - common ground required!)
 * 
 * ============================================================================
 * SERIAL COMMANDS
 * ============================================================================
 * 
 * Movement Commands:
 *   W         → Forward
 *   S         → Backward
 *   A         → Turn Left
 *   D         → Turn Right
 *   X/Space   → Emergency Stop
 *   H         → Reset (stop + default speeds)
 * 
 * Speed Adjustment:
 *   Q         → Increase forward/backward speed (+5%)
 *   Z         → Decrease forward/backward speed (-5%)
 *   E         → Increase turning speed (+5%)
 *   C         → Decrease turning speed (-5%)
 * 
 * PID Control:
 *   P         → Toggle PID on/off
 *   M         → Print motor status (once)
 *   V         → Toggle continuous status printing
 *   G         → Toggle CSV mode (for Python GUI)
 * 
 * PID Tuning (format: <command><value>):
 *   P1.5      → Set Kp = 1.5
 *   I0.5      → Set Ki = 0.5
 *   D0.05     → Set Kd = 0.05
 *   F0.85     → Set feed-forward gain = 0.85
 *   T150      → Set target to 150 RPM (all motors)
 * 
 * ============================================================================
 */

#include <Arduino.h>

// ===== Pin Configuration (ARDUINO MEGA) =====
const int LED_PIN = 13;

// PWM Pins (3, 5, 6, 9 on MEGA)
const int MOTOR_PWM_1 = 3;
const int MOTOR_PWM_2 = 5;
const int MOTOR_PWM_3 = 6;
const int MOTOR_PWM_4 = 9;

// Direction Pins
const int MOTOR_DIR_1 = 4;
const int MOTOR_DIR_2 = 7;
const int MOTOR_DIR_3 = 8;
const int MOTOR_DIR_4 = 12;

// Speed Feedback Pins (INTERRUPT CAPABLE - MEGA ONLY)
// Pin 21 = INT0, Pin 20 = INT1, Pin 19 = INT2, Pin 18 = INT3
const int SPEED_FB_1 = 21;  // INT0
const int SPEED_FB_2 = 20;  // INT1
const int SPEED_FB_3 = 19;  // INT2
const int SPEED_FB_4 = 18;  // INT3

// Grouping Arrays
const int PWM_PINS[] = { MOTOR_PWM_1, MOTOR_PWM_2, MOTOR_PWM_3, MOTOR_PWM_4 };
const int DIR_PINS[] = { MOTOR_DIR_1, MOTOR_DIR_2, MOTOR_DIR_3, MOTOR_DIR_4 };
const int SPEED_FB_PINS[] = { SPEED_FB_1, SPEED_FB_2, SPEED_FB_3, SPEED_FB_4 };

// Motor inversion flags
const int MOTOR_INVERT[] = { 0, 1, 0, 1 };

// Motor Indices
const int LEFT_MOTORS[]  = { 1, 3 }; 
const int RIGHT_MOTORS[] = { 0, 2 }; 
const int MOTOR_COUNT = 4;

// ===== Hardware Constants =====
const float POLE_PAIRS = 4.0f;           // 86BLF has 8 poles = 4 pole pairs
const float GEAR_RATIO = 16.0f;          // 16:1 gearbox reduction ratio
const float DRIVER_CONSTANT = 20.0f;     // JKBLD300 specific constant
const float MAX_WHEEL_RPM = 3000.0f / GEAR_RATIO;  // ~187.5 RPM max

// ===== PID Configuration =====
struct PIDController {
  float kp;
  float ki;
  float kd;
  float setpoint;      // Target RPM
  float integral;      // Integral accumulator
  float lastError;     // Previous error for derivative
  float output;        // PID output (0-255 PWM)
  float feedForward;   // Feed-forward term
  unsigned long lastTime;
};

// Initial PID parameters
PIDController motorPID[MOTOR_COUNT] = {
  {0.8f, 0.5f, 0.05f, 0, 0, 0, 0, 0, 0},  // Motor 1 (Right Front)
  {0.8f, 0.5f, 0.05f, 0, 0, 0, 0, 0, 0},  // Motor 2 (Left Front)
  {0.8f, 0.5f, 0.05f, 0, 0, 0, 0, 0, 0},  // Motor 3 (Right Back)
  {0.8f, 0.5f, 0.05f, 0, 0, 0, 0, 0, 0}   // Motor 4 (Left Back)
};

// PID limits
const float INTEGRAL_LIMIT = 100.0f;
const int PWM_MIN = 0;
const int PWM_MAX = 255;

// Feed-forward gain
float feedForwardGain = 0.85f;

// ===== Speed Measurement Variables =====
volatile unsigned long speedPulseCounts[MOTOR_COUNT] = {0, 0, 0, 0};
unsigned long lastSpeedPulseCounts[MOTOR_COUNT] = {0, 0, 0, 0};
float currentRPM[MOTOR_COUNT] = {0, 0, 0, 0};
int currentDirection[MOTOR_COUNT] = {1, 1, 1, 1};

// ===== Timing =====
const unsigned long PID_UPDATE_INTERVAL = 50;    // 50ms = 20Hz
const unsigned long RPM_CALC_INTERVAL = 50;      // 50ms
const unsigned long PRINT_INTERVAL = 100;        // 100ms
unsigned long lastPIDUpdate = 0;
unsigned long lastRPMCalc = 0;
unsigned long lastPrint = 0;

// ===== Speed Control Variables =====
int forwardBackwardSpeed = 10;
int turningSpeed = 10;
const int FORWARD_BACKWARD_CAP = 25;
const int TURNING_CAP = 100;

// Control flags
bool pidEnabled = true;
bool continuousPrint = false;
bool csvMode = false;

// ===== ISR Functions (Motor Speed Feedback - INTERRUPT MODE) =====
void speedISR_Motor1() {
  speedPulseCounts[0]++;
}

void speedISR_Motor2() {
  speedPulseCounts[1]++;
}

void speedISR_Motor3() {
  speedPulseCounts[2]++;
}

void speedISR_Motor4() {
  speedPulseCounts[3]++;
}

// ===== Utility Functions =====
int limitDuty(int dutyPercent) {
  return constrain(dutyPercent, -100, 100);
}

float limitFloat(float value, float minVal, float maxVal) {
  return constrain(value, minVal, maxVal);
}

int limitPWM(float value) {
  return constrain((int)value, PWM_MIN, PWM_MAX);
}

// ===== Motor Control =====
void setMotorRaw(int index, int pwmValue) {
  if (index < 0 || index >= MOTOR_COUNT) return;
  
  pwmValue = constrain(pwmValue, PWM_MIN, PWM_MAX);
  
  bool forward = (motorPID[index].setpoint >= 0);
  currentDirection[index] = forward ? 1 : -1;
  
  if (MOTOR_INVERT[index]) forward = !forward;
  
  digitalWrite(DIR_PINS[index], forward ? HIGH : LOW);
  analogWrite(PWM_PINS[index], pwmValue);
}

// ===== RPM Calculation (JKBLD300 specific) =====
void calculateRPM() {
  unsigned long currentTime = millis();
  unsigned long deltaTime = currentTime - lastRPMCalc;
  
  if (deltaTime >= RPM_CALC_INTERVAL) {
    float dt_sec = deltaTime / 1000.0f;
    
    for (int i = 0; i < MOTOR_COUNT; i++) {
      // Read interrupt-safe pulse count
      noInterrupts();
      unsigned long currentCount = speedPulseCounts[i];
      interrupts();
      
      unsigned long deltaPulses = currentCount - lastSpeedPulseCounts[i];
      float frequency = (float)deltaPulses / dt_sec;
      
      // Motor RPM using JKBLD300 formula
      float motorRPM = (frequency / POLE_PAIRS) * DRIVER_CONSTANT;
      
      // Wheel RPM after gearbox
      float wheelRPM = motorRPM / GEAR_RATIO;
      
      // Apply direction sign
      currentRPM[i] = wheelRPM * currentDirection[i];
      
      lastSpeedPulseCounts[i] = currentCount;
    }
    lastRPMCalc = currentTime;
  }
}

// ===== PID Control with Feed-Forward =====
void updatePID() {
  unsigned long currentTime = millis();
  
  if (currentTime - lastPIDUpdate >= PID_UPDATE_INTERVAL) {
    float dt = (currentTime - lastPIDUpdate) / 1000.0f;
    
    for (int i = 0; i < MOTOR_COUNT; i++) {
      if (!pidEnabled) {
        // Direct control mode
        int pwm = map(abs((int)motorPID[i].setpoint), 0, 100, 0, 255);
        motorPID[i].output = pwm;
        motorPID[i].integral = 0;
        motorPID[i].lastError = 0;
        setMotorRaw(i, pwm);
        continue;
      }
      
      // Calculate error
      float error = motorPID[i].setpoint - currentRPM[i];
      
      // Proportional term
      float pTerm = motorPID[i].kp * error;
      
      // Integral term with anti-windup
      motorPID[i].integral += error * dt;
      motorPID[i].integral = limitFloat(motorPID[i].integral, -INTEGRAL_LIMIT, INTEGRAL_LIMIT);
      float iTerm = motorPID[i].ki * motorPID[i].integral;
      
      // Derivative term
      float dTerm = 0;
      if (dt > 0) {
        dTerm = motorPID[i].kd * (error - motorPID[i].lastError) / dt;
      }
      
      // PID output
      float pidResult = pTerm + iTerm + dTerm;
      
      // Feed-forward term
      motorPID[i].feedForward = abs(motorPID[i].setpoint) * feedForwardGain;
      
      // Combine PID + Feed-forward
      float totalOutput = pidResult + motorPID[i].feedForward;
      
      // Limit to PWM range
      int pwm = limitPWM(totalOutput);
      motorPID[i].output = pwm;
      
      // Store for next iteration
      motorPID[i].lastError = error;
      
      // Apply to motor
      setMotorRaw(i, pwm);
    }
    
    lastPIDUpdate = currentTime;
  }
}

// ===== High-Level Motion Functions =====
void setLeftMotorsPID(float targetRPM) {
  for (size_t i = 0; i < sizeof(LEFT_MOTORS)/sizeof(LEFT_MOTORS[0]); i++) {
    motorPID[LEFT_MOTORS[i]].setpoint = targetRPM;
  }
}

void setRightMotorsPID(float targetRPM) {
  for (size_t i = 0; i < sizeof(RIGHT_MOTORS)/sizeof(RIGHT_MOTORS[0]); i++) {
    motorPID[RIGHT_MOTORS[i]].setpoint = targetRPM;
  }
}

void setAllMotorsPID(float targetRPM) {
  for (int i = 0; i < MOTOR_COUNT; i++) {
    motorPID[i].setpoint = targetRPM;
  }
}

float percentToRPM(int percent) {
  return (percent / 100.0f) * MAX_WHEEL_RPM;
}

void forward(int speedPercent) {
  speedPercent = limitDuty(speedPercent);
  speedPercent = constrain(speedPercent, 0, forwardBackwardSpeed);
  speedPercent = min(speedPercent, FORWARD_BACKWARD_CAP);
  
  float targetRPM = percentToRPM(speedPercent);
  setLeftMotorsPID(targetRPM);
  setRightMotorsPID(targetRPM);
  
  if (!csvMode) {
    Serial.print("Forward: ");
    Serial.print(speedPercent);
    Serial.print("% (");
    Serial.print(targetRPM, 1);
    Serial.println(" RPM)");
  }
}

void backward(int speedPercent) {
  speedPercent = limitDuty(speedPercent);
  speedPercent = constrain(speedPercent, 0, forwardBackwardSpeed);
  speedPercent = min(speedPercent, FORWARD_BACKWARD_CAP);
  
  float targetRPM = -percentToRPM(speedPercent);
  setLeftMotorsPID(targetRPM);
  setRightMotorsPID(targetRPM);
  
  if (!csvMode) {
    Serial.print("Backward: ");
    Serial.print(speedPercent);
    Serial.print("% (");
    Serial.print(abs(targetRPM), 1);
    Serial.println(" RPM)");
  }
}

void turnRight(int speedPercent) {
  speedPercent = limitDuty(speedPercent);
  speedPercent = constrain(speedPercent, 0, turningSpeed);
  speedPercent = min(speedPercent, TURNING_CAP);
  
  float targetRPM = percentToRPM(speedPercent);
  setLeftMotorsPID(targetRPM);
  setRightMotorsPID(-targetRPM);
  
  if (!csvMode) {
    Serial.print("Turn Right: ");
    Serial.print(speedPercent);
    Serial.println("%");
  }
}

void turnLeft(int speedPercent) {
  speedPercent = limitDuty(speedPercent);
  speedPercent = constrain(speedPercent, 0, turningSpeed);
  speedPercent = min(speedPercent, TURNING_CAP);
  
  float targetRPM = percentToRPM(speedPercent);
  setLeftMotorsPID(-targetRPM);
  setRightMotorsPID(targetRPM);
  
  if (!csvMode) {
    Serial.print("Turn Left: ");
    Serial.print(speedPercent);
    Serial.println("%");
  }
}

void stopAll() {
  setAllMotorsPID(0);
  if (!csvMode) Serial.println("STOP");
}

void resetControl() {
  stopAll();
  forwardBackwardSpeed = 10;
  turningSpeed = 10;
  
  for (int i = 0; i < MOTOR_COUNT; i++) {
    motorPID[i].integral = 0;
    motorPID[i].lastError = 0;
  }
  
  if (!csvMode) Serial.println("Reset Complete");
}

// ===== Status Printing =====
void printStatus() {
  if (csvMode) {
    // CSV format for Python GUI
    float avgTarget = 0, avgActual = 0, avgPWM = 0;
    for (int i = 0; i < MOTOR_COUNT; i++) {
      avgTarget += motorPID[i].setpoint;
      avgActual += currentRPM[i];
      avgPWM += motorPID[i].output;
    }
    avgTarget /= MOTOR_COUNT;
    avgActual /= MOTOR_COUNT;
    avgPWM /= MOTOR_COUNT;
    
    Serial.print(abs(avgTarget), 1); Serial.print(",");
    Serial.print(abs(avgActual), 1); Serial.print(",");
    Serial.print(avgPWM, 1); Serial.print(",");
    Serial.print(motorPID[0].kp, 2); Serial.print(",");
    Serial.print(motorPID[0].ki, 2); Serial.print(",");
    Serial.println(motorPID[0].kd, 3);
  } else {
    Serial.println("\n=== Motor Status ===");
    Serial.print("PID: "); Serial.println(pidEnabled ? "ON" : "OFF");
    Serial.print("Gains - Kp:"); Serial.print(motorPID[0].kp, 2);
    Serial.print(" Ki:"); Serial.print(motorPID[0].ki, 2);
    Serial.print(" Kd:"); Serial.println(motorPID[0].kd, 3);
    Serial.print("FF Gain: "); Serial.println(feedForwardGain, 2);
    
    const char* names[] = {"RF", "LF", "RB", "LB"};
    for (int i = 0; i < MOTOR_COUNT; i++) {
      float error = motorPID[i].setpoint - currentRPM[i];
      Serial.print(names[i]);
      Serial.print(" | Tgt:"); Serial.print(motorPID[i].setpoint, 1);
      Serial.print(" Act:"); Serial.print(currentRPM[i], 1);
      Serial.print(" Err:"); Serial.print(error, 1);
      Serial.print(" PWM:"); Serial.print((int)motorPID[i].output);
      Serial.print(" Cnt:"); Serial.println(speedPulseCounts[i]);
    }
    Serial.println();
  }
}

// ===== Serial Command Processing =====
void processSerialCommand() {
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    input.trim();
    
    if (input.length() == 0) return;
    
    // Commands with parameters
    if (input.length() > 1) {
      char cmd = input.charAt(0);
      float val = input.substring(1).toFloat();
      
      switch(cmd) {
        case 'P':
          for (int i = 0; i < MOTOR_COUNT; i++) motorPID[i].kp = val;
          if (!csvMode) { Serial.print("Kp = "); Serial.println(val, 2); }
          return;
        case 'I':
          for (int i = 0; i < MOTOR_COUNT; i++) motorPID[i].ki = val;
          if (!csvMode) { Serial.print("Ki = "); Serial.println(val, 2); }
          return;
        case 'D':
          for (int i = 0; i < MOTOR_COUNT; i++) motorPID[i].kd = val;
          if (!csvMode) { Serial.print("Kd = "); Serial.println(val, 3); }
          return;
        case 'T':
          setAllMotorsPID(val);
          for (int i = 0; i < MOTOR_COUNT; i++) motorPID[i].integral = 0;
          if (!csvMode) { Serial.print("Target = "); Serial.print(val, 1); Serial.println(" RPM"); }
          return;
        case 'F':
          feedForwardGain = val;
          if (!csvMode) { Serial.print("FF Gain = "); Serial.println(val, 2); }
          return;
      }
    }
    
    // Single character commands
    char cmd = input.charAt(0);
    switch (cmd) {
      case 'w': case 'W':
        digitalWrite(LED_PIN, HIGH);
        forward(forwardBackwardSpeed);
        break;
      case 's': case 'S':
        digitalWrite(LED_PIN, HIGH);
        backward(forwardBackwardSpeed);
        break;
      case 'a': case 'A':
        digitalWrite(LED_PIN, HIGH);
        turnLeft(turningSpeed);
        break;
      case 'd': case 'D':
        digitalWrite(LED_PIN, HIGH);
        turnRight(turningSpeed);
        break;
      case 'x': case 'X': case ' ':
        digitalWrite(LED_PIN, LOW);
        stopAll();
        break;
      case 'h': case 'H':
        digitalWrite(LED_PIN, LOW);
        resetControl();
        break;
      case 'q': case 'Q':
        forwardBackwardSpeed = min(forwardBackwardSpeed + 5, FORWARD_BACKWARD_CAP);
        if (!csvMode) { Serial.print("Fwd/Back: "); Serial.println(forwardBackwardSpeed); }
        break;
      case 'z': case 'Z':
        forwardBackwardSpeed = max(forwardBackwardSpeed - 5, 5);
        if (!csvMode) { Serial.print("Fwd/Back: "); Serial.println(forwardBackwardSpeed); }
        break;
      case 'e': case 'E':
        turningSpeed = min(turningSpeed + 5, TURNING_CAP);
        if (!csvMode) { Serial.print("Turning: "); Serial.println(turningSpeed); }
        break;
      case 'c': case 'C':
        turningSpeed = max(turningSpeed - 5, 5);
        if (!csvMode) { Serial.print("Turning: "); Serial.println(turningSpeed); }
        break;
      case 'p': case 'P':
        pidEnabled = !pidEnabled;
        if (!csvMode) {
          Serial.print("PID: ");
          Serial.println(pidEnabled ? "ON" : "OFF");
        }
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
    }
  }
}

// ===== Setup =====
void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000);
  
  // Configure motor pins
  for (int i = 0; i < MOTOR_COUNT; i++) {
    pinMode(PWM_PINS[i], OUTPUT);
    pinMode(DIR_PINS[i], OUTPUT);
    analogWrite(PWM_PINS[i], 0);
    digitalWrite(DIR_PINS[i], LOW);
  }
  
  // Configure SPEED feedback pins with pull-ups
  pinMode(SPEED_FB_1, INPUT_PULLUP);
  pinMode(SPEED_FB_2, INPUT_PULLUP);
  pinMode(SPEED_FB_3, INPUT_PULLUP);
  pinMode(SPEED_FB_4, INPUT_PULLUP);
  
  // Attach interrupts (MEGA: Pin 21=INT0, 20=INT1, 19=INT2, 18=INT3)
  attachInterrupt(digitalPinToInterrupt(SPEED_FB_1), speedISR_Motor1, RISING);
  attachInterrupt(digitalPinToInterrupt(SPEED_FB_2), speedISR_Motor2, RISING);
  attachInterrupt(digitalPinToInterrupt(SPEED_FB_3), speedISR_Motor3, RISING);
  attachInterrupt(digitalPinToInterrupt(SPEED_FB_4), speedISR_Motor4, RISING);
  
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);
  
  lastPIDUpdate = millis();
  lastRPMCalc = millis();
  lastPrint = millis();
  
  Serial.println("\n=== Arduino MEGA 4WD PID Controller ===");
  Serial.println("Motor Config: JKBLD300 + 86BLF (8-pole)");
  Serial.println("Gearbox: 16:1 | Max RPM: ~188");
  Serial.println();
  Serial.println("=== Interrupt Configuration ===");
  Serial.println("M1 (RF): Pin 21 (INT0)");
  Serial.println("M2 (LF): Pin 20 (INT1)");
  Serial.println("M3 (RB): Pin 19 (INT2)");
  Serial.println("M4 (LB): Pin 18 (INT3)");
  Serial.println();
  Serial.println("=== Commands ===");
  Serial.println("Motion: W/S/A/D, Stop: X/Space, Reset: H");
  Serial.println("Speed: Q/Z (fwd/back), E/C (turn)");
  Serial.println("PID: P (toggle), M (status), V (continuous)");
  Serial.println("Tune: P<val> I<val> D<val> T<rpm> F<ff>");
  Serial.println("  P1.5  → Kp=1.5");
  Serial.println("  T100  → Target 100 RPM");
  Serial.println("CSV: G (toggle CSV mode)");
  Serial.println();
  Serial.println("=== Initial Settings ===");
  Serial.print("Kp:"); Serial.print(motorPID[0].kp, 2);
  Serial.print(" Ki:"); Serial.print(motorPID[0].ki, 2);
  Serial.print(" Kd:"); Serial.println(motorPID[0].kd, 3);
  Serial.println("\nReady!\n");
}

// ===== Main Loop =====
void loop() {
  calculateRPM();
  updatePID();
  processSerialCommand();
  
  if (continuousPrint && (millis() - lastPrint >= PRINT_INTERVAL)) {
    printStatus();
    lastPrint = millis();
  }
}