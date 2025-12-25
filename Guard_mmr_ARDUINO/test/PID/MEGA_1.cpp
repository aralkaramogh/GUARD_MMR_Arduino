/**
 * ============================================================================
 * ARDUINO MEGA - 4WD AMR PID CONTROLLER
 * ============================================================================
 * 
 * Compatible with: Arduino Mega
 * 
 * Hardware:
 * - Arduino Mega
 * - 4x JKBLD300 V2 drivers
 * - 4x 86BLF BLDC motors
 * - 16:1 Gearbox
 * 
 * Wiring:
 * ┌──────────────┬─────┬─────┬───────┬───────────────┐
 * │ Motor        │ PWM │ DIR │ SPEED │ Notes         │
 * ├──────────────┼─────┼─────┼───────┼───────────────┤
 * │ Right Front  │ 3   │ 4   │ 50    │               │
 * │ Left Front   │ 5   │ 7   │ 51    │               │
 * │ Right Back   │ 6   │ 8   │ 52    │               │
 * │ Left Back    │ 9   │ 12  │ 53    │               │
 * └──────────────┴─────┴─────┴───────┴───────────────┘
 * 
 * Commands: W/S/A/D (move), X (stop), M (status), P/I/D/T/F (tune), G (CSV)
 * 
 * ============================================================================
 */

#include <Arduino.h>

// ===== Pin Configuration (UNCHANGED from Uno for PWM/DIR/SPEED) =====
const int LED_PIN = 13;

// PWM Pins
const int MOTOR_PWM_1 = 3;  // Right Front
const int MOTOR_PWM_2 = 5;  // Left Front
const int MOTOR_PWM_3 = 6;  // Right Back
const int MOTOR_PWM_4 = 9;  // Left Back

// Direction Pins
const int MOTOR_DIR_1 = 4;
const int MOTOR_DIR_2 = 7;
const int MOTOR_DIR_3 = 8;
const int MOTOR_DIR_4 = 12;

// SPEED Feedback Pins
const int SPEED_FB_1 = 50;  // Right Front
const int SPEED_FB_2 = 51;  // Left Front
const int SPEED_FB_3 = 52;  // Right Back  
const int SPEED_FB_4 = 53;  // Left Back

// Grouping Arrays
const int PWM_PINS[] = { MOTOR_PWM_1, MOTOR_PWM_2, MOTOR_PWM_3, MOTOR_PWM_4 };
const int DIR_PINS[] = { MOTOR_DIR_1, MOTOR_DIR_2, MOTOR_DIR_3, MOTOR_DIR_4 };
const int SPEED_FB_PINS[] = { SPEED_FB_1, SPEED_FB_2, SPEED_FB_3, SPEED_FB_4 };

// Motor configuration
const int MOTOR_INVERT[] = { 0, 1, 0, 1 };
const int LEFT_MOTORS[]  = { 1, 3 }; 
const int RIGHT_MOTORS[] = { 0, 2 }; 
const int MOTOR_COUNT = 4;

// ===== Hardware Constants =====
const float POLE_PAIRS = 4.0f;
const float GEAR_RATIO = 16.0f;
const float DRIVER_CONSTANT = 20.0f;
const float MAX_MOTOR_RPM = 3000.0f;
const float MAX_WHEEL_RPM = MAX_MOTOR_RPM / GEAR_RATIO;

// ===== PID Configuration =====
struct PIDController {
  float kp, ki, kd;
  float setpoint;
  float integral;
  float lastError;
  float output;
  float feedForward;
  unsigned long lastTime;
};

PIDController motorPID[MOTOR_COUNT] = {
  {0.8f, 0.5f, 0.05f, 0, 0, 0, 0, 0, 0},
  {0.8f, 0.5f, 0.05f, 0, 0, 0, 0, 0, 0},
  {0.8f, 0.5f, 0.05f, 0, 0, 0, 0, 0, 0},
  {0.8f, 0.5f, 0.05f, 0, 0, 0, 0, 0, 0}
};

const float INTEGRAL_LIMIT = 100.0f;
const int PWM_MIN = 0;
const int PWM_MAX = 255;
float feedForwardGain = 0.85f;

// ===== Speed Measurement =====
volatile unsigned long speedPulseCounts[MOTOR_COUNT] = {0, 0, 0, 0};
unsigned long lastSpeedPulseCounts[MOTOR_COUNT] = {0, 0, 0, 0};
float currentRPM[MOTOR_COUNT] = {0, 0, 0, 0};
int currentDirection[MOTOR_COUNT] = {1, 1, 1, 1};

// ===== Timing =====
const unsigned long PID_UPDATE_INTERVAL = 50;
const unsigned long RPM_CALC_INTERVAL = 50;
const unsigned long PRINT_INTERVAL = 100;
unsigned long lastPIDUpdate = 0;
unsigned long lastRPMCalc = 0;
unsigned long lastPrint = 0;

// ===== Control Variables =====
int forwardBackwardSpeed = 10;
int turningSpeed = 10;
const int FORWARD_BACKWARD_CAP = 25;
const int TURNING_CAP = 100;

bool pidEnabled = true;
bool continuousPrint = false;
bool csvMode = false;

// ===== Speed Feedback ISR =====
void speedISR_0() {
  speedPulseCounts[0]++;
}

// ===== Utility Functions =====
int limitPWM(float value) {
  return constrain((int)value, PWM_MIN, PWM_MAX);
}

float limitFloat(float value, float minVal, float maxVal) {
  return constrain(value, minVal, maxVal);
}

// ===== Motor Control =====
void setMotorRaw(int index, int pwmValue) {
  if (index < 0 || index >= MOTOR_COUNT) return;
  
  pwmValue = limitPWM(pwmValue);
  
  if (pwmValue == 0) {
    // Stop this motor
    analogWrite(PWM_PINS[index], 0);
    return;
  }

  // Set direction
  bool forward = (motorPID[index].setpoint >= 0);
  currentDirection[index] = forward ? 1 : -1;
  
  if (MOTOR_INVERT[index]) forward = !forward;
  
  digitalWrite(DIR_PINS[index], forward ? HIGH : LOW);
  analogWrite(PWM_PINS[index], pwmValue);
}

// ===== RPM Calculation =====
void calculateRPM() {
  unsigned long now = millis();
  unsigned long dt = now - lastRPMCalc;
  
  if (dt >= RPM_CALC_INTERVAL) {
    float dt_sec = dt / 1000.0f;
    
    for (int i = 0; i < MOTOR_COUNT; i++) {
      unsigned long pulses = speedPulseCounts[i] - lastSpeedPulseCounts[i];
      float freq = (float)pulses / dt_sec;
      float motorRPM = (freq / POLE_PAIRS) * DRIVER_CONSTANT;
      float wheelRPM = motorRPM / GEAR_RATIO;
      currentRPM[i] = wheelRPM * currentDirection[i];
      lastSpeedPulseCounts[i] = speedPulseCounts[i];
    }
    lastRPMCalc = now;
  }
}

// ===== PID Control =====
void updatePID() {
  unsigned long now = millis();
  
  if (now - lastPIDUpdate >= PID_UPDATE_INTERVAL) {
    float dt = (now - lastPIDUpdate) / 1000.0f;
    
    for (int i = 0; i < MOTOR_COUNT; i++) {
      if (!pidEnabled) {
        int pwm = map(abs((int)motorPID[i].setpoint), 0, 100, 0, 255);
        motorPID[i].output = pwm;
        motorPID[i].integral = 0;
        motorPID[i].lastError = 0;
        setMotorRaw(i, pwm);
        continue;
      }
      
      float error = motorPID[i].setpoint - currentRPM[i];
      
      float pTerm = motorPID[i].kp * error;
      
      motorPID[i].integral += error * dt;
      motorPID[i].integral = limitFloat(motorPID[i].integral, -INTEGRAL_LIMIT, INTEGRAL_LIMIT);
      float iTerm = motorPID[i].ki * motorPID[i].integral;
      
      float dTerm = 0;
      if (dt > 0) {
        dTerm = motorPID[i].kd * (error - motorPID[i].lastError) / dt;
      }
      
      float pidResult = pTerm + iTerm + dTerm;
      motorPID[i].feedForward = abs(motorPID[i].setpoint) * feedForwardGain;
      float totalOutput = pidResult + motorPID[i].feedForward;
      
      int pwm = limitPWM(totalOutput);
      motorPID[i].output = pwm;
      motorPID[i].lastError = error;
      
      setMotorRaw(i, pwm);
    }
    
    lastPIDUpdate = now;
  }
}

// ===== Speed Feedback Polling =====
byte lastSpeedState[MOTOR_COUNT] = {0, 0, 0, 0};

void pollSpeedFeedback() {
  for (int i = 1; i < MOTOR_COUNT; i++) {
    byte state = digitalRead(SPEED_FB_PINS[i]);
    if (state == HIGH && lastSpeedState[i] == LOW) {
      speedPulseCounts[i]++;
    }
    lastSpeedState[i] = state;
  }
}

// ===== Motion Functions =====
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
  speedPercent = constrain(speedPercent, 0, forwardBackwardSpeed);
  speedPercent = min(speedPercent, FORWARD_BACKWARD_CAP);
  
  float targetRPM = percentToRPM(speedPercent);
  setLeftMotorsPID(targetRPM);
  setRightMotorsPID(targetRPM);
  
  if (!csvMode) {
    Serial.print("Forward: "); Serial.print(speedPercent);
    Serial.print("% ("); Serial.print(targetRPM, 0); Serial.println(" RPM)");
  }
}

void backward(int speedPercent) {
  speedPercent = constrain(speedPercent, 0, forwardBackwardSpeed);
  speedPercent = min(speedPercent, FORWARD_BACKWARD_CAP);
  
  float targetRPM = -percentToRPM(speedPercent);
  setLeftMotorsPID(targetRPM);
  setRightMotorsPID(targetRPM);
  
  if (!csvMode) {
    Serial.print("Backward: "); Serial.print(speedPercent);
    Serial.print("% ("); Serial.print(abs(targetRPM), 0); Serial.println(" RPM)");
  }
}

void turnRight(int speedPercent) {
  speedPercent = constrain(speedPercent, 0, turningSpeed);
  speedPercent = min(speedPercent, TURNING_CAP);
  
  float targetRPM = percentToRPM(speedPercent);
  setLeftMotorsPID(targetRPM);
  setRightMotorsPID(-targetRPM);
  
  if (!csvMode) {
    Serial.print("Turn Right: "); Serial.print(speedPercent); Serial.println("%");
  }
}

void turnLeft(int speedPercent) {
  speedPercent = constrain(speedPercent, 0, turningSpeed);
  speedPercent = min(speedPercent, TURNING_CAP);
  
  float targetRPM = percentToRPM(speedPercent);
  setLeftMotorsPID(-targetRPM);
  setRightMotorsPID(targetRPM);
  
  if (!csvMode) {
    Serial.print("Turn Left: "); Serial.print(speedPercent); Serial.println("%");
  }
}

void stopAll() {
  setAllMotorsPID(0);
  
  // Stop all motors
  for (int i = 0; i < MOTOR_COUNT; i++) {
    analogWrite(PWM_PINS[i], 0);
  }
  
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
  
  if (!csvMode) Serial.println("Reset complete");
}

// ===== Status Printing =====
void printStatus() {
  if (csvMode) {
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
    
    const char* names[] = {"RF", "LF", "RB", "LB"};
    for (int i = 0; i < MOTOR_COUNT; i++) {
      Serial.print(names[i]);
      Serial.print(" | Tgt:"); Serial.print(motorPID[i].setpoint, 1);
      Serial.print(" Act:"); Serial.print(currentRPM[i], 1);
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
    
    if (input.length() > 1) {
      char cmd = input.charAt(0);
      float val = input.substring(1).toFloat();
      
      switch(cmd) {
        case 'P': case 'p':
          for (int i = 0; i < MOTOR_COUNT; i++) motorPID[i].kp = val;
          if (!csvMode) { Serial.print("Kp = "); Serial.println(val, 2); }
          return;
        case 'I': case 'i':
          for (int i = 0; i < MOTOR_COUNT; i++) motorPID[i].ki = val;
          if (!csvMode) { Serial.print("Ki = "); Serial.println(val, 2); }
          return;
        case 'D': case 'd':
          for (int i = 0; i < MOTOR_COUNT; i++) motorPID[i].kd = val;
          if (!csvMode) { Serial.print("Kd = "); Serial.println(val, 3); }
          return;
        case 'T': case 't':
          setAllMotorsPID(val);
          for (int i = 0; i < MOTOR_COUNT; i++) motorPID[i].integral = 0;
          if (!csvMode) { Serial.print("Target = "); Serial.print(val, 1); Serial.println(" RPM"); }
          return;
        case 'F': case 'f':
          feedForwardGain = val;
          if (!csvMode) { Serial.print("FF = "); Serial.println(val, 2); }
          return;
      }
    }
    
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
          Serial.print("Continuous: ");
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
  while (!Serial && millis() < 2000);
  
  // Configure motor pins
  for (int i = 0; i < MOTOR_COUNT; i++) {
    pinMode(PWM_PINS[i], OUTPUT);
    pinMode(DIR_PINS[i], OUTPUT);
    analogWrite(PWM_PINS[i], 0);
    digitalWrite(DIR_PINS[i], LOW);
  }
  
  // Configure SPEED pins
  for (int i = 0; i < MOTOR_COUNT; i++) {
    pinMode(SPEED_FB_PINS[i], INPUT);
  }
  
  attachInterrupt(digitalPinToInterrupt(SPEED_FB_1), speedISR_0, RISING);
  
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);
  
  lastPIDUpdate = millis();
  lastRPMCalc = millis();
  lastPrint = millis();
  
  Serial.println("\n=== Arduino Mega - 4WD PID Controller ===");
  Serial.println("Platform: Arduino Mega 2560");
  Serial.println("\nCommands: W/S/A/D (move), X (stop), M (status)");
  Serial.println("          P/I/D/T/F (tune), G (CSV mode)");
  Serial.print("\nInitial PID: Kp="); Serial.print(motorPID[0].kp, 2);
  Serial.print(" Ki="); Serial.print(motorPID[0].ki, 2);
  Serial.print(" Kd="); Serial.println(motorPID[0].kd, 3);
  Serial.println("Ready!\n");
}

// ===== Main Loop =====
void loop() {
  pollSpeedFeedback();
  calculateRPM();
  updatePID();
  processSerialCommand();
  
  if (continuousPrint && (millis() - lastPrint >= PRINT_INTERVAL)) {
    printStatus();
    lastPrint = millis();
  }
}