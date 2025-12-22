/**
 * ============================================================================
 * PID-CONTROLLED 4WD AMR MOTOR CONTROLLER
 * ============================================================================
 * 
 * Hardware Configuration:
 * - MCU: Arduino Uno (5V TTL logic - compatible with JKBLD300)
 * - Motor Drivers: 4x JKBLD300 V2
 * - Motors: 4x 86BLF series BLDC (8-pole, rated 3000 RPM)
 * - Gearbox: 16:1 reduction ratio
 * - Drive Type: Skid-steer (differential steering)
 * 
 * ============================================================================
 * WIRING CONNECTIONS
 * ============================================================================
 * 
 * Motor Control Pins:
 * ┌─────────────┬──────────┬──────────┬────────────────────────┐
 * │ Motor       │ PWM Pin  │ DIR Pin  │ SPEED Feedback Pin     │
 * ├─────────────┼──────────┼──────────┼────────────────────────┤
 * │ Right Front │ Pin 3    │ Pin 4    │ Pin 2 (INT0 - hardware)│
 * │ Left Front  │ Pin 5    │ Pin 7    │ Pin A0 (polled)        │
 * │ Right Back  │ Pin 6    │ Pin 8    │ Pin A1 (polled)        │
 * │ Left Back   │ Pin 9    │ Pin 12   │ Pin A2 (polled)        │
 * └─────────────┴──────────┴──────────┴────────────────────────┘
 * 
 * JKBLD300 Driver Connections (per motor):
 * - Arduino PWM pin → JKBLD300 PWM input
 * - Arduino DIR pin → JKBLD300 DIR input
 * - JKBLD300 SPEED output → Arduino feedback pin
 * - JKBLD300 GND → Arduino GND (common ground essential!)
 * 
 * Important Notes:
 * - Arduino Uno uses 5V TTL logic (fully compatible with JKBLD300)
 * - STM32 uses 3.3V CMOS (requires level shifters with JKBLD300)
 * - SPEED output from JKBLD300 is TTL compatible (5V)
 * 
 * ============================================================================
 * SPEED FEEDBACK CALCULATION
 * ============================================================================
 * 
 * The JKBLD300 SPEED pin outputs pulses proportional to motor speed.
 * 
 * Formula (from your STM32 implementation):
 *   1. Measure pulse frequency (Hz)
 *   2. Motor RPM = (frequency / POLE_PAIRS) × DRIVER_CONSTANT
 *   3. Wheel RPM = Motor RPM / GEAR_RATIO
 * 
 * Constants:
 * - POLE_PAIRS = 4 (86BLF has 8 poles)
 * - DRIVER_CONSTANT = 20.0 (JKBLD300 specific)
 * - GEAR_RATIO = 16.0 (your gearbox)
 * 
 * Example at 3000 motor RPM:
 * - Frequency = 3000 × (4/60) / 20 = 10 Hz (approx)
 * - Wheel RPM = 3000 / 16 = 187.5 RPM
 * 
 * ============================================================================
 * PID CONTROL ARCHITECTURE
 * ============================================================================
 * 
 * Each motor has independent PID controller with feed-forward:
 * 
 *   output = Kp×error + Ki×∫error×dt + Kd×(d/dt)error + FF×setpoint
 * 
 * Default Gains (from STM32 implementation):
 * - Kp = 0.8   (Proportional - main correction force)
 * - Ki = 0.5   (Integral - eliminates steady-state error)
 * - Kd = 0.05  (Derivative - dampens oscillations)
 * - FF = 0.85  (Feed-forward - immediate response)
 * 
 * Update Rate: 20 Hz (50ms interval)
 * RPM Calculation: 20 Hz (50ms interval)
 * 
 * ============================================================================
 * SERIAL COMMANDS - MANUAL CONTROL
 * ============================================================================
 * 
 * Movement Commands:
 *   W or F    → Forward at current speed setting
 *   S or B    → Backward at current speed setting
 *   A or L    → Turn left (rotate counter-clockwise)
 *   D or R    → Turn right (rotate clockwise)
 *   X/Space   → Emergency stop (all motors)
 *   H         → Reset (stop + restore default speeds)
 * 
 * Speed Adjustment:
 *   Q         → Increase forward/backward speed (+5%)
 *   Z         → Decrease forward/backward speed (-5%)
 *   E         → Increase turning speed (+5%)
 *   C         → Decrease turning speed (-5%)
 * 
 * PID Control:
 *   P         → Toggle PID on/off (off = direct PWM control)
 *   M         → Print current motor status (once)
 *   V         → Toggle continuous status printing (100ms)
 *   G         → Toggle CSV mode (for Python GUI)
 * 
 * ============================================================================
 * SERIAL COMMANDS - PID TUNING
 * ============================================================================
 * 
 * Send these commands via Serial Monitor:
 * 
 * Format: <command><value>
 * 
 * PID Gains:
 *   P1.5      → Set Kp = 1.5 (all motors)
 *   I0.5      → Set Ki = 0.5 (all motors)
 *   D0.05     → Set Kd = 0.05 (all motors)
 *   F0.85     → Set feed-forward gain = 0.85
 * 
 * Target Speed:
 *   T150      → Set target to 150 RPM (all motors)
 *   T0        → Stop all motors (target = 0)
 * 
 * Examples:
 *   P2.0      → Try higher proportional gain
 *   I0.3      → Reduce integral to prevent overshoot
 *   T100      → Test at 100 wheel RPM
 * 
 * ============================================================================
 * PYTHON GUI TUNING (RECOMMENDED FOR PID TUNING)
 * ============================================================================
 * 
 * Step 1: Enable CSV Mode
 *   - Upload this code to Arduino
 *   - Open Serial Monitor
 *   - Press 'G' to enable CSV mode
 *   - Close Serial Monitor (important!)
 * 
 * Step 2: Run Python Tuner
 *   - Run: python python_PID_tuner.py
 *   - Select Arduino COM port
 *   - Click "Connect"
 * 
 * Step 3: Real-Time Tuning
 *   - Use sliders to adjust Kp, Ki, Kd
 *   - Set target RPM
 *   - Watch real-time response graph
 *   - CSV format: Target,Actual,PWM,Kp,Ki,Kd
 * 
 * Step 4: Save Good Values
 *   - Once tuned, note down the values
 *   - Update the PID initial values in code
 *   - Press 'G' again to disable CSV mode
 * 
 * ============================================================================
 * PID TUNING GUIDE
 * ============================================================================
 * 
 * Ziegler-Nichols Method (Quick Start):
 * 
 * Step 1: Find Critical Gain (Kp only)
 *   P0      I0      D0      (disable I and D)
 *   P0.5    T100    (start small)
 *   P1.0    (increase until oscillation)
 *   P2.0    (keep increasing)
 *   → Note the Kp value where motor oscillates steadily = Ku
 * 
 * Step 2: Calculate Initial Values
 *   Kp = 0.6 × Ku
 *   Ki = 1.2 × Ku / Tu  (Tu = oscillation period in seconds)
 *   Kd = 0.075 × Ku × Tu
 * 
 * Step 3: Fine Tuning
 *   Too much overshoot?     → Reduce Kp, increase Kd
 *   Slow to reach target?   → Increase Kp
 *   Steady-state error?     → Increase Ki
 *   Oscillating?            → Reduce Kp and Ki
 *   Noisy/jittery?          → Reduce Kd
 * 
 * Feed-Forward Tuning:
 *   - Measure max RPM at 255 PWM (e.g., 300 RPM)
 *   - FF gain = 255 / max_RPM
 *   - Example: 255 / 300 = 0.85
 *   - Adjust with F command (e.g., F0.9)
 * 
 * ============================================================================
 * OPERATIONAL MODES
 * ============================================================================
 * 
 * Mode 1: PID Control (Default, pidEnabled = true)
 *   - Closed-loop velocity control
 *   - Motors maintain constant RPM despite load changes
 *   - Best for autonomous navigation
 *   - Use for: line following, odometry, path tracking
 * 
 * Mode 2: Direct PWM Control (pidEnabled = false)
 *   - Open-loop control (legacy mode)
 *   - Speed varies with load and battery voltage
 *   - Toggle with 'P' command
 *   - Use for: manual testing, debugging
 * 
 * ============================================================================
 * TROUBLESHOOTING
 * ============================================================================
 * 
 * Problem: No SPEED pulses detected
 *   → Check JKBLD300 SPEED output wiring
 *   → Verify common ground between Arduino and drivers
 *   → Test with manual PWM (analogWrite test)
 *   → Check if motors are actually spinning
 * 
 * Problem: RPM reading is wrong
 *   → Verify POLE_PAIRS = 4 for 86BLF motors
 *   → Verify DRIVER_CONSTANT = 20.0 for JKBLD300
 *   → Verify GEAR_RATIO = 16.0 for your gearbox
 *   → Check pulse counting (print speedPulseCounts)
 * 
 * Problem: Motor oscillates/unstable
 *   → Reduce Kp (too aggressive)
 *   → Reduce Ki (integral windup)
 *   → Increase Kd (add damping)
 *   → Check mechanical issues (friction, binding)
 * 
 * Problem: Slow response
 *   → Increase Kp
 *   → Increase feed-forward gain
 *   → Check if PID is actually enabled ('P' to toggle)
 * 
 * Problem: Motors don't reach target
 *   → Increase Ki (steady-state error)
 *   → Check PWM saturation (output = 255?)
 *   → Verify target RPM is achievable
 *   → Check battery voltage
 * 
 * Problem: One motor behaves differently
 *   → Check MOTOR_INVERT array (physical wiring)
 *   → Verify that motor's SPEED feedback is working
 *   → Check driver connections
 *   → Motors may need individual tuning
 * 
 * ============================================================================
 * SAFETY FEATURES
 * ============================================================================
 * 
 * - Forward/Backward speed capped at 25% (FORWARD_BACKWARD_CAP)
 * - Turning speed capped at 100% (TURNING_CAP)
 * - PWM output clamped to 0-255 range
 * - Integral anti-windup (prevents runaway)
 * - Emergency stop with 'X' or Space
 * - All motors stop on reset ('H')
 * 
 * To change safety limits, edit these constants:
 *   const int FORWARD_BACKWARD_CAP = 25;  // max forward/back %
 *   const int TURNING_CAP = 100;          // max turning %
 * 
 * ============================================================================
 * PERFORMANCE SPECIFICATIONS
 * ============================================================================
 * 
 * Motor Specifications (86BLF):
 * - Rated voltage: 48V DC
 * - Rated speed: 3000 RPM (motor shaft)
 * - Poles: 8 (4 pole pairs)
 * - Torque: varies by model (0.35 - 2.1 Nm)
 * 
 * With 16:1 Gearbox:
 * - Max wheel RPM: 3000 / 16 = 187.5 RPM
 * - Max wheel speed depends on wheel diameter
 * - Example: 100mm diameter wheel
 *   → Max speed = 187.5 × π × 0.1 × 60 / 1000 = 3.53 m/min
 *   → Max speed ≈ 0.059 m/s = 5.9 cm/s
 * 
 * Control Loop Performance:
 * - PID update rate: 20 Hz (50ms)
 * - RPM calculation: 20 Hz (50ms)
 * - Response time: typically < 500ms to target
 * - Settling time: < 1 second (with proper tuning)
 * 
 * ============================================================================
 * VERSION HISTORY
 * ============================================================================
 * 
 * Based on STM32 implementation (PID_Gemini_1_Mtr.cpp)
 * Adapted for Arduino Uno with 4-motor skid-steer configuration
 * 
 * Key differences from STM32 version:
 * - Arduino Uno: 5V TTL (no level shifters needed)
 * - STM32: 3.3V CMOS (requires level shifters)
 * - Added 4-motor synchronized control
 * - Added skid-steer motion primitives
 * - Added Python GUI compatibility
 * - Added comprehensive documentation
 * 
 * ============================================================================
 */

#include <Arduino.h>

// ===== Pin Configuration =====
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

// SPEED Feedback Pins from JKBLD300 drivers
const int SPEED_FB_1 = 2;   // Right Front (INT0 - hardware interrupt)
const int SPEED_FB_2 = A0;  // Left Front
const int SPEED_FB_3 = A1;  // Right Back  
const int SPEED_FB_4 = A2;  // Left Back

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
const float MAX_WHEEL_RPM = 3000.0f / GEAR_RATIO;  // Wheel RPM after gearbox

// ===== PID Configuration =====
struct PIDController {
  // PID Gains
  float kp;
  float ki;
  float kd;
  
  // Control variables
  float setpoint;      // Target RPM
  float integral;      // Integral accumulator
  float lastError;     // Previous error for derivative
  float output;        // PID output (0-255 PWM)
  
  // Feed-forward
  float feedForward;   // Feed-forward term
  
  // Timing
  unsigned long lastTime;
};

// Initial PID parameters (from STM32 implementation)
PIDController motorPID[MOTOR_COUNT] = {
  {0.8f, 0.5f, 0.05f, 0, 0, 0, 0, 0, 0},  // Motor 0 (Right Front)
  {0.8f, 0.5f, 0.05f, 0, 0, 0, 0, 0, 0},  // Motor 1 (Left Front)
  {0.8f, 0.5f, 0.05f, 0, 0, 0, 0, 0, 0},  // Motor 2 (Right Back)
  {0.8f, 0.5f, 0.05f, 0, 0, 0, 0, 0, 0}   // Motor 3 (Left Back)
};

// PID limits
const float INTEGRAL_LIMIT = 100.0f;
const int PWM_MIN = 0;
const int PWM_MAX = 255;

// Feed-forward gain: PWM needed per RPM
// If 255 PWM gives ~300 RPM, then gain = 255/300 = 0.85
float feedForwardGain = 0.85f;

// ===== Speed Measurement Variables =====
volatile unsigned long speedPulseCounts[MOTOR_COUNT] = {0, 0, 0, 0};
unsigned long lastSpeedPulseCounts[MOTOR_COUNT] = {0, 0, 0, 0};
float currentRPM[MOTOR_COUNT] = {0, 0, 0, 0};
int currentDirection[MOTOR_COUNT] = {1, 1, 1, 1};  // 1 = forward, -1 = backward

// ===== Timing =====
const unsigned long PID_UPDATE_INTERVAL = 50;    // 50ms = 20Hz PID loop
const unsigned long RPM_CALC_INTERVAL = 50;      // 50ms for faster response
const unsigned long PRINT_INTERVAL = 100;        // 100ms for serial output
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
bool csvMode = false;  // For Python GUI compatibility

// ===== Speed Feedback ISR =====
void speedISR_0() {
  speedPulseCounts[0]++;
}

// ===== Utility Functions =====
int limitDuty(int dutyPercent) {
  return constrain(dutyPercent, -100, 100);
}

float limitFloat(float value, float minVal, float maxVal) {
  if (value > maxVal) return maxVal;
  if (value < minVal) return minVal;
  return value;
}

int limitPWM(float value) {
  if (value > PWM_MAX) return PWM_MAX;
  if (value < PWM_MIN) return PWM_MIN;
  return (int)value;
}

// ===== Motor Control Functions =====
void setMotorRaw(int index, int pwmValue) {
  if (index < 0 || index >= MOTOR_COUNT) return;
  
  pwmValue = constrain(pwmValue, PWM_MIN, PWM_MAX);
  
  // Determine direction from setpoint sign
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
      unsigned long deltaPulses = speedPulseCounts[i] - lastSpeedPulseCounts[i];
      
      // Calculate frequency (Hz)
      float frequency = (float)deltaPulses / dt_sec;
      
      // Motor RPM using JKBLD300 formula
      // motorRPM = (freq / POLE_PAIRS) * DRIVER_CONSTANT
      float motorRPM = (frequency / POLE_PAIRS) * DRIVER_CONSTANT;
      
      // Wheel RPM (after gearbox reduction)
      float wheelRPM = motorRPM / GEAR_RATIO;
      
      // Apply direction sign
      currentRPM[i] = wheelRPM * currentDirection[i];
      
      lastSpeedPulseCounts[i] = speedPulseCounts[i];
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
        // Direct control mode - convert percentage to PWM
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
      
      // Feed-forward term (helps motor start immediately)
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

// ===== Speed Feedback Polling =====
byte lastSpeedState[MOTOR_COUNT] = {0, 0, 0, 0};

void pollSpeedFeedback() {
  // Poll SPEED pins 1, 2, 3 (Motor 0 uses hardware interrupt)
  for (int i = 1; i < MOTOR_COUNT; i++) {
    byte currentState = digitalRead(SPEED_FB_PINS[i]);
    if (currentState == HIGH && lastSpeedState[i] == LOW) {
      speedPulseCounts[i]++;
    }
    lastSpeedState[i] = currentState;
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
    // CSV format for Python GUI: Target,Actual,PWM,Kp,Ki,Kd
    // Average all motors for simplicity
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
    
    // Python GUI commands (format: P1.5, I0.5, D0.05, T150)
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
  
  // Configure SPEED feedback pins
  for (int i = 0; i < MOTOR_COUNT; i++) {
    pinMode(SPEED_FB_PINS[i], INPUT);
  }
  
  // Attach interrupt for Motor 0
  attachInterrupt(digitalPinToInterrupt(SPEED_FB_1), speedISR_0, RISING);
  
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);
  
  lastPIDUpdate = millis();
  lastRPMCalc = millis();
  lastPrint = millis();
  
  Serial.println("\n=== Arduino Uno BLDC PID Controller ===");
  Serial.println("Based on STM32 implementation");
  Serial.println("JKBLD300 + 86BLF Motors (TTL Compatible)");
  Serial.println("\n=== Commands ===");
  Serial.println("Motion: W/S/A/D, Stop: X/Space, Reset: H");
  Serial.println("Speed: Q/Z (fwd/back), E/C (turn)");
  Serial.println("PID: P (toggle), M (status), V (continuous)");
  Serial.println("Tune: P<val> I<val> D<val> T<rpm> F<ff>");
  Serial.println("  Example: P1.5  (sets Kp=1.5)");
  Serial.println("  Example: T150  (sets target 150 RPM)");
  Serial.println("GUI: G (toggle CSV mode for Python tuner)");
  Serial.println("\n=== Initial Settings ===");
  Serial.print("Kp:"); Serial.print(motorPID[0].kp, 2);
  Serial.print(" Ki:"); Serial.print(motorPID[0].ki, 2);
  Serial.print(" Kd:"); Serial.println(motorPID[0].kd, 3);
  Serial.print("Gear Ratio: "); Serial.println(GEAR_RATIO, 1);
  Serial.print("Max Wheel RPM: "); Serial.println((int)MAX_WHEEL_RPM);
  Serial.println("\nReady!\n");
}

// ===== Main Loop =====
void loop() {
  pollSpeedFeedback();
  calculateRPM();
  updatePID();
  processSerialCommand();
  
  // Continuous status printing
  if (continuousPrint && (millis() - lastPrint >= PRINT_INTERVAL)) {
    printStatus();
    lastPrint = millis();
  }
}