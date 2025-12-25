/**
 * Skid-steer motor controller for Arduino Uno + JKBLD300 drivers.
 * UPDATED: Removed Active Braking Logic.
 *
 * Overview of key responsibilities and functions:
 * - Pin mapping: PWM and DIR pins for 4 motors.
 * - Logic: Moving = PWM > 0, Stopping = PWM 0 (Coast).
 */

#include <Arduino.h>

// ===== Pin Configuration =====
const int LED_PIN = 13; // Built-in LED on Uno 

// PWM Pins (Must be ~ pins: 3, 5, 6, 9, 10, 11)
const int MOTOR_PWM_1 = 3;  // Right Front
const int MOTOR_PWM_2 = 5;  // Left Front
const int MOTOR_PWM_3 = 6;  // Right Back
const int MOTOR_PWM_4 = 9;  // Left Back

// Direction Pins (Any Digital Pin)
const int MOTOR_DIR_1 = 4;
const int MOTOR_DIR_2 = 7;
const int MOTOR_DIR_3 = 8;
const int MOTOR_DIR_4 = 12;

// Grouping Arrays
const int PWM_PINS[] = { MOTOR_PWM_1, MOTOR_PWM_2, MOTOR_PWM_3, MOTOR_PWM_4 };
const int DIR_PINS[] = { MOTOR_DIR_1, MOTOR_DIR_2, MOTOR_DIR_3, MOTOR_DIR_4 };

// Motor inversion: set to 1 for motors whose direction pin must be flipped
// to achieve the same "forward" motion. Left side commonly needs inversion.
const int MOTOR_INVERT[] = { 0, 1, 0, 1 };

// Motor Indices (0-based)
// M1(0) & M3(2) are RIGHT side
// M2(1) & M4(3) are LEFT side
const int LEFT_MOTORS[]  = { 1, 3 }; 
const int RIGHT_MOTORS[] = { 0, 2 }; 

const int MOTOR_COUNT = sizeof(PWM_PINS) / sizeof(PWM_PINS[0]);

// Global Speed Variables (Default 10%)
int forwardBackwardSpeed = 10;  // Desired forward/backward speed (serial-adjustable)
int turningSpeed = 10;          // Desired turning speed (serial-adjustable)

// Non-serial-adjustable caps (safety/limit). Change these constants in code only.
const int FORWARD_BACKWARD_CAP = 40; // max allowed forward/backward percent
const int TURNING_CAP = 100;         // max allowed turning percent

// ===== Helpers =====

// Limit duty to range -100 .. 100
int limitDuty(int dutyPercent) {
  if (dutyPercent > 100) return 100;
  if (dutyPercent < -100) return -100;
  return dutyPercent;
}

// Convert -100..100 duty percent to 0..255 PWM
int convertDutyToPwm(int dutyPercent) {
  int mag = abs(dutyPercent);
  mag = constrain(mag, 0, 100);
  return map(mag, 0, 100, 0, 255);
}

// ===== Control Functions =====

void setMotor(int index, int dutyPercent) {
  if (index < 0 || index >= MOTOR_COUNT) return;
  
  int duty = limitDuty(dutyPercent);
  bool forward = (duty >= 0);
  // If this motor is inverted (physically mirrored), flip the direction
  if (index >=0 && index < MOTOR_COUNT) {
    if (MOTOR_INVERT[index]) forward = !forward;
  }
  int pwm = convertDutyToPwm(duty);

  // Set Direction
  digitalWrite(DIR_PINS[index], forward ? HIGH : LOW);
  
  // Set Speed
  analogWrite(PWM_PINS[index], pwm);
}

void setLeftMotorsAll(int dutyPercent) {
  for (size_t i = 0; i < sizeof(LEFT_MOTORS)/sizeof(LEFT_MOTORS[0]); ++i) {
    setMotor(LEFT_MOTORS[i], dutyPercent);
  }
}

void setRightMotorsAll(int dutyPercent) {
  for (size_t i = 0; i < sizeof(RIGHT_MOTORS)/sizeof(RIGHT_MOTORS[0]); ++i) {
    setMotor(RIGHT_MOTORS[i], dutyPercent);
  }
}

void setAllMotors(int dutyPercent) {
  for (int i = 0; i < MOTOR_COUNT; ++i) setMotor(i, dutyPercent);
}

// ===== Motion Functions =====

void forward(int speedPercent) {
  speedPercent = limitDuty(speedPercent);
  // Apply user desired forward/backward limit
  speedPercent = constrain(speedPercent, 0, forwardBackwardSpeed);
  // Enforce non-serial-adjustable safety cap
  speedPercent = min(speedPercent, FORWARD_BACKWARD_CAP);

  setLeftMotorsAll(speedPercent);
  setRightMotorsAll(speedPercent); 
  Serial.print("Cmd: Forward | Speed: "); Serial.println(speedPercent);
}

void backward(int speedPercent) {
  speedPercent = limitDuty(speedPercent);
  // Apply user desired forward/backward limit
  speedPercent = constrain(speedPercent, 0, forwardBackwardSpeed);
  // Enforce non-serial-adjustable safety cap
  speedPercent = min(speedPercent, FORWARD_BACKWARD_CAP);

  setLeftMotorsAll(-speedPercent);
  setRightMotorsAll(-speedPercent);
  Serial.print("Cmd: Backward | Speed: "); Serial.println(speedPercent);
}

void turnRight(int speedPercent) {
  speedPercent = limitDuty(speedPercent);
  // Apply user desired turning limit
  speedPercent = constrain(speedPercent, 0, turningSpeed);
  // Enforce non-serial-adjustable safety cap
  speedPercent = min(speedPercent, TURNING_CAP);

  setLeftMotorsAll(speedPercent);
  setRightMotorsAll(-speedPercent);
  Serial.print("Cmd: Right | Speed: "); Serial.println(speedPercent);
}

void turnLeft(int speedPercent) {
  speedPercent = limitDuty(speedPercent);
  // Apply user desired turning limit
  speedPercent = constrain(speedPercent, 0, turningSpeed);
  // Enforce non-serial-adjustable safety cap
  speedPercent = min(speedPercent, TURNING_CAP);

  setLeftMotorsAll(-speedPercent);
  setRightMotorsAll(speedPercent);
  Serial.print("Cmd: Left | Speed: "); Serial.println(speedPercent);
}

void stopAll() {
  setAllMotors(0);
  Serial.println("Cmd: Stop");
}

void resetControl() {
  stopAll();
  forwardBackwardSpeed = 10;
  // Reset both forward/backward and turning speeds to defaults
  turningSpeed = 10;
  Serial.println("Reset: Forward/Backward and Turning speeds reset to 10.");
}

// ===== Setup and Loop =====

void setup() {
  Serial.begin(115200);

  // Configure Pins
  for (int i = 0; i < MOTOR_COUNT; ++i) {
    pinMode(PWM_PINS[i], OUTPUT);
    pinMode(DIR_PINS[i], OUTPUT);
    analogWrite(PWM_PINS[i], 0);   // Start stopped
    digitalWrite(DIR_PINS[i], LOW);
  }
  
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW); 

  Serial.println("=== Arduino Uno Motor Control Ready (No Brake) ===");
  Serial.println("Use keys: W(Fwd), S(Back), A(Left), D(Right), Space(Stop)");
  Serial.println("Speed Controls: Q/Z (Lin), E/C (Turn), H (Reset)");
}

/**
 * @brief Processes serial commands for robot movement and speed control
 */
void loop() {
  // Check Serial
  if (Serial.available() > 0) {
    char cmd = Serial.read();

    // Ignore newline/carriage return
    if (cmd == '\n' || cmd == '\r') return;

    // Track previous movement/speed state to avoid spamming serial
    static int lastFwdSpeed = -1;
    static int lastTurnSpeed = -1;
    static char lastMoveCmd = 0;

    // Process command
    switch (cmd) {
      // --- Movement ---
      case 'w': 
      case 'W':
      case 'f': 
        digitalWrite(LED_PIN, HIGH);
        forward(forwardBackwardSpeed);
        break;

      case 's': 
      case 'S':
      case 'b': 
        digitalWrite(LED_PIN, HIGH);
        backward(forwardBackwardSpeed);
        break;

      case 'a': 
      case 'A':
      case 'l': 
        digitalWrite(LED_PIN, HIGH);
        turnLeft(turningSpeed);
        break;

      case 'd': 
      case 'D':
      case 'r': 
        digitalWrite(LED_PIN, HIGH);
        turnRight(turningSpeed);
        break;

      // --- Stop ---
      case 'x':
      case 'X':
      case ' ': 
        digitalWrite(LED_PIN, LOW);
        stopAll();
        break;

      // --- Reset ---
      case 'h':
      case 'H':
        digitalWrite(LED_PIN, LOW);
        resetControl();
        // Force update on next change since we reset everything
        lastFwdSpeed = 10; 
        lastTurnSpeed = 10;
        break;

      // --- Forward/Backward Speed Control ---
      case 'q':
      case 'Q':
        forwardBackwardSpeed += 5;
        if (forwardBackwardSpeed > FORWARD_BACKWARD_CAP) forwardBackwardSpeed = FORWARD_BACKWARD_CAP;
        if (forwardBackwardSpeed != lastFwdSpeed) {
            Serial.print("Forward/Backward Speed Increased: "); Serial.println(forwardBackwardSpeed);
            lastFwdSpeed = forwardBackwardSpeed;
        }
        break;

      case 'z':
      case 'Z':
        forwardBackwardSpeed -= 5;
        if (forwardBackwardSpeed < 5) forwardBackwardSpeed = 5;
        if (forwardBackwardSpeed != lastFwdSpeed) {
            Serial.print("Forward/Backward Speed Decreased: "); Serial.println(forwardBackwardSpeed);
            lastFwdSpeed = forwardBackwardSpeed;
        }
        break;

      // --- Turning Speed Control ---
      case 'e':
      case 'E':
        turningSpeed += 5;
        if (turningSpeed > TURNING_CAP) turningSpeed = TURNING_CAP;
        if (turningSpeed != lastTurnSpeed) {
            Serial.print("Turning Speed Increased: "); Serial.println(turningSpeed);
            lastTurnSpeed = turningSpeed;
        }
        break;

      case 'c':
      case 'C':
        turningSpeed -= 5;
        if (turningSpeed < 5) turningSpeed = 5;
        if (turningSpeed != lastTurnSpeed) {
            Serial.print("Turning Speed Decreased: "); Serial.println(turningSpeed);
            lastTurnSpeed = turningSpeed;
        }
        break;

      default:
        break;
    }
  }
}