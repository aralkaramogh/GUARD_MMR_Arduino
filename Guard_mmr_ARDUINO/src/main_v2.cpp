/**
 * @brief 4-Motor Skid Steer Control for Arduino Uno + JKBLD300 Drivers
 * 
 * HARDWARE:
 * - MCU: Arduino Uno (5V Logic)
 * - Drivers: JKBLD300 (Expects 0-5V PWM)
 * 
 * PINOUT:
 * - M1 (Right Front): PWM 3, DIR 4
 * - M2 (Left Front):  PWM 5, DIR 7
 * - M3 (Right Back):  PWM 6, DIR 8
 * - M4 (Left Back):   PWM 9, DIR 12
 * 
 * COMMANDS (Serial Control):
 * Movement:
 *   W/F - Forward  | S/B - Backward  | A/L - Turn Left  | D/R - Turn Right  | X/Space - Stop
 * Speed Control:
 *   Q - Inc Fwd/Back Speed (+10)  | Z - Dec Fwd/Back Speed (-10)
 *   E - Inc Turn Speed (+10)       | C - Dec Turn Speed (-10)
 *   H - Reset (stop, Fwd/Back to 10, keep turn speed)
 * Speed Range: 10-100% | Default: 10%
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

// Motor Indices (0-based)
// M1(0) & M3(2) are RIGHT side
// M2(1) & M4(3) are LEFT side
const int LEFT_MOTORS[]  = { 1, 3 }; 
const int RIGHT_MOTORS[] = { 0, 2 }; 

const int MOTOR_COUNT = sizeof(PWM_PINS) / sizeof(PWM_PINS[0]);

// Global Speed Variables (Default 10%)
int forwardBackwardSpeed = 10;  // Speed limit for forward/backward motion
int turningSpeed = 10;          // Speed limit for turning motion

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

// ===== Basic Setters =====

void setMotor(int index, int dutyPercent) {
  if (index < 0 || index >= MOTOR_COUNT) return;
  
  int duty = limitDuty(dutyPercent);
  bool forward = (duty >= 0);
  int pwm = convertDutyToPwm(duty);

  // Set Direction
  // Note: If a wheel spins backward when it should go forward, flip HIGH/LOW here
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
  // Apply forward/backward speed limit
  speedPercent = constrain(speedPercent, 0, forwardBackwardSpeed);
  // Skid Steer Logic: Usually both sides get Positive speed if wired identically.
  // If motors are mounted mirrored, one side might need negative.
  // Assuming standard wiring where 'Positive' = 'Forward' for that specific motor:
  setLeftMotorsAll(speedPercent);
  setRightMotorsAll(speedPercent); 
  Serial.print("Cmd: Forward | Speed: "); Serial.println(speedPercent);
}

void backward(int speedPercent) {
  speedPercent = limitDuty(speedPercent);
  // Apply forward/backward speed limit
  speedPercent = constrain(speedPercent, 0, forwardBackwardSpeed);
  setLeftMotorsAll(-speedPercent);
  setRightMotorsAll(-speedPercent);
  Serial.print("Cmd: Backward | Speed: "); Serial.println(speedPercent);
}

void turnRight(int speedPercent) {
  speedPercent = limitDuty(speedPercent);
  // Apply turning speed limit
  speedPercent = constrain(speedPercent, 0, turningSpeed);
  // Skid Steer Right: Left moves Forward (+), Right moves Backward (-)
  setLeftMotorsAll(speedPercent);
  setRightMotorsAll(-speedPercent);
  Serial.print("Cmd: Right | Speed: "); Serial.println(speedPercent);
}

void turnLeft(int speedPercent) {
  speedPercent = limitDuty(speedPercent);
  // Apply turning speed limit
  speedPercent = constrain(speedPercent, 0, turningSpeed);
  // Skid Steer Left: Left moves Backward (-), Right moves Forward (+)
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
  // turningSpeed remains unchanged
  Serial.println("Reset: Forward/Backward speed reset to 10. Turning speed unchanged.");
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

  Serial.println("=== Arduino Uno Motor Control Ready ===");
  Serial.println("Use keys: W(Fwd), S(Back), A(Left), D(Right), Space(Stop)");
  Serial.println("Speed Controls:");
  Serial.println("  Q/Z - Increase/Decrease Forward/Backward Speed");
  Serial.println("  E/C - Increase/Decrease Turning Speed");
  Serial.println("  H - Reset (stops all, resets Fwd/Back speed to 10)");
  Serial.print("Initial Forward/Backward Speed: "); Serial.println(forwardBackwardSpeed);
  Serial.print("Initial Turning Speed: "); Serial.println(turningSpeed);
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
        break;

      // --- Forward/Backward Speed Control ---
      case 'q':
      case 'Q':
        forwardBackwardSpeed += 10;
        if (forwardBackwardSpeed > 100) forwardBackwardSpeed = 100;
        Serial.print("Forward/Backward Speed Increased: "); Serial.println(forwardBackwardSpeed);
        break;

      case 'z':
      case 'Z':
        forwardBackwardSpeed -= 10;
        if (forwardBackwardSpeed < 10) forwardBackwardSpeed = 10;
        Serial.print("Forward/Backward Speed Decreased: "); Serial.println(forwardBackwardSpeed);
        break;

      // --- Turning Speed Control ---
      case 'e':
      case 'E':
        turningSpeed += 10;
        if (turningSpeed > 100) turningSpeed = 100;
        Serial.print("Turning Speed Increased: "); Serial.println(turningSpeed);
        break;

      case 'c':
      case 'C':
        turningSpeed -= 10;
        if (turningSpeed < 10) turningSpeed = 10;
        Serial.print("Turning Speed Decreased: "); Serial.println(turningSpeed);
        break;

      default:
        break;
    }
  }
}