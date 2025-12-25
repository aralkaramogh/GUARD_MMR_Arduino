/**
 * Skid-steer motor controller for Arduino MEGA + JKBLD300 drivers.
 * 
 * NEW STEERING LOGIC:
 * - Forward/Backward: Both sides same speed
 * - Turn Right: Both move forward, but RIGHT motors at ~50% PWM of LEFT motors
 *   (creates smooth arc instead of spinning in place)
 * - Turn Left: Both move forward, but LEFT motors at ~50% PWM of RIGHT motors
 * 
 * TEST MODE: Serial commands to adjust PWM per motor in real-time
 */

#include <Arduino.h>

// ===== Pin Configuration (Arduino MEGA) =====
const int LED_PIN = 13;

// PWM Pins (MEGA has many: 2-13, 44-46)
const int MOTOR_PWM_1 = 3;   // Right Front
const int MOTOR_PWM_2 = 5;   // Left Front
const int MOTOR_PWM_3 = 6;   // Right Back
const int MOTOR_PWM_4 = 9;   // Left Back

// Direction Pins
const int MOTOR_DIR_1 = 4;
const int MOTOR_DIR_2 = 7;
const int MOTOR_DIR_3 = 8;
const int MOTOR_DIR_4 = 12;

// Grouping Arrays
const int PWM_PINS[] = { MOTOR_PWM_1, MOTOR_PWM_2, MOTOR_PWM_3, MOTOR_PWM_4 };
const int DIR_PINS[] = { MOTOR_DIR_1, MOTOR_DIR_2, MOTOR_DIR_3, MOTOR_DIR_4 };

// Motor inversion
const int MOTOR_INVERT[] = { 0, 1, 0, 1 };

// Motor Indices
const int LEFT_MOTORS[]  = { 1, 3 };
const int RIGHT_MOTORS[] = { 0, 2 };

const int MOTOR_COUNT = 4;

// ===== TEST MODE: Per-Motor PWM Control =====
// These are the actual PWM values sent to each motor in test mode
// Range: 0-255, but we'll also track direction (sign)
int motorDuty[] = { 0, 0, 0, 0 }; // M1(RF), M2(LF), M3(RB), M4(LB)

// Speed settings for normal operation
int forwardBackwardSpeed = 50;  // % (0-100)
int turningSpeed = 50;          // %
int turnRatioPercent = 50;      // % PWM for turning inner wheels (adjustable for tuning)

const int FORWARD_BACKWARD_CAP = 100;
const int TURNING_CAP = 100;

// ===== Helpers =====

int limitDuty(int dutyPercent) {
  if (dutyPercent > 100) return 100;
  if (dutyPercent < -100) return -100;
  return dutyPercent;
}

int convertDutyToPwm(int dutyPercent) {
  int mag = abs(dutyPercent);
  mag = constrain(mag, 0, 100);
  return map(mag, 0, 100, 0, 255);
}

// ===== Basic Motor Control =====

void setMotor(int index, int dutyPercent) {
  if (index < 0 || index >= MOTOR_COUNT) return;
  
  int duty = limitDuty(dutyPercent);
  bool forward = (duty >= 0);
  
  if (MOTOR_INVERT[index]) forward = !forward;
  
  int pwm = convertDutyToPwm(duty);
  
  digitalWrite(DIR_PINS[index], forward ? HIGH : LOW);
  analogWrite(PWM_PINS[index], pwm);
}

void setAllMotors(int dutyPercent) {
  for (int i = 0; i < MOTOR_COUNT; ++i) setMotor(i, dutyPercent);
}

// ===== NEW SKID STEERING LOGIC =====

void forward(int speedPercent) {
  speedPercent = limitDuty(speedPercent);
  speedPercent = constrain(speedPercent, 0, forwardBackwardSpeed);
  speedPercent = min(speedPercent, FORWARD_BACKWARD_CAP);
  
  // Both sides: same positive speed
  for (int i = 0; i < MOTOR_COUNT; ++i) {
    setMotor(i, speedPercent);
  }
  Serial.print(">> FORWARD | Speed: "); Serial.println(speedPercent);
}

void backward(int speedPercent) {
  speedPercent = limitDuty(speedPercent);
  speedPercent = constrain(speedPercent, 0, forwardBackwardSpeed);
  speedPercent = min(speedPercent, FORWARD_BACKWARD_CAP);
  
  // Both sides: same negative speed
  for (int i = 0; i < MOTOR_COUNT; ++i) {
    setMotor(i, -speedPercent);
  }
  Serial.print(">> BACKWARD | Speed: "); Serial.println(speedPercent);
}

void turnRight(int speedPercent) {
  speedPercent = limitDuty(speedPercent);
  speedPercent = constrain(speedPercent, 0, turningSpeed);
  speedPercent = min(speedPercent, TURNING_CAP);
  
  // Both sides move FORWARD, but right motors slower
  int leftSpeed = speedPercent;
  int rightSpeed = (speedPercent * turnRatioPercent) / 100;
  
  // Left motors: full speed forward
  for (int i = 0; i < 2; ++i) {
    setMotor(LEFT_MOTORS[i], leftSpeed);
  }
  
  // Right motors: reduced speed forward (creates smooth arc)
  for (int i = 0; i < 2; ++i) {
    setMotor(RIGHT_MOTORS[i], rightSpeed);
  }
  
  Serial.print(">> TURN RIGHT | Left: "); Serial.print(leftSpeed);
  Serial.print("% | Right: "); Serial.print(rightSpeed); Serial.println("%");
}

void turnLeft(int speedPercent) {
  speedPercent = limitDuty(speedPercent);
  speedPercent = constrain(speedPercent, 0, turningSpeed);
  speedPercent = min(speedPercent, TURNING_CAP);
  
  // Both sides move FORWARD, but left motors slower
  int rightSpeed = speedPercent;
  int leftSpeed = (speedPercent * turnRatioPercent) / 100;
  
  // Left motors: reduced speed forward
  for (int i = 0; i < 2; ++i) {
    setMotor(LEFT_MOTORS[i], leftSpeed);
  }
  
  // Right motors: full speed forward
  for (int i = 0; i < 2; ++i) {
    setMotor(RIGHT_MOTORS[i], rightSpeed);
  }
  
  Serial.print(">> TURN LEFT | Left: "); Serial.print(leftSpeed);
  Serial.print("% | Right: "); Serial.print(rightSpeed); Serial.println("%");
}

void stopAll() {
  setAllMotors(0);
  Serial.println(">> STOP");
}

// ===== TEST MODE: Direct Motor Control =====
void printTestMenu() {
  Serial.println("\n===== TEST MODE =====");
  Serial.println("Commands:");
  Serial.println("  M<N><PWM> - Set motor N (0-3) to PWM (0-255)");
  Serial.println("             M0100 = Motor 0 at PWM 100");
  Serial.println("  A<PWM>    - Set all motors to PWM");
  Serial.println("  W/S/A/D   - Forward/Backward/Left/Right at current speed");
  Serial.println("  Q/Z       - Increase/Decrease forward speed (0-100%)");
  Serial.println("  E/C       - Increase/Decrease turning speed (0-100%)");
  Serial.println("  R/T       - Increase/Decrease turn ratio (0-100%)");
  Serial.println("  X         - Stop all");
  Serial.println("  ?         - Show motor status");
  Serial.println("  H         - Show this menu");
  Serial.println();
}

void printMotorStatus() {
  Serial.println("\n--- Motor Status ---");
  Serial.print("M0 (RF): "); Serial.print(motorDuty[0]); Serial.print("/255");
  Serial.print("  | M1 (LF): "); Serial.print(motorDuty[1]); Serial.println("/255");
  Serial.print("M2 (RB): "); Serial.print(motorDuty[2]); Serial.print("/255");
  Serial.print("  | M3 (LB): "); Serial.print(motorDuty[3]); Serial.println("/255");
  Serial.print("\nSettings: Fwd="); Serial.print(forwardBackwardSpeed);
  Serial.print("% | Turn="); Serial.print(turningSpeed);
  Serial.print("% | TurnRatio="); Serial.print(turnRatioPercent);
  Serial.println("%");
}

void setup() {
  Serial.begin(115200);

  // Configure Pins
  for (int i = 0; i < MOTOR_COUNT; ++i) {
    pinMode(PWM_PINS[i], OUTPUT);
    pinMode(DIR_PINS[i], OUTPUT);
    analogWrite(PWM_PINS[i], 0);
    digitalWrite(DIR_PINS[i], LOW);
  }
  
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  Serial.println("\n");
  Serial.println("====================================");
  Serial.println("  Arduino MEGA Skid Steering TEST");
  Serial.println("====================================");
  Serial.println("Motor Layout:");
  Serial.println("  M0(RF) & M2(RB) = RIGHT SIDE");
  Serial.println("  M1(LF) & M3(LB) = LEFT SIDE");
  Serial.println("\nNew Turn Logic:");
  Serial.println("  RIGHT: Both forward, RIGHT at 50% PWM");
  Serial.println("  LEFT:  Both forward, LEFT at 50% PWM");
  Serial.println();
  
  printTestMenu();
}

void loop() {
  if (Serial.available() > 0) {
    char cmd = Serial.read();
    
    if (cmd == '\n' || cmd == '\r') return;
    
    cmd = tolower(cmd);

    // Direct motor control: M<motorNum><PWM>
    if (cmd == 'm') {
      if (Serial.available() >= 2) {
        int motorNum = Serial.parseInt(); // reads one digit (0-3)
        int pwmVal = Serial.parseInt();   // reads PWM value
        
        if (motorNum >= 0 && motorNum < 4 && pwmVal >= 0 && pwmVal <= 255) {
          motorDuty[motorNum] = pwmVal;
          analogWrite(PWM_PINS[motorNum], pwmVal);
          Serial.print("M"); Serial.print(motorNum);
          Serial.print(" set to PWM "); Serial.println(pwmVal);
        }
      }
      return;
    }
    
    // Set all motors
    if (cmd == 'a') {
      if (Serial.available() >= 1) {
        int pwmVal = Serial.parseInt();
        if (pwmVal >= 0 && pwmVal <= 255) {
          for (int i = 0; i < 4; ++i) {
            motorDuty[i] = pwmVal;
            analogWrite(PWM_PINS[i], pwmVal);
          }
          Serial.print("All motors set to PWM "); Serial.println(pwmVal);
        }
      }
      return;
    }

    // Movement commands
    switch (cmd) {
      case 'w':
        forward(forwardBackwardSpeed);
        break;
      
      case 's':
        backward(forwardBackwardSpeed);
        break;
      
      case 'd':
        turnRight(turningSpeed);
        break;
      
      case 'a':
        turnLeft(turningSpeed);
        break;
      
      case 'x':
        stopAll();
        break;
      
      // Speed controls
      case 'q':
        forwardBackwardSpeed += 5;
        if (forwardBackwardSpeed > 100) forwardBackwardSpeed = 100;
        Serial.print("Forward/Backward Speed: "); Serial.println(forwardBackwardSpeed);
        break;
      
      case 'z':
        forwardBackwardSpeed -= 5;
        if (forwardBackwardSpeed < 5) forwardBackwardSpeed = 5;
        Serial.print("Forward/Backward Speed: "); Serial.println(forwardBackwardSpeed);
        break;
      
      case 'e':
        turningSpeed += 5;
        if (turningSpeed > 100) turningSpeed = 100;
        Serial.print("Turning Speed: "); Serial.println(turningSpeed);
        break;
      
      case 'c':
        turningSpeed -= 5;
        if (turningSpeed < 5) turningSpeed = 5;
        Serial.print("Turning Speed: "); Serial.println(turningSpeed);
        break;
      
      // Turn ratio control (adjusts inner wheel speed during turns)
      case 'r':
        turnRatioPercent += 5;
        if (turnRatioPercent > 100) turnRatioPercent = 100;
        Serial.print("Turn Ratio: "); Serial.print(turnRatioPercent); Serial.println("%");
        break;
      
      case 't':
        turnRatioPercent -= 5;
        if (turnRatioPercent < 5) turnRatioPercent = 5;
        Serial.print("Turn Ratio: "); Serial.print(turnRatioPercent); Serial.println("%");
        break;
      
      // Status/Info
      case '?':
        printMotorStatus();
        break;
      
      case 'h':
        printTestMenu();
        break;
      
      default:
        break;
    }
  }
}