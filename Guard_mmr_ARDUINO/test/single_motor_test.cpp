#include <Arduino.h>

// Single Motor Test
// - Select a motor (1..4) and run it forward/reverse/stop
// - Useful for verifying direction/polarity of individual motors (e.g., right-back)

// --- Pin mapping (match your main sketch) ---
const int MOTOR_PWM_1 = 3;  // Right Front
const int MOTOR_PWM_2 = 5;  // Left Front
const int MOTOR_PWM_3 = 6;  // Right Back
const int MOTOR_PWM_4 = 9;  // Left Back

const int MOTOR_DIR_1 = 4;
const int MOTOR_DIR_2 = 7;
const int MOTOR_DIR_3 = 8;
const int MOTOR_DIR_4 = 12;

const int PWM_PINS[] = { MOTOR_PWM_1, MOTOR_PWM_2, MOTOR_PWM_3, MOTOR_PWM_4 };
const int DIR_PINS[] = { MOTOR_DIR_1, MOTOR_DIR_2, MOTOR_DIR_3, MOTOR_DIR_4 };
const int MOTOR_COUNT = sizeof(PWM_PINS) / sizeof(PWM_PINS[0]);

// If a motor is physically inverted, set 1 here to flip logical direction
int MOTOR_INVERT[] = { 0, 1, 1, 1 }; // default tuned for your hardware after previous change

int selectedMotor = 3; // default to motor index 3 (right-back is index 2; user-friendly choice is 3 -> index2)
int selectedIndex = 2; // 0-based index used internally
int testSpeed = 80; // percent

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

void setMotor(int index, int dutyPercent) {
  if (index < 0 || index >= MOTOR_COUNT) return;
  int duty = limitDuty(dutyPercent);
  bool forward = (duty >= 0);
  if (MOTOR_INVERT[index]) forward = !forward;
  int pwm = convertDutyToPwm(duty);
  digitalWrite(DIR_PINS[index], forward ? HIGH : LOW);
  analogWrite(PWM_PINS[index], pwm);
  Serial.print("Motor "); Serial.print(index+1);
  Serial.print(" | Dir: "); Serial.print(forward ? "FWD" : "REV");
  Serial.print(" | PWM: "); Serial.println(pwm);
}

void stopMotor(int index) {
  if (index < 0 || index >= MOTOR_COUNT) return;
  analogWrite(PWM_PINS[index], 0);
  Serial.print("Motor "); Serial.print(index+1); Serial.println(" stopped");
}

void printHelp() {
  Serial.println("--- Single Motor Test ---");
  Serial.println("Commands:");
  Serial.println("  1..4  - Select motor (1-based)");
  Serial.println("  v     - Show selected motor and settings");
  Serial.println("  f     - Run selected motor forward at current speed");
  Serial.println("  r     - Run selected motor reverse at current speed");
  Serial.println("  s     - Stop selected motor");
  Serial.println("  p     - Pulse selected motor forward briefly (500ms)");
  Serial.println("  +     - Increase speed by 10%");
  Serial.println("  -     - Decrease speed by 10%");
  Serial.println("  a     - Auto-run: forward 2s -> stop 0.5s -> reverse 2s -> stop");
  Serial.println("  i     - Toggle inversion for selected motor (live)");
  Serial.println("  I     - Print inversion flags for all motors");
  Serial.println("  D/d   - Set DIR pin HIGH / LOW (raw mode, stops PWM)");
  Serial.println("  t     - Toggle DIR pin state (raw)");
  Serial.println("  w     - Show DIR pin current state");
  Serial.println("--------------------------");
}

void showStatus() {
  Serial.print("Selected motor: "); Serial.print(selectedMotor);
  Serial.print(" (index "); Serial.print(selectedIndex);
  Serial.print(") | Speed: "); Serial.print(testSpeed);
  Serial.print("% | Inverted: "); Serial.print(MOTOR_INVERT[selectedIndex] ? "Yes" : "No");
  Serial.print(" | DIR pin: "); Serial.println(digitalRead(DIR_PINS[selectedIndex]) ? "HIGH" : "LOW");
}

void setup() {
  Serial.begin(115200);
  while (!Serial) { delay(10); }

  for (int i = 0; i < MOTOR_COUNT; ++i) {
    pinMode(PWM_PINS[i], OUTPUT);
    pinMode(DIR_PINS[i], OUTPUT);
    analogWrite(PWM_PINS[i], 0);
    digitalWrite(DIR_PINS[i], LOW);
  }

  Serial.println("Single motor test ready. Press 'v' for help.");
  showStatus();
  printHelp();
}

void autoRun() {
  Serial.println("Auto-run: forward");
  setMotor(selectedIndex, testSpeed);
  delay(2000);
  stopMotor(selectedIndex);
  delay(500);
  Serial.println("Auto-run: reverse");
  setMotor(selectedIndex, -testSpeed);
  delay(2000);
  stopMotor(selectedIndex);
  Serial.println("Auto-run finished");
}

// --- Diagnostic helpers ---
void toggleInversion(int index) {
  if (index < 0 || index >= MOTOR_COUNT) return;
  MOTOR_INVERT[index] = !MOTOR_INVERT[index];
  Serial.print("Motor "); Serial.print(index+1);
  Serial.print(" inversion set to "); Serial.println(MOTOR_INVERT[index] ? "ON" : "OFF");
}

void printAllInversions() {
  Serial.print("Inversions: ");
  for (int i = 0; i < MOTOR_COUNT; ++i) {
    Serial.print(i+1); Serial.print(":"); Serial.print(MOTOR_INVERT[i] ? "1" : "0");
    if (i < MOTOR_COUNT-1) Serial.print(", ");
  }
  Serial.println();
}

void setDirRaw(int index, bool high) {
  if (index < 0 || index >= MOTOR_COUNT) return;
  analogWrite(PWM_PINS[index], 0); // stop PWM to avoid spinning during raw DIR tests
  digitalWrite(DIR_PINS[index], high ? HIGH : LOW);
  Serial.print("DIR "); Serial.print(index+1);
  Serial.print(" set to "); Serial.println(high ? "HIGH" : "LOW");
}

void showDirState(int index) {
  if (index < 0 || index >= MOTOR_COUNT) return;
  Serial.print("DIR "); Serial.print(index+1);
  Serial.print(" is "); Serial.println(digitalRead(DIR_PINS[index]) ? "HIGH" : "LOW");
}

void loop() {
  if (Serial.available() > 0) {
    char c = Serial.read();
    if (c >= '1' && c <= '4') {
      selectedMotor = c - '0';
      selectedIndex = selectedMotor - 1;
      Serial.print("Selected motor set to "); Serial.println(selectedMotor);
      showStatus();
      return;
    }
    switch (c) {
      case 'v': showStatus(); printHelp(); break;
      case 'f': setMotor(selectedIndex, testSpeed); break;
      case 'r': setMotor(selectedIndex, -testSpeed); break;
      case 's': stopMotor(selectedIndex); break;
      case 'p': setMotor(selectedIndex, testSpeed); delay(500); stopMotor(selectedIndex); break;
      case 'i': toggleInversion(selectedIndex); showStatus(); break;
      case 'I': printAllInversions(); break;
      case 'D': setDirRaw(selectedIndex, true); break;
      case 'd': setDirRaw(selectedIndex, false); break;
      case 't': setDirRaw(selectedIndex, !digitalRead(DIR_PINS[selectedIndex])); showDirState(selectedIndex); break;
      case 'w': showDirState(selectedIndex); break;
      case '+': testSpeed = min(100, testSpeed + 10); Serial.print("Speed: "); Serial.println(testSpeed); break;
      case '-': testSpeed = max(0, testSpeed - 10); Serial.print("Speed: "); Serial.println(testSpeed); break;
      case 'a': autoRun(); break;
      default: break;
    }
  }
}
