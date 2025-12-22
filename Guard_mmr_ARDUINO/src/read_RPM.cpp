/**
 * ============================================================================
 * RPM READER ONLY - SPEED PIN MONITOR
 * ============================================================================
 * 
 * This code ONLY reads and displays RPM from JKBLD300 SPEED output.
 * Motor control is done manually via potentiometer connected to JKBLD300.
 * 
 * Purpose:
 * - Verify SPEED feedback wiring
 * - Confirm RPM calculation is correct
 * - Test different motor speeds manually
 * - Validate constants (POLE_PAIRS, DRIVER_CONSTANT, GEAR_RATIO)
 * 
 * Hardware Setup:
 * - Arduino Uno
 * - JKBLD300 V2 Driver
 * - 86BLF Motor with 16:1 Gearbox
 * - Potentiometer connected to JKBLD300 for manual speed control
 * 
 * Wiring:
 * ┌─────────────────────────────────────────────────────────┐
 * │ JKBLD300 SPEED output → Arduino Pin 2 (INT0)           │
 * │ JKBLD300 GND → Arduino GND (CRITICAL!)                 │
 * │                                                         │
 * │ Potentiometer → JKBLD300 PWM/Speed Control Input       │
 * │ (Follow JKBLD300 manual for potentiometer connection)  │
 * └─────────────────────────────────────────────────────────┘
 * 
 * NO OTHER CONNECTIONS NEEDED - Arduino only reads SPEED pin!
 * 
 * Serial Commands:
 * - M : Print current status
 * - V : Toggle continuous printing (default ON)
 * - R : Reset pulse counter
 * - C : Clear screen and show header
 * - S : Show statistics (min/max/avg RPM)
 * - H : Help menu
 * 
 * Testing Procedure:
 * 1. Upload this code
 * 2. Open Serial Monitor (115200 baud)
 * 3. Adjust potentiometer slowly from 0 to max
 * 4. Watch RPM values increase
 * 5. Verify RPM readings make sense (max ~187 RPM for 16:1 gearbox)
 * 
 * ============================================================================
 */

#include <Arduino.h>

// ===== Pin Configuration =====
const int SPEED_FB = 2;         // SPEED feedback from JKBLD300 (INT0)
const int LED_PIN = 13;         // Visual indicator

// ===== Hardware Constants =====
const float POLE_PAIRS = 4.0f;           // 86BLF: 8 poles = 4 pole pairs
const float GEAR_RATIO = 16.0f;          // 16:1 gearbox
const float DRIVER_CONSTANT = 20.0f;     // JKBLD300 specific constant
const float MAX_MOTOR_RPM = 3000.0f;     // 86BLF rated speed
const float MAX_WHEEL_RPM = MAX_MOTOR_RPM / GEAR_RATIO;  // 187.5 RPM

// ===== Speed Measurement Variables =====
volatile unsigned long speedPulseCount = 0;
unsigned long lastPulseCount = 0;
unsigned long lastCalcTime = 0;
float currentRPM = 0.0f;
float currentMotorRPM = 0.0f;
float currentFrequency = 0.0f;

// ===== Statistics =====
float minRPM = 999999.0f;
float maxRPM = 0.0f;
float sumRPM = 0.0f;
unsigned long sampleCount = 0;

// ===== Timing =====
const unsigned long RPM_CALC_INTERVAL = 100;   // Calculate RPM every 100ms
const unsigned long PRINT_INTERVAL = 200;       // Print every 200ms
unsigned long lastPrintTime = 0;

// ===== Display Settings =====
bool continuousPrint = true;
bool showRaw = false;
unsigned long startTime = 0;

// ===== ISR for SPEED feedback =====
void speedISR() {
  speedPulseCount++;
  // Brief LED pulse on each pulse
  digitalWrite(LED_PIN, !digitalRead(LED_PIN));
}

// ===== RPM Calculation =====
void calculateRPM() {
  unsigned long currentTime = millis();
  unsigned long deltaTime = currentTime - lastCalcTime;
  
  if (deltaTime >= RPM_CALC_INTERVAL) {
    // Get pulse count atomically
    noInterrupts();
    unsigned long currentCount = speedPulseCount;
    interrupts();
    
    // Calculate pulses in this interval
    unsigned long deltaPulses = currentCount - lastPulseCount;
    float dt_sec = deltaTime / 1000.0f;
    
    // Calculate frequency (Hz) - pulses per second
    currentFrequency = (float)deltaPulses / dt_sec;
    
    // Motor RPM using JKBLD300 formula:
    // motorRPM = (frequency / POLE_PAIRS) × DRIVER_CONSTANT
    currentMotorRPM = (currentFrequency / POLE_PAIRS) * DRIVER_CONSTANT;
    
    // Wheel RPM (after gearbox reduction)
    currentRPM = currentMotorRPM / GEAR_RATIO;
    
    // Update statistics
    if (currentRPM > 1.0f) {  // Ignore very low speeds
      if (currentRPM < minRPM) minRPM = currentRPM;
      if (currentRPM > maxRPM) maxRPM = currentRPM;
      sumRPM += currentRPM;
      sampleCount++;
    }
    
    // Store for next calculation
    lastPulseCount = currentCount;
    lastCalcTime = currentTime;
  }
}

// ===== Display Functions =====
void printHeader() {
  Serial.println("\n\n");
  Serial.println("════════════════════════════════════════════════════════");
  Serial.println("           RPM READER - SPEED PIN MONITOR");
  Serial.println("════════════════════════════════════════════════════════");
  Serial.println();
  Serial.println("Configuration:");
  Serial.print("  Motor: 86BLF ("); Serial.print((int)POLE_PAIRS*2); Serial.println(" poles)");
  Serial.print("  Gearbox: "); Serial.print(GEAR_RATIO, 1); Serial.println(":1");
  Serial.print("  Max Motor RPM: "); Serial.println((int)MAX_MOTOR_RPM);
  Serial.print("  Max Wheel RPM: "); Serial.println((int)MAX_WHEEL_RPM);
  Serial.print("  Driver Constant: "); Serial.println(DRIVER_CONSTANT, 1);
  Serial.println();
  Serial.println("SPEED Pin: Arduino Pin 2 (INT0)");
  Serial.println("Control: Manual via Potentiometer");
  Serial.println();
  Serial.println("Commands: M=Status, V=Toggle Print, R=Reset, S=Stats, H=Help");
  Serial.println("════════════════════════════════════════════════════════");
  Serial.println();
  
  if (continuousPrint) {
    Serial.println("Wheel RPM | Motor RPM | Freq(Hz) | Pulses | Time(s)");
    Serial.println("──────────┼───────────┼──────────┼────────┼─────────");
  }
}

void printRPM() {
  float elapsedTime = (millis() - startTime) / 1000.0f;
  
  // Format: Wheel RPM | Motor RPM | Frequency | Total Pulses | Time
  char buffer[100];
  sprintf(buffer, "%6.1f    | %7.1f   | %6.1f   | %6lu | %6.1f",
          currentRPM, 
          currentMotorRPM, 
          currentFrequency,
          speedPulseCount,
          elapsedTime);
  Serial.println(buffer);
}

void printStatus() {
  Serial.println("\n╔════════════════════════════════════════════════════╗");
  Serial.println("║              CURRENT STATUS                        ║");
  Serial.println("╚════════════════════════════════════════════════════╝");
  Serial.println();
  Serial.println("SPEED Readings:");
  Serial.print("  Wheel RPM:    "); Serial.println(currentRPM, 2);
  Serial.print("  Motor RPM:    "); Serial.println(currentMotorRPM, 1);
  Serial.print("  Frequency:    "); Serial.print(currentFrequency, 2); Serial.println(" Hz");
  Serial.print("  Total Pulses: "); Serial.println(speedPulseCount);
  Serial.println();
  
  float elapsedTime = (millis() - startTime) / 1000.0f;
  Serial.print("Elapsed Time:   "); Serial.print(elapsedTime, 1); Serial.println(" seconds");
  
  if (sampleCount > 0) {
    Serial.println();
    Serial.println("Statistics (RPM > 1):");
    Serial.print("  Min RPM:  "); Serial.println(minRPM, 1);
    Serial.print("  Max RPM:  "); Serial.println(maxRPM, 1);
    Serial.print("  Avg RPM:  "); Serial.println(sumRPM / sampleCount, 1);
    Serial.print("  Samples:  "); Serial.println(sampleCount);
  }
  Serial.println();
}

void printStatistics() {
  Serial.println("\n╔════════════════════════════════════════════════════╗");
  Serial.println("║              RPM STATISTICS                        ║");
  Serial.println("╚════════════════════════════════════════════════════╝");
  Serial.println();
  
  if (sampleCount == 0) {
    Serial.println("No data collected yet (RPM must be > 1)");
    Serial.println("Run the motor and try again.");
  } else {
    float avgRPM = sumRPM / sampleCount;
    float elapsedTime = (millis() - startTime) / 1000.0f;
    
    Serial.print("Samples Collected: "); Serial.println(sampleCount);
    Serial.print("Time Period:       "); Serial.print(elapsedTime, 1); Serial.println(" sec");
    Serial.println();
    Serial.print("Minimum RPM:       "); Serial.println(minRPM, 2);
    Serial.print("Maximum RPM:       "); Serial.println(maxRPM, 2);
    Serial.print("Average RPM:       "); Serial.println(avgRPM, 2);
    Serial.print("Range:             "); Serial.println(maxRPM - minRPM, 2);
    Serial.println();
    
    // Speed as percentage of max
    float percentOfMax = (avgRPM / MAX_WHEEL_RPM) * 100.0f;
    Serial.print("Average Speed:     "); Serial.print(percentOfMax, 1); Serial.println("% of max");
    
    // Total pulses and rate
    Serial.println();
    Serial.print("Total Pulses:      "); Serial.println(speedPulseCount);
    Serial.print("Pulse Rate:        "); 
    Serial.print(speedPulseCount / elapsedTime, 1); 
    Serial.println(" pulses/sec");
  }
  Serial.println();
}

void resetStats() {
  speedPulseCount = 0;
  lastPulseCount = 0;
  minRPM = 999999.0f;
  maxRPM = 0.0f;
  sumRPM = 0.0f;
  sampleCount = 0;
  startTime = millis();
  lastCalcTime = millis();
  Serial.println("\n✓ Counter and statistics reset!");
  Serial.println();
}

void printHelp() {
  Serial.println("\n╔════════════════════════════════════════════════════╗");
  Serial.println("║                 HELP MENU                          ║");
  Serial.println("╚════════════════════════════════════════════════════╝");
  Serial.println();
  Serial.println("Commands:");
  Serial.println("  M - Print current status (one-time)");
  Serial.println("  V - Toggle continuous printing (ON/OFF)");
  Serial.println("  R - Reset pulse counter and statistics");
  Serial.println("  S - Show detailed statistics");
  Serial.println("  C - Clear screen and show header");
  Serial.println("  H - Show this help menu");
  Serial.println();
  Serial.println("How to Test:");
  Serial.println("  1. Slowly turn potentiometer from 0 to max");
  Serial.println("  2. Watch RPM values increase");
  Serial.println("  3. Verify max RPM is ~187 (with 16:1 gearbox)");
  Serial.println("  4. Check that readings are stable");
  Serial.println();
  Serial.println("Expected Values:");
  Serial.print("  Max Wheel RPM:  ~"); Serial.print((int)MAX_WHEEL_RPM); Serial.println(" RPM");
  Serial.print("  Max Motor RPM:  ~"); Serial.print((int)MAX_MOTOR_RPM); Serial.println(" RPM");
  Serial.println("  Frequency:      Proportional to motor speed");
  Serial.println();
  Serial.println("Troubleshooting:");
  Serial.println("  No pulses?         → Check SPEED pin wiring");
  Serial.println("  RPM = 0?           → Check common ground");
  Serial.println("  Wrong RPM?         → Verify constants in code");
  Serial.println("  Erratic readings?  → Check for loose connections");
  Serial.println();
}

// ===== Serial Command Processing =====
void processCommand() {
  if (Serial.available() > 0) {
    char cmd = Serial.read();
    
    // Clear remaining buffer
    while (Serial.available()) Serial.read();
    
    switch (cmd) {
      case 'm': case 'M':
        printStatus();
        break;
        
      case 'v': case 'V':
        continuousPrint = !continuousPrint;
        Serial.print("\nContinuous Print: ");
        Serial.println(continuousPrint ? "ON" : "OFF");
        if (continuousPrint) {
          Serial.println();
          Serial.println("Wheel RPM | Motor RPM | Freq(Hz) | Pulses | Time(s)");
          Serial.println("──────────┼───────────┼──────────┼────────┼─────────");
        }
        Serial.println();
        break;
        
      case 'r': case 'R':
        resetStats();
        break;
        
      case 's': case 'S':
        printStatistics();
        break;
        
      case 'c': case 'C':
        printHeader();
        break;
        
      case 'h': case 'H': case '?':
        printHelp();
        break;
        
      case '\n': case '\r':
        // Ignore newlines
        break;
        
      default:
        Serial.print("\nUnknown command: ");
        Serial.println(cmd);
        Serial.println("Press 'H' for help");
        Serial.println();
        break;
    }
  }
}

// ===== Setup =====
void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000);  // Wait up to 3s for serial
  
  // Configure pins
  pinMode(SPEED_FB, INPUT);
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);
  
  // Attach interrupt for SPEED feedback
  attachInterrupt(digitalPinToInterrupt(SPEED_FB), speedISR, RISING);
  
  // Initialize timing
  startTime = millis();
  lastCalcTime = millis();
  lastPrintTime = millis();
  
  // Show startup screen
  printHeader();
  
  // LED startup sequence
  for (int i = 0; i < 3; i++) {
    digitalWrite(LED_PIN, HIGH);
    delay(100);
    digitalWrite(LED_PIN, LOW);
    delay(100);
  }
  
  Serial.println("✓ Ready! Waiting for motor movement...");
  Serial.println();
}

// ===== Main Loop =====
void loop() {
  // Calculate RPM
  calculateRPM();
  
  // Process serial commands
  processCommand();
  
  // Continuous printing
  if (continuousPrint && (millis() - lastPrintTime >= PRINT_INTERVAL)) {
    printRPM();
    lastPrintTime = millis();
  }
  
  // LED stays on if motor is running (RPM > 5)
  static unsigned long lastLEDUpdate = 0;
  if (millis() - lastLEDUpdate > 1000) {
    digitalWrite(LED_PIN, (currentRPM > 5.0f) ? HIGH : LOW);
    lastLEDUpdate = millis();
  }
}