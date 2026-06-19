#include <ESP32Servo.h>
#include <Wire.h>

// ============================================================================
// --- PHYSICAL HARDWARE CONFIGURATION ---
// ============================================================================
const int PIN_RX_LEFT_IN   = 25;  // Receiver CH3
const int PIN_RX_RIGHT_IN  = 26;  // Receiver CH4
const int PIN_RX_SWITCH_IN = 27;  // Receiver CH5

const int PIN_ESC_LEFT_OUT  = 18; 
const int PIN_ESC_RIGHT_OUT = 19; 

const int PWM_NEUTRAL = 1500;     
const int MPU_ADDR    = 0x68;     

// ============================================================================
// --- TUNING PARAMETERS ---
// ============================================================================
const float Kp = 1.5;             
const float Kd = 0.4;             
const float GYRO_DEADBAND = 1.2;  

// ============================================================================
// --- VOLATILE INTERRUPT REGISTERS (NON-BLOCKING) ---
// ============================================================================
volatile unsigned long rx_left_start  = 0;
volatile int rx_left_val              = 1500;
volatile unsigned long rx_right_start = 0;
volatile int rx_right_val             = 1500;
volatile unsigned long rx_switch_start= 0;
volatile int rx_switch_val            = 1000;

unsigned long lastValidSignalTime = 0;
unsigned long lastLoopTime = 0;

int lastOutputLeft  = 1500;
int lastOutputRight = 1500;
const int MAX_SLEW_RATE = 180;    

bool gyroEnabled = false;

// IMU Memory
int16_t rawGyroZ = 0;
float gyroZ_degPerSec = 0.0;
float gyroZ_bias = 0.0;           
float lastGyroZ_degPerSec = 0.0;

Servo escLeft;
Servo escRight;

// ============================================================================
// --- HARDWARE INTERRUPT SERVICE ROUTINES (ISRs) ---
// ============================================================================
void IRAM_ATTR ISR_LEFT_CH() {
  unsigned long current_time = micros();
  if (digitalRead(PIN_RX_LEFT_IN) == HIGH) {
    rx_left_start = current_time;
  } else {
    int width = (int)(current_time - rx_left_start);
    if (width >= 900 && width <= 2100) {
      rx_left_val = width;
    }
  }
}

void IRAM_ATTR ISR_RIGHT_CH() {
  unsigned long current_time = micros();
  if (digitalRead(PIN_RX_RIGHT_IN) == HIGH) {
    rx_right_start = current_time;
  } else {
    int width = (int)(current_time - rx_right_start);
    if (width >= 900 && width <= 2100) {
      rx_right_val = width;
    }
  }
}

void IRAM_ATTR ISR_SWITCH_CH() {
  unsigned long current_time = micros();
  if (digitalRead(PIN_RX_SWITCH_IN) == HIGH) {
    rx_switch_start = current_time;
  } else {
    int width = (int)(current_time - rx_switch_start);
    if (width >= 900 && width <= 2100) {
      rx_switch_val = width;
    }
  }
}

// ============================================================================
// --- INITIALIZATION ---
// ============================================================================
void setup() {
  Serial.begin(115200);
  
  escLeft.attach(PIN_ESC_LEFT_OUT, 1000, 2000);
  escRight.attach(PIN_ESC_RIGHT_OUT, 1000, 2000);
  
  escLeft.writeMicroseconds(PWM_NEUTRAL);
  escRight.writeMicroseconds(PWM_NEUTRAL);

  pinMode(PIN_RX_LEFT_IN, INPUT_PULLUP);
  pinMode(PIN_RX_RIGHT_IN, INPUT_PULLUP);
  pinMode(PIN_RX_SWITCH_IN, INPUT_PULLUP);

  // Attach background hardware interrupts to pin state changes
  attachInterrupt(digitalPinToInterrupt(PIN_RX_LEFT_IN),   ISR_LEFT_CH,   CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_RX_RIGHT_IN),  ISR_RIGHT_CH,  CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_RX_SWITCH_IN), ISR_SWITCH_CH, CHANGE);

  Wire.begin(21, 22, 400000); 

  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B); 
  Wire.write(0x00); 
  if (Wire.endTransmission() != 0) {
    Serial.println("[ERROR] MPU-9250 Initial Connection Failed!");
  }

  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x1B); 
  Wire.write(0x08); 
  Wire.endTransmission();

  // Calibration background sample loop
  long biasAccumulator = 0;
  int validSamples = 0;
  for (int i = 0; i < 150; i++) {
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x47); 
    if(Wire.endTransmission(false) == 0) {
      Wire.requestFrom(MPU_ADDR, 2, true);
      int16_t sample = (Wire.read() << 8) | Wire.read();
      if(sample != -32768 && sample != 32767) {
        biasAccumulator += sample;
        validSamples++;
      }
    }
    delay(10); 
  }
  gyroZ_bias = (validSamples > 0) ? ((float)biasAccumulator / (float)validSamples) : 0.0;
  lastValidSignalTime = millis();
  Serial.println("[SYSTEM] Interrupt Architecture Configured. Locked.");
}

// ============================================================================
// --- MAIN PROCESSING CONTROL LOOP (NON-BLOCKING) ---
// ============================================================================
void loop() {
  unsigned long currentTime = millis();
  
  // Enforce rigid 20ms frame updates
  if (currentTime - lastLoopTime >= 20) { 
    lastLoopTime = currentTime;

    // Local snapshots safely uncoupled from background interrupt modifications
    int rawLeftPulse;
    int rawRightPulse;
    int rawSwitchPulse;

    // Prevent race conditions while copying multi-byte volatile memory
    noInterrupts();
    rawLeftPulse   = rx_left_val;
    rawRightPulse  = rx_right_val;
    rawSwitchPulse = rx_switch_val;
    interrupts();

    // Check if the background interrupt parameters have updated recently
    static int prevLeftPulse = 1500;
    static int prevRightPulse = 1500;
    
    if (rawLeftPulse != prevLeftPulse || rawRightPulse != prevRightPulse) {
      lastValidSignalTime = currentTime;
      prevLeftPulse = rawLeftPulse;
      prevRightPulse = rawRightPulse;
    }

    int targetLeft  = PWM_NEUTRAL;
    int targetRight = PWM_NEUTRAL;

    // HARD WATCHDOG RECOVERY: Signal dead-lock breaker
    if (currentTime - lastValidSignalTime > 150) {
      rawLeftPulse    = PWM_NEUTRAL;
      rawRightPulse   = PWM_NEUTRAL;
      lastOutputLeft  = PWM_NEUTRAL;
      lastOutputRight = PWM_NEUTRAL;
      Serial.println("!!! WATCHDOG ACTIVE -> RECOVERY FORCE TO NEUTRAL !!!");
    }

    gyroEnabled = (rawSwitchPulse > 1600);

    // --- NON-BLOCKING REGISTER READ FROM IMU ---
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x47); 
    if (Wire.endTransmission(false) != 0) {
      Wire.begin(21, 22, 400000); 
      gyroZ_degPerSec = 0.0;
    } else {
      Wire.requestFrom(MPU_ADDR, 2, true);
      if(Wire.available() >= 2) {
        rawGyroZ = (Wire.read() << 8) | Wire.read();
        if (rawGyroZ == -32768 || rawGyroZ == 32767) {
          gyroZ_degPerSec = 0.0; 
        } else {
          gyroZ_degPerSec = ((float)rawGyroZ - gyroZ_bias) / 65.5;
        }
      }
    }

    if (abs(gyroZ_degPerSec) < GYRO_DEADBAND) {
      gyroZ_degPerSec = 0.0;
    }

    // --- DIRECTIONAL ASSIGNMENT MIX MATRIX ---
    int invertedLeft  = 3000 - rawLeftPulse;
    int invertedRight = 3000 - rawRightPulse;
    
    targetLeft  = invertedRight;
    targetRight = invertedLeft;

    bool sticksAreCentered = (abs(invertedLeft - PWM_NEUTRAL) < 45 && abs(invertedRight - PWM_NEUTRAL) < 45);

    // --- CLOSED LOOP GYRO INJECTION LOOP (PHASE CORRECTED) ---
    if (gyroEnabled && !sticksAreCentered) {
      // FIX: Flipped from (targetLeft - targetRight) to map stick intent properly
      float userIntendedYawRate = (targetRight - targetLeft) * 0.35; 

      float yawError = userIntendedYawRate - gyroZ_degPerSec;
      float gyroDerivative = gyroZ_degPerSec - lastGyroZ_degPerSec;
      
      float feedbackCorrection = (yawError * Kp) - (gyroDerivative * Kd);

      if (abs(gyroZ_degPerSec) > 3.0 && abs(feedbackCorrection) < 40.0) {
         feedbackCorrection = (gyroZ_degPerSec > 0) ? -40.0 : 40.0;
      }

      // Negative Feedback Correction Vector (Flipped)
      targetLeft  -= (int)feedbackCorrection; 
      targetRight += (int)feedbackCorrection; 
    }
    
    lastGyroZ_degPerSec = gyroZ_degPerSec;

    if (sticksAreCentered) {
      targetLeft  = PWM_NEUTRAL;
      targetRight = PWM_NEUTRAL;
    }

    // --- SLEW DRIVER STEP RATE LIMITERS ---
    int deltaLeft = targetLeft - lastOutputLeft;
    if (targetLeft == PWM_NEUTRAL) {
      lastOutputLeft = PWM_NEUTRAL;
    } else if (abs(deltaLeft) > MAX_SLEW_RATE) {
      lastOutputLeft += (deltaLeft > 0) ? MAX_SLEW_RATE : -MAX_SLEW_RATE;
    } else {
      lastOutputLeft = targetLeft;
    }

    int deltaRight = targetRight - lastOutputRight;
    if (targetRight == PWM_NEUTRAL) {
      lastOutputRight = PWM_NEUTRAL;
    } else if (abs(deltaRight) > MAX_SLEW_RATE) {
      lastOutputRight += (deltaRight > 0) ? MAX_SLEW_RATE : -MAX_SLEW_RATE;
    } else {
      lastOutputRight = targetRight;
    }

    lastOutputLeft  = constrain(lastOutputLeft, 1000, 2000);
    lastOutputRight = constrain(lastOutputRight, 1000, 2000);

    escLeft.writeMicroseconds(lastOutputLeft);
    escRight.writeMicroseconds(lastOutputRight);
    
    // Telemetry trace log
    Serial.print("MODE: ");        Serial.print(gyroEnabled ? "STABILIZED" : "MANUAL");
    Serial.print(" | GYRO: ");     Serial.print(gyroZ_degPerSec, 1);
    Serial.print(" || L_OUT: ");   Serial.print(lastOutputLeft);
    Serial.print(" | R_OUT: ");    Serial.println(lastOutputRight);
  }
}
