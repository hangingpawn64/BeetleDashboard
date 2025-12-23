/*
 * ============================================
 * BEETLE ROBOT - ESP32 SERIAL SENSOR OUTPUT
 * ============================================
 * 
 * Sensors:
 *   - MPU6050 (I2C)
 *   - 6 Encoders (2 pins each - A/B channels)
 *   - 7 VL53L0X ToF Sensors (I2C with XSHUT pins)
 * 
 * Data Format (Serial output):
 *   ax,ay,az,gx,gy,gz,enc1,enc2,enc3,enc4,enc5,enc6,tof1,tof2,tof3,tof4,tof5,tof6,tof7
 * 
 * Libraries Required:
 *   - Adafruit MPU6050
 *   - Adafruit VL53L0X
 */

#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_VL53L0X.h>

// ============================================
// CONFIGURATION
// ============================================

#define SERIAL_BAUD 115200
#define SEND_INTERVAL_MS 100  // Send data every 100ms

// ============================================
// PIN DEFINITIONS
// ============================================

// I2C Pins
#define I2C_SDA 21
#define I2C_SCL 22

// Encoder Pins (using interrupt-capable pins)
// Each encoder has A and B channels
#define ENC1_A 32
#define ENC1_B 33
#define ENC2_A 25
#define ENC2_B 26
#define ENC3_A 27
#define ENC3_B 14
#define ENC4_A 12
#define ENC4_B 13
#define ENC5_A 15
#define ENC5_B 2
#define ENC6_A 4
#define ENC6_B 16

// ToF XSHUT Pins (for address assignment)
#define TOF1_XSHUT 17
#define TOF2_XSHUT 5
#define TOF3_XSHUT 18
#define TOF4_XSHUT 19
#define TOF5_XSHUT 23
#define TOF6_XSHUT 34  // Input only - may need different pin
#define TOF7_XSHUT 35  // Input only - may need different pin

// ============================================
// SENSOR OBJECTS
// ============================================

Adafruit_MPU6050 mpu;

// ToF Sensors with unique addresses
Adafruit_VL53L0X tof[7];
const uint8_t TOF_ADDRESSES[7] = {0x30, 0x31, 0x32, 0x33, 0x34, 0x35, 0x36};
const uint8_t TOF_XSHUT_PINS[7] = {TOF1_XSHUT, TOF2_XSHUT, TOF3_XSHUT, TOF4_XSHUT, TOF5_XSHUT, TOF6_XSHUT, TOF7_XSHUT};

// ============================================
// ENCODER VARIABLES (Volatile for ISR)
// ============================================

volatile long encoderCounts[6] = {0, 0, 0, 0, 0, 0};

// Encoder pin arrays
const uint8_t ENC_A_PINS[6] = {ENC1_A, ENC2_A, ENC3_A, ENC4_A, ENC5_A, ENC6_A};
const uint8_t ENC_B_PINS[6] = {ENC1_B, ENC2_B, ENC3_B, ENC4_B, ENC5_B, ENC6_B};

// ============================================
// SENSOR DATA
// ============================================

float ax, ay, az;  // Acceleration in m/s²
float gx, gy, gz;  // Angular velocity in °/s
long enc[6];       // Encoder counts
int tofDist[7];    // ToF distances in mm

bool mpuConnected = false;
bool tofConnected[7] = {false};

unsigned long lastSendTime = 0;

// ============================================
// ENCODER ISR HANDLERS
// ============================================

void IRAM_ATTR encoderISR0() {
  if (digitalRead(ENC_A_PINS[0]) == digitalRead(ENC_B_PINS[0])) {
    encoderCounts[0]++;
  } else {
    encoderCounts[0]--;
  }
}

void IRAM_ATTR encoderISR1() {
  if (digitalRead(ENC_A_PINS[1]) == digitalRead(ENC_B_PINS[1])) {
    encoderCounts[1]++;
  } else {
    encoderCounts[1]--;
  }
}

void IRAM_ATTR encoderISR2() {
  if (digitalRead(ENC_A_PINS[2]) == digitalRead(ENC_B_PINS[2])) {
    encoderCounts[2]++;
  } else {
    encoderCounts[2]--;
  }
}

void IRAM_ATTR encoderISR3() {
  if (digitalRead(ENC_A_PINS[3]) == digitalRead(ENC_B_PINS[3])) {
    encoderCounts[3]++;
  } else {
    encoderCounts[3]--;
  }
}

void IRAM_ATTR encoderISR4() {
  if (digitalRead(ENC_A_PINS[4]) == digitalRead(ENC_B_PINS[4])) {
    encoderCounts[4]++;
  } else {
    encoderCounts[4]--;
  }
}

void IRAM_ATTR encoderISR5() {
  if (digitalRead(ENC_A_PINS[5]) == digitalRead(ENC_B_PINS[5])) {
    encoderCounts[5]++;
  } else {
    encoderCounts[5]--;
  }
}

// Array of ISR function pointers
void (*encoderISRs[6])() = {encoderISR0, encoderISR1, encoderISR2, encoderISR3, encoderISR4, encoderISR5};

// ============================================
// SETUP FUNCTIONS
// ============================================

void setupMPU() {
  Serial.print("Initializing MPU6050...");
  
  if (mpu.begin()) {
    mpuConnected = true;
    
    // Configure MPU6050
    mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
    
    Serial.println(" OK");
  } else {
    Serial.println(" FAILED");
  }
}

void setupEncoders() {
  Serial.println("Initializing Encoders...");
  
  for (int i = 0; i < 6; i++) {
    pinMode(ENC_A_PINS[i], INPUT_PULLUP);
    pinMode(ENC_B_PINS[i], INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(ENC_A_PINS[i]), encoderISRs[i], CHANGE);
  }
  
  Serial.println("Encoders OK");
}

void setupToFSensors() {
  Serial.println("Initializing ToF Sensors...");
  
  // First, set all XSHUT pins LOW to disable all sensors
  for (int i = 0; i < 7; i++) {
    pinMode(TOF_XSHUT_PINS[i], OUTPUT);
    digitalWrite(TOF_XSHUT_PINS[i], LOW);
  }
  delay(10);
  
  // Enable and configure each sensor one by one
  int connectedCount = 0;
  for (int i = 0; i < 7; i++) {
    // Enable this sensor
    digitalWrite(TOF_XSHUT_PINS[i], HIGH);
    delay(10);
    
    // Initialize with default address first
    if (tof[i].begin(0x29)) {
      // Change to unique address
      if (tof[i].setAddress(TOF_ADDRESSES[i])) {
        tofConnected[i] = true;
        connectedCount++;
      }
    }
  }
  
  Serial.print("ToF: ");
  Serial.print(connectedCount);
  Serial.println("/7 connected");
}

// ============================================
// READ FUNCTIONS
// ============================================

void readMPU() {
  if (!mpuConnected) {
    ax = ay = az = gx = gy = gz = 0;
    return;
  }
  
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  
  // Acceleration in m/s²
  ax = a.acceleration.x;
  ay = a.acceleration.y;
  az = a.acceleration.z;
  
  // Angular velocity in °/s (convert from rad/s)
  gx = g.gyro.x * 57.2958;  // RAD_TO_DEG
  gy = g.gyro.y * 57.2958;
  gz = g.gyro.z * 57.2958;
}

void readEncoders() {
  // Copy volatile counts to regular array
  noInterrupts();
  for (int i = 0; i < 6; i++) {
    enc[i] = encoderCounts[i];
  }
  interrupts();
}

void readToF() {
  for (int i = 0; i < 7; i++) {
    if (tofConnected[i]) {
      VL53L0X_RangingMeasurementData_t measure;
      tof[i].rangingTest(&measure, false);
      
      if (measure.RangeStatus != 4) {  // Phase failure = no reading
        tofDist[i] = measure.RangeMilliMeter;
      } else {
        tofDist[i] = 8190;  // Out of range
      }
    } else {
      tofDist[i] = 0;
    }
  }
}

// ============================================
// SERIAL OUTPUT
// ============================================

void sendSerialData() {
  // Format: ax,ay,az,gx,gy,gz,enc1,enc2,enc3,enc4,enc5,enc6,tof1,tof2,tof3,tof4,tof5,tof6,tof7
  // Prefix with "DATA:" for easy parsing
  Serial.print("DATA:");
  Serial.print(ax, 2); Serial.print(",");
  Serial.print(ay, 2); Serial.print(",");
  Serial.print(az, 2); Serial.print(",");
  Serial.print(gx, 2); Serial.print(",");
  Serial.print(gy, 2); Serial.print(",");
  Serial.print(gz, 2); Serial.print(",");
  Serial.print(enc[0]); Serial.print(",");
  Serial.print(enc[1]); Serial.print(",");
  Serial.print(enc[2]); Serial.print(",");
  Serial.print(enc[3]); Serial.print(",");
  Serial.print(enc[4]); Serial.print(",");
  Serial.print(enc[5]); Serial.print(",");
  Serial.print(tofDist[0]); Serial.print(",");
  Serial.print(tofDist[1]); Serial.print(",");
  Serial.print(tofDist[2]); Serial.print(",");
  Serial.print(tofDist[3]); Serial.print(",");
  Serial.print(tofDist[4]); Serial.print(",");
  Serial.print(tofDist[5]); Serial.print(",");
  Serial.println(tofDist[6]);
}

// ============================================
// MAIN SETUP
// ============================================

void setup() {
  Serial.begin(SERIAL_BAUD);
  delay(1000);
  
  Serial.println();
  Serial.println("=== BEETLE ESP32 SENSOR NODE ===");
  Serial.println();
  
  // Initialize I2C
  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(400000);  // 400kHz I2C
  
  // Setup all components
  setupMPU();
  setupEncoders();
  setupToFSensors();
  
  Serial.println();
  Serial.println("=== SETUP COMPLETE ===");
  Serial.println("Sending data as: DATA:ax,ay,az,gx,gy,gz,enc1-6,tof1-7");
  Serial.println();
}

// ============================================
// MAIN LOOP
// ============================================

void loop() {
  // Read all sensors
  readMPU();
  readEncoders();
  readToF();
  
  // Send data at specified interval
  unsigned long currentTime = millis();
  if (currentTime - lastSendTime >= SEND_INTERVAL_MS) {
    lastSendTime = currentTime;
    sendSerialData();
  }
  
  // Small delay to prevent overwhelming the CPU
  delay(5);
}
