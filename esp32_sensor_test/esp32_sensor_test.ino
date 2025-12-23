/*
 * ============================================
 * BEETLE ROBOT - ESP32 WROOM SENSOR TEST
 * ============================================
 * 
 * Sensors:
 *   - MPU6050 (I2C)
 *   - 6 Encoders (2 pins each - A/B channels)
 *   - 7 VL53L0X ToF Sensors (I2C with XSHUT pins)
 * 
 * Data Format sent to dashboard:
 *   ax,ay,az,gx,gy,gz,enc1,enc2,enc3,enc4,enc5,enc6,tof1,tof2,tof3,tof4,tof5,tof6,tof7
 * 
 * Libraries Required:
 *   - Adafruit MPU6050
 *   - Adafruit VL53L0X
 *   - WiFi (built-in)
 *   - HTTPClient (built-in)
 */

#include <Wire.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_VL53L0X.h>

// ============================================
// CONFIGURATION
// ============================================

// WiFi Credentials
const char* WIFI_SSID = "YOUR_WIFI_SSID";
const char* WIFI_PASSWORD = "YOUR_WIFI_PASSWORD";

// Dashboard Server
const char* SERVER_URL = "http://YOUR_COMPUTER_IP:5000/update";
const int SEND_INTERVAL_MS = 100;  // Send data every 100ms

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
bool wifiConnected = false;

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

void setupWiFi() {
  Serial.print("Connecting to WiFi");
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  
  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 30) {
    delay(500);
    Serial.print(".");
    attempts++;
  }
  
  if (WiFi.status() == WL_CONNECTED) {
    wifiConnected = true;
    Serial.println("\n✓ WiFi Connected!");
    Serial.print("  IP Address: ");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println("\n✗ WiFi Failed! Running in offline mode.");
  }
}

void setupMPU() {
  Serial.print("Initializing MPU6050...");
  
  if (mpu.begin()) {
    mpuConnected = true;
    
    // Configure MPU6050
    mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
    
    Serial.println(" ✓ Connected!");
    Serial.println("  Accel Range: ±8G");
    Serial.println("  Gyro Range: ±500°/s");
  } else {
    Serial.println(" ✗ Failed!");
    Serial.println("  Check wiring: SDA=21, SCL=22, VCC=3.3V");
  }
}

void setupEncoders() {
  Serial.println("Initializing Encoders...");
  
  for (int i = 0; i < 6; i++) {
    pinMode(ENC_A_PINS[i], INPUT_PULLUP);
    pinMode(ENC_B_PINS[i], INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(ENC_A_PINS[i]), encoderISRs[i], CHANGE);
    Serial.printf("  Encoder %d: Pins %d/%d ✓\n", i+1, ENC_A_PINS[i], ENC_B_PINS[i]);
  }
  
  Serial.println("  All encoders initialized!");
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
  for (int i = 0; i < 7; i++) {
    // Enable this sensor
    digitalWrite(TOF_XSHUT_PINS[i], HIGH);
    delay(10);
    
    // Initialize with default address first
    if (tof[i].begin(0x29)) {
      // Change to unique address
      if (tof[i].setAddress(TOF_ADDRESSES[i])) {
        tofConnected[i] = true;
        Serial.printf("  ToF %d: Address 0x%02X ✓\n", i+1, TOF_ADDRESSES[i]);
      }
    } else {
      Serial.printf("  ToF %d: ✗ Not found\n", i+1);
    }
  }
  
  // Count connected sensors
  int connectedCount = 0;
  for (int i = 0; i < 7; i++) {
    if (tofConnected[i]) connectedCount++;
  }
  Serial.printf("  %d/7 ToF sensors connected\n", connectedCount);
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
// DATA TRANSMISSION
// ============================================

void sendDataToServer() {
  if (!wifiConnected || WiFi.status() != WL_CONNECTED) {
    return;
  }
  
  // Build data string
  String data = String(ax, 2) + "," +
                String(ay, 2) + "," +
                String(az, 2) + "," +
                String(gx, 2) + "," +
                String(gy, 2) + "," +
                String(gz, 2) + "," +
                String(enc[0]) + "," +
                String(enc[1]) + "," +
                String(enc[2]) + "," +
                String(enc[3]) + "," +
                String(enc[4]) + "," +
                String(enc[5]) + "," +
                String(tofDist[0]) + "," +
                String(tofDist[1]) + "," +
                String(tofDist[2]) + "," +
                String(tofDist[3]) + "," +
                String(tofDist[4]) + "," +
                String(tofDist[5]) + "," +
                String(tofDist[6]);
  
  HTTPClient http;
  http.begin(SERVER_URL);
  http.addHeader("Content-Type", "text/plain");
  
  int responseCode = http.POST(data);
  
  if (responseCode != 200) {
    Serial.printf("HTTP Error: %d\n", responseCode);
  }
  
  http.end();
}

void printDebugData() {
  Serial.println("\n========== SENSOR DATA ==========");
  
  Serial.println("--- MPU6050 ---");
  Serial.printf("  Accel: X=%.2f Y=%.2f Z=%.2f m/s²\n", ax, ay, az);
  Serial.printf("  Gyro:  X=%.2f Y=%.2f Z=%.2f °/s\n", gx, gy, gz);
  
  Serial.println("--- Encoders ---");
  Serial.printf("  E1=%ld E2=%ld E3=%ld E4=%ld E5=%ld E6=%ld\n", 
                enc[0], enc[1], enc[2], enc[3], enc[4], enc[5]);
  
  Serial.println("--- ToF Sensors (mm) ---");
  Serial.printf("  T1=%d T2=%d T3=%d T4=%d T5=%d T6=%d T7=%d\n",
                tofDist[0], tofDist[1], tofDist[2], tofDist[3], 
                tofDist[4], tofDist[5], tofDist[6]);
  
  Serial.println("==================================");
}

// ============================================
// MAIN SETUP
// ============================================

void setup() {
  Serial.begin(115200);
  delay(1000);
  
  Serial.println("\n");
  Serial.println("╔═══════════════════════════════════════════╗");
  Serial.println("║     BEETLE ROBOT - ESP32 SENSOR TEST      ║");
  Serial.println("╚═══════════════════════════════════════════╝");
  Serial.println();
  
  // Initialize I2C
  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(400000);  // 400kHz I2C
  
  // Setup all components
  setupWiFi();
  Serial.println();
  setupMPU();
  Serial.println();
  setupEncoders();
  Serial.println();
  setupToFSensors();
  Serial.println();
  
  Serial.println("╔═══════════════════════════════════════════╗");
  Serial.println("║           SETUP COMPLETE!                 ║");
  Serial.println("╚═══════════════════════════════════════════╝");
  Serial.println("\nStarting sensor readings...\n");
}

// ============================================
// MAIN LOOP
// ============================================

void loop() {
  // Read all sensors
  readMPU();
  readEncoders();
  readToF();
  
  // Send to server at specified interval
  unsigned long currentTime = millis();
  if (currentTime - lastSendTime >= SEND_INTERVAL_MS) {
    lastSendTime = currentTime;
    
    sendDataToServer();
    
    // Print debug output every 500ms (every 5th send)
    static int printCounter = 0;
    if (++printCounter >= 5) {
      printCounter = 0;
      printDebugData();
    }
  }
  
  // Small delay to prevent overwhelming the CPU
  delay(5);
}
