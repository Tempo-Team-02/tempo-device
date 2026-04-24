// ======================= LIBRARIES =======================
#include <Wire.h>
#include <ICM_20948.h>
#include <SparkFun_I2C_GPS_Arduino_Library.h>
#include <TinyGPSPlus.h>

// ======================= OBJECTS =========================
ICM_20948_I2C imu;
I2CGPS myI2CGPS;
TinyGPSPlus gps;

// ======================= CONFIGURATION ===================
#define SDA_PIN 23
#define SCL_PIN 22
#define BUTTON_PIN 4          // GPIO pin connected to button
#define RAD_TO_DEG 57.2957795f

// ======================= STATUS FLAGS ====================
bool imuOK = false;
bool gpsOK = false;
bool recording = false;      // TRUE = actively collecting data

// ======================= BUTTON VARIABLES ================
bool lastButtonState = HIGH; // Assume pull-up (not pressed = HIGH)
unsigned long lastDebounceTime = 0;
const unsigned long debounceDelay = 50; // ms

// ========================================================
// ======================= SETUP ==========================
// ========================================================
void setup() {

  Serial.begin(115200);
  delay(2000);

  // ---------- I2C INIT ----------
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000);

  // ---------- BUTTON INIT ----------
  pinMode(BUTTON_PIN, INPUT_PULLUP); // Internal pull-up resistor

  // ======================================================
  // -------------------- IMU SETUP -----------------------
  // ======================================================
  ICM_20948_Status_e status;

  status = imu.begin(Wire, ICM_20948_I2C_ADDR_AD1);
  if (status != ICM_20948_Stat_Ok)
    status = imu.begin(Wire, ICM_20948_I2C_ADDR_AD0);

  if (status == ICM_20948_Stat_Ok) {
    imuOK = true;
    Serial.println("IMU detected.");
  } else {
    Serial.println("IMU not detected.");
  }

  // ======================================================
  // -------------------- GPS SETUP -----------------------
  // ======================================================
  if (myI2CGPS.begin()) {
    gpsOK = true;
    Serial.println("GPS detected.");
  } else {
    Serial.println("GPS not detected.");
  }

  Serial.println("Press button to START/STOP recording.");
}

// ========================================================
// ======================= BUTTON LOGIC ===================
// ========================================================
void handleButton() {

  bool reading = digitalRead(BUTTON_PIN);

  // If button state changed, reset debounce timer
  if (reading != lastButtonState) {
    lastDebounceTime = millis();
  }

  // If stable for debounce delay → accept change
  if ((millis() - lastDebounceTime) > debounceDelay) {

    // Detect button press (HIGH → LOW transition)
    if (lastButtonState == HIGH && reading == LOW) {

      recording = !recording; // TOGGLE STATE

      if (recording) {
        Serial.println("=== RECORDING STARTED ===");
      } else {
        Serial.println("=== RECORDING STOPPED ===");
      }
    }
  }

  lastButtonState = reading;
}

// ========================================================
// ======================= MAIN LOOP ======================
// ========================================================
void loop() {

  // Always check button state
  handleButton();

  // Only run sensor reading if recording is ON
  if (recording) {

    // ====================================================
    // -------------------- IMU SECTION -------------------
    // ====================================================
    if (imuOK) {

      imu.getAGMT();

      Serial.print("Accel (g): ");
      Serial.print(imu.accX());
      Serial.print(", ");
      Serial.print(imu.accY());
      Serial.print(", ");
      Serial.println(imu.accZ());

      Serial.print("Gyro (dps): ");
      Serial.print(imu.gyrX());
      Serial.print(", ");
      Serial.print(imu.gyrY());
      Serial.print(", ");
      Serial.println(imu.gyrZ());

      Serial.print("Mag (uT): ");
      Serial.print(imu.magX());
      Serial.print(", ");
      Serial.print(imu.magY());
      Serial.print(", ");
      Serial.println(imu.magZ());
    }

    // ====================================================
    // -------------------- GPS SECTION -------------------
    // ====================================================
    if (gpsOK) {

      while (myI2CGPS.available()) {
        char c = myI2CGPS.read();
        gps.encode(c);
      }

      if (gps.location.isValid()) {

        Serial.print("Lat: ");
        Serial.print(gps.location.lat(), 7);

        Serial.print(" Lon: ");
        Serial.print(gps.location.lng(), 7);

        if (gps.altitude.isValid()) {
          Serial.print(" Alt(m): ");
          Serial.print(gps.altitude.meters());
        }

        Serial.println();

      } else {
        Serial.println("No GPS fix yet...");
      }
    }

    Serial.println("---------------------");
    delay(1000); // Maintain original timing
  }
}