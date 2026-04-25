#include <Wire.h>
#include <ICM_20948.h>
#include <SparkFun_I2C_GPS_Arduino_Library.h>
#include <TinyGPSPlus.h>
#include <FS.h>
#include <SD.h>
#include <SPI.h>
#include <math.h>

ICM_20948_I2C imu;
I2CGPS myI2CGPS;
TinyGPSPlus gps;
SPIClass sdSPI(VSPI);

// Pins
#define SDA_PIN 21
#define SCL_PIN 22

#define SD_CS   5
#define SD_SCK  18
#define SD_MISO 19
#define SD_MOSI 23

#define RECORD_BUTTON 32
#define RECORD_LED    25
#define GPS_LED       33

// Timing
const unsigned long imuInterval = 10;              // 100 Hz
const unsigned long recordBlinkInterval = 300;
const unsigned long gpsBlinkInterval = 500;
const unsigned long debounceDelay = 30;

// States
bool imuOK = false;
bool recording = false;

// Button
bool buttonState = LOW;
bool lastButtonReading = LOW;
unsigned long lastDebounceTime = 0;

// Timing trackers
unsigned long prevIMU = 0;
unsigned long sampleCount = 0;
unsigned long trialNumber = 0;

// LEDs
unsigned long lastRecordBlink = 0;
bool recordLedState = false;

unsigned long lastGpsBlink = 0;
bool gpsLedState = false;

// File
File dataFile;
char filename[20];

// GPS STATE
struct GpsState {
  bool timeValid;
  bool dateValid;
  bool locationValid;
  bool altitudeValid;
  int year, month, day;
  int hour, minute, second, centisecond;
  double lat, lon, alt;
};

GpsState gpsState = {false, false, false, false, 0, 0, 0, 0, 0, 0, 0, 0.0, 0.0, 0.0};

// FILTERED IMU
struct FilteredIMU {
  float ax, ay, az;
  float gx, gy, gz;
  float mx, my, mz;
  bool initialized;
};

FilteredIMU filt = {0, 0, 0, 0, 0, 0, 0, 0, 0, false};

// Filter constants
const float alphaAccel = 0.20f;
const float alphaGyro  = 0.20f;
const float alphaMag   = 0.15f;

// Symmetry scaling
const float maxTiltDeg = 30.0f;

// Upright calibration
float rollOffset = 0.0f;
float pitchOffset = 0.0f;
bool postureCalibrated = false;

// Helpers
float lowPass(float input, float prev, float alpha) {
  return alpha * input + (1.0f - alpha) * prev;
}

float clampFloat(float x, float minVal, float maxVal) {
  if (x < minVal) return minVal;
  if (x > maxVal) return maxVal;
  return x;
}

// GPS UPDATE
void updateGPS() {
  while (myI2CGPS.available()) {
    gps.encode(myI2CGPS.read());
  }

  gpsState.timeValid = gps.time.isValid();
  gpsState.dateValid = gps.date.isValid();
  gpsState.locationValid = gps.location.isValid();
  gpsState.altitudeValid = gps.altitude.isValid();

  if (gpsState.timeValid) {
    gpsState.hour = gps.time.hour();
    gpsState.minute = gps.time.minute();
    gpsState.second = gps.time.second();
    gpsState.centisecond = gps.time.centisecond();
  }

  if (gpsState.dateValid) {
    gpsState.year = gps.date.year();
    gpsState.month = gps.date.month();
    gpsState.day = gps.date.day();
  }

  if (gpsState.locationValid) {
    gpsState.lat = gps.location.lat();
    gpsState.lon = gps.location.lng();
  }

  if (gpsState.altitudeValid) {
    gpsState.alt = gps.altitude.meters();
  }
}

// LEDS
void updateGPSLED() {
  unsigned long now = millis();

  if (gpsState.locationValid && gpsState.timeValid) {
    digitalWrite(GPS_LED, HIGH);
  } else {
    if (now - lastGpsBlink >= gpsBlinkInterval) {
      lastGpsBlink = now;
      gpsLedState = !gpsLedState;
      digitalWrite(GPS_LED, gpsLedState);
    }
  }
}

void updateRecordLED() {
  unsigned long now = millis();

  if (recording) {
    if (now - lastRecordBlink >= recordBlinkInterval) {
      lastRecordBlink = now;
      recordLedState = !recordLedState;
      digitalWrite(RECORD_LED, recordLedState);
    }
  } else {
    recordLedState = false;
    digitalWrite(RECORD_LED, LOW);
  }
}

// FILE
bool createTrialFile() {
  for (int i = 1; i < 1000; i++) {
    snprintf(filename, sizeof(filename), "/TRIAL%03d.CSV", i);

    if (!SD.exists(filename)) {
      dataFile = SD.open(filename, FILE_WRITE);
      if (!dataFile) return false;

      trialNumber = i;

      dataFile.println("trial,sample,millis,year,month,day,hour,minute,second,centisecond,gps_time_valid,gps_fix_valid,lat,lon,alt,ax,ay,az,gx,gy,gz,mx,my,mz,roll_deg,pitch_deg,left_right_score,forward_back_score,overall_balance_score");
      dataFile.flush();
      return true;
    }
  }
  return false;
}

// RECORD CONTROL
void startRecording() {
  if (createTrialFile()) {
    recording = true;
    sampleCount = 0;
    filt.initialized = false;
    postureCalibrated = false;

    lastRecordBlink = millis();
    recordLedState = true;
    digitalWrite(RECORD_LED, HIGH);

    Serial.print("Recording started: ");
    Serial.println(filename);
    Serial.println("Hold upright briefly for posture calibration.");
  }
}

void stopRecording() {
  if (dataFile) {
    dataFile.flush();
    dataFile.close();
  }

  recording = false;
  digitalWrite(RECORD_LED, LOW);
  Serial.println("Recording stopped");
}

// BUTTON
void handleButton() {
  bool reading = digitalRead(RECORD_BUTTON);

  if (reading != lastButtonReading) {
    lastDebounceTime = millis();
  }

  if ((millis() - lastDebounceTime) > debounceDelay) {
    if (reading != buttonState) {
      buttonState = reading;

      if (buttonState == HIGH) {
        if (!recording) startRecording();
        else stopRecording();
      }
    }
  }

  lastButtonReading = reading;
}

// FILTER UPDATE
void updateFilteredIMU(float ax, float ay, float az,
                       float gx, float gy, float gz,
                       float mx, float my, float mz) {
  if (!filt.initialized) {
    filt.ax = ax; filt.ay = ay; filt.az = az;
    filt.gx = gx; filt.gy = gy; filt.gz = gz;
    filt.mx = mx; filt.my = my; filt.mz = mz;
    filt.initialized = true;
    return;
  }

  filt.ax = lowPass(ax, filt.ax, alphaAccel);
  filt.ay = lowPass(ay, filt.ay, alphaAccel);
  filt.az = lowPass(az, filt.az, alphaAccel);

  filt.gx = lowPass(gx, filt.gx, alphaGyro);
  filt.gy = lowPass(gy, filt.gy, alphaGyro);
  filt.gz = lowPass(gz, filt.gz, alphaGyro);

  filt.mx = lowPass(mx, filt.mx, alphaMag);
  filt.my = lowPass(my, filt.my, alphaMag);
  filt.mz = lowPass(mz, filt.mz, alphaMag);
}

// UPRIGHT CALIBRATION
void calibratePostureReference() {
  if (!filt.initialized) return;

  float rollDeg = atan2(filt.ay, sqrt(filt.ax * filt.ax + filt.az * filt.az)) * 180.0 / PI;
  float pitchDeg = atan2(-filt.ax, sqrt(filt.ay * filt.ay + filt.az * filt.az)) * 180.0 / PI;

  rollOffset = rollDeg;
  pitchOffset = pitchDeg;
  postureCalibrated = true;

  Serial.print("Posture calibrated. Roll offset: ");
  Serial.print(rollOffset);
  Serial.print(" Pitch offset: ");
  Serial.println(pitchOffset);
}

// LOGGING
void logIMUSample() {
  if (!recording || !imuOK || !dataFile || !imu.dataReady()) return;

  imu.getAGMT();

  float ax = imu.accX();
  float ay = imu.accY();
  float az = imu.accZ();

  float gx = imu.gyrX();
  float gy = imu.gyrY();
  float gz = imu.gyrZ();

  float mx = imu.magX();
  float my = imu.magY();
  float mz = imu.magZ();

  updateFilteredIMU(ax, ay, az, gx, gy, gz, mx, my, mz);

  // If not calibrated yet, keep collecting filtered data but don't trust posture scores yet
  if (!postureCalibrated && filt.initialized && sampleCount > 20) {
    calibratePostureReference();
  }

  float rollDeg = atan2(filt.ay, sqrt(filt.ax * filt.ax + filt.az * filt.az)) * 180.0 / PI;
  float pitchDeg = atan2(-filt.ax, sqrt(filt.ay * filt.ay + filt.az * filt.az)) * 180.0 / PI;

  if (postureCalibrated) {
    rollDeg -= rollOffset;
    pitchDeg -= pitchOffset;
  }

  float lr = clampFloat(rollDeg / maxTiltDeg, -1.0f, 1.0f);
  float fb = clampFloat(pitchDeg / maxTiltDeg, -1.0f, 1.0f);
  float balance = clampFloat(1.0f - ((fabs(lr) + fabs(fb)) / 2.0f), 0.0f, 1.0f);

  dataFile.print(trialNumber); dataFile.print(",");
  dataFile.print(sampleCount++); dataFile.print(",");
  dataFile.print(millis()); dataFile.print(",");

  // GPS date
  if (gpsState.dateValid) {
    dataFile.print(gpsState.year); dataFile.print(",");
    dataFile.print(gpsState.month); dataFile.print(",");
    dataFile.print(gpsState.day); dataFile.print(",");
  } else {
    dataFile.print("-1,-1,-1,");
  }

  // GPS time (UTC)
  if (gpsState.timeValid) {
    dataFile.print(gpsState.hour); dataFile.print(",");
    dataFile.print(gpsState.minute); dataFile.print(",");
    dataFile.print(gpsState.second); dataFile.print(",");
    dataFile.print(gpsState.centisecond); dataFile.print(",");
  } else {
    dataFile.print("-1,-1,-1,-1,");
  }

  // Valid flags
  dataFile.print(gpsState.timeValid ? 1 : 0); dataFile.print(",");
  dataFile.print(gpsState.locationValid ? 1 : 0); dataFile.print(",");

  // GPS location
  if (gpsState.locationValid) {
    dataFile.print(gpsState.lat, 7); dataFile.print(",");
    dataFile.print(gpsState.lon, 7); dataFile.print(",");
  } else {
    dataFile.print("-999,-999,");
  }

  // GPS altitude
  if (gpsState.altitudeValid) {
    dataFile.print(gpsState.alt);
  } else {
    dataFile.print("-999");
  }
  dataFile.print(",");

  // Filtered IMU only
  dataFile.print(filt.ax); dataFile.print(",");
  dataFile.print(filt.ay); dataFile.print(",");
  dataFile.print(filt.az); dataFile.print(",");
  dataFile.print(filt.gx); dataFile.print(",");
  dataFile.print(filt.gy); dataFile.print(",");
  dataFile.print(filt.gz); dataFile.print(",");
  dataFile.print(filt.mx); dataFile.print(",");
  dataFile.print(filt.my); dataFile.print(",");
  dataFile.print(filt.mz); dataFile.print(",");

  // Symmetry / posture scores
  dataFile.print(rollDeg); dataFile.print(",");
  dataFile.print(pitchDeg); dataFile.print(",");
  dataFile.print(lr); dataFile.print(",");
  dataFile.print(fb); dataFile.print(",");
  dataFile.println(balance);

  if (sampleCount % 25 == 0) {
    dataFile.flush();
  }
}

// SETUP
void setup() {
  Serial.begin(115200);
  delay(1000);

  pinMode(RECORD_BUTTON, INPUT);
  pinMode(RECORD_LED, OUTPUT);
  pinMode(GPS_LED, OUTPUT);

  digitalWrite(RECORD_LED, LOW);
  digitalWrite(GPS_LED, LOW);

  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000);

  // IMU init: try both possible addresses
  ICM_20948_Status_e status = imu.begin(Wire, ICM_20948_I2C_ADDR_AD1);
  if (status != ICM_20948_Stat_Ok) {
    status = imu.begin(Wire, ICM_20948_I2C_ADDR_AD0);
  }
  imuOK = (status == ICM_20948_Stat_Ok);

  if (!imuOK) {
    Serial.println("IMU init failed");
  } else {
    Serial.println("IMU ready");
  }

  // GPS init
  if (myI2CGPS.begin()) {
    Serial.println("GPS ready");
  } else {
    Serial.println("GPS init failed");
  }

  // SD init
  sdSPI.begin(SD_SCK, SD_MISO, SD_MOSI, SD_CS);
  if (!SD.begin(SD_CS, sdSPI)) {
    Serial.println("SD Card Mount Failed");
    while (1);
  }

  Serial.println("System Ready");
}

// LOOP
void loop() {
  updateGPS();
  updateGPSLED();
  handleButton();
  updateRecordLED();

  if (recording && millis() - prevIMU >= imuInterval) {
    prevIMU = millis();
    logIMUSample();
  }
}
