#include <Wire.h>
#include <ICM_20948.h>
#include <SparkFun_I2C_GPS_Arduino_Library.h>
#include <TinyGPSPlus.h>
#include <FS.h>
#include <SD.h>
#include <SPI.h>

ICM_20948_I2C imu;
I2CGPS myI2CGPS;
TinyGPSPlus gps;
SPIClass sdSPI(VSPI);

// I2C pins
#define SDA_PIN 21
#define SCL_PIN 22

// SD SPI pins
#define SD_CS   5
#define SD_SCK  18
#define SD_MISO 19
#define SD_MOSI 23

// Control pins
#define RECORD_BUTTON 32   // pull-down
#define RECORD_LED    25   // blinking while recording
#define GPS_LED       33   // blink until fix, then solid

// Timing
const unsigned long imuInterval = 10; // 100 Hz
const unsigned long recordBlinkInterval = 300;
const unsigned long gpsBlinkInterval = 500;
const unsigned long debounceDelay = 30;

// States
bool imuOK = false;
bool gpsOK = false;
bool recording = false;

// Button
bool buttonState = LOW;
bool lastButtonReading = LOW;
unsigned long lastDebounceTime = 0;

// Timing trackers
unsigned long prevIMU = 0;
unsigned long sampleCount = 0;
unsigned long trialNumber = 0;

// LED states
unsigned long lastRecordBlink = 0;
bool recordLedState = false;

unsigned long lastGpsBlink = 0;
bool gpsLedState = false;

// File
File dataFile;
char filename[20];

// GPS State
struct GpsState {
  bool timeValid;
  bool locationValid;
  int hour, minute, second;
  double lat, lon, alt;
};

GpsState gpsState = {false, false, 0, 0, 0, 0.0, 0.0, 0.0};

// ---------- GPS UPDATE ----------
void updateGPS() {
  while (myI2CGPS.available()) {
    gps.encode(myI2CGPS.read());
  }

  if (gps.time.isValid()) {
    gpsState.timeValid = true;
    gpsState.hour = gps.time.hour();
    gpsState.minute = gps.time.minute();
    gpsState.second = gps.time.second();
  }

  if (gps.location.isValid()) {
    gpsState.locationValid = true;
    gpsState.lat = gps.location.lat();
    gpsState.lon = gps.location.lng();
  } else {
    gpsState.locationValid = false;
  }

  if (gps.altitude.isValid()) {
    gpsState.alt = gps.altitude.meters();
  }
}

// ---------- GPS LED ----------
void updateGPSLED() {
  unsigned long now = millis();

  if (gpsState.locationValid) {
    digitalWrite(GPS_LED, HIGH); // solid when real fix
  } else {
    if (now - lastGpsBlink >= gpsBlinkInterval) {
      lastGpsBlink = now;
      gpsLedState = !gpsLedState;
      digitalWrite(GPS_LED, gpsLedState);
    }
  }
}

// ---------- RECORD LED ----------
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

// ---------- FILE ----------
bool createTrialFile() {
  for (int i = 1; i < 1000; i++) {
    snprintf(filename, sizeof(filename), "/TRIAL%03d.CSV", i);

    if (!SD.exists(filename)) {
      dataFile = SD.open(filename, FILE_WRITE);
      if (!dataFile) return false;

      trialNumber = i;

      dataFile.println("trial,sample,millis,hour,minute,second,lat,lon,alt,ax,ay,az,gx,gy,gz,mx,my,mz");
      dataFile.flush();
      return true;
    }
  }
  return false;
}

// ---------- RECORD CONTROL ----------
void startRecording() {
  if (createTrialFile()) {
    recording = true;
    sampleCount = 0;

    lastRecordBlink = millis();
    recordLedState = true;
    digitalWrite(RECORD_LED, HIGH);

    Serial.print("Recording started: ");
    Serial.println(filename);
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

// ---------- BUTTON ----------
void handleButton() {
  bool reading = digitalRead(RECORD_BUTTON);

  if (reading != lastButtonReading) {
    lastDebounceTime = millis();
  }

  if ((millis() - lastDebounceTime) > debounceDelay) {
    if (reading != buttonState) {
      buttonState = reading;

      if (buttonState == HIGH) { // pressed
        if (!recording) startRecording();
        else stopRecording();
      }
    }
  }

  lastButtonReading = reading;
}

// ---------- LOGGING ----------
void logIMUSample() {
  if (!recording || !imuOK || !dataFile) return;
  if (!imu.dataReady()) return;

  imu.getAGMT();

  dataFile.print(trialNumber); dataFile.print(",");
  dataFile.print(sampleCount++); dataFile.print(",");
  dataFile.print(millis()); dataFile.print(",");

  if (gpsState.timeValid) {
    dataFile.print(gpsState.hour); dataFile.print(",");
    dataFile.print(gpsState.minute); dataFile.print(",");
    dataFile.print(gpsState.second); dataFile.print(",");
  } else {
    dataFile.print("-1,-1,-1,");
  }

  if (gpsState.locationValid) {
    dataFile.print(gpsState.lat, 7); dataFile.print(",");
    dataFile.print(gpsState.lon, 7); dataFile.print(",");
    dataFile.print(gpsState.alt); dataFile.print(",");
  } else {
    dataFile.print("0,0,0,");
  }

  dataFile.print(imu.accX()); dataFile.print(",");
  dataFile.print(imu.accY()); dataFile.print(",");
  dataFile.print(imu.accZ()); dataFile.print(",");
  dataFile.print(imu.gyrX()); dataFile.print(",");
  dataFile.print(imu.gyrY()); dataFile.print(",");
  dataFile.print(imu.gyrZ()); dataFile.print(",");
  dataFile.print(imu.magX()); dataFile.print(",");
  dataFile.print(imu.magY()); dataFile.print(",");
  dataFile.println(imu.magZ());

  if ((sampleCount % 25) == 0) {
    dataFile.flush();
  }
}

// ---------- SETUP ----------
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

  // IMU
  ICM_20948_Status_e status = imu.begin(Wire, ICM_20948_I2C_ADDR_AD1);
  if (status != ICM_20948_Stat_Ok)
    status = imu.begin(Wire, ICM_20948_I2C_ADDR_AD0);

  imuOK = (status == ICM_20948_Stat_Ok);

  // GPS
  gpsOK = myI2CGPS.begin();

  // SD
  sdSPI.begin(SD_SCK, SD_MISO, SD_MOSI, SD_CS);
  if (!SD.begin(SD_CS, sdSPI)) {
    Serial.println("SD Card Mount Failed");
    while (1);
  }

  Serial.println("System Ready");
}

// ---------- LOOP ----------
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
