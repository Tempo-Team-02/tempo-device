#include <Wire.h>
#include <SPI.h>
#include <SD.h>

#include <ICM_20948.h>                        // SparkFun ICM-20948 library
#include <SparkFun_u-blox_GNSS_Arduino_Library.h> // SparkFun u-blox GNSS
#include <TinyGPSPlus.h>

// ---------------------- PIN DEFINITIONS ----------------------
#define PIN_SD_CS        10
#define PIN_STARTSTOP     9
#define PIN_GPS_RX        1   // GPS TX -> Feather RX
#define PIN_GPS_TX        0   // GPS RX <- Feather TX
#define PIN_GPS_PPS       5

// ---------------------- GLOBAL OBJECTS -----------------------
ICM_20948_I2C imu;
SFE_UBLOX_GNSS gps;       // used to configure the MAX-M10S
TinyGPSPlus tinyGPS;      // used to parse NMEA from Serial1

File logFile;

bool loggingEnabled = false;
unsigned long lastIMUTime = 0;

// -------------------------------------------------------------
void setup() {
  Serial.begin(115200);
  delay(2000);

  Serial.println("Starting system...");

  // -------- BUTTON --------
  pinMode(PIN_STARTSTOP, INPUT_PULLUP);
  pinMode(PIN_GPS_PPS, INPUT);

  // -------- I2C (IMU + GPS I2C control if needed) --------
  Wire.setSDA(4);
  Wire.setSCL(5);
  Wire.begin();

  // -------- IMU Initialization --------
  ICM_20948_Status_e imuStatus;
  imuStatus = imu.begin(Wire, ICM_20948_I2C_ADDR_AD0);  // typical address 0x68
  if (imuStatus != ICM_20948_Stat_Ok) {
    Serial.print("IMU not detected. Status: ");
    Serial.println(imuStatus);
    while (1);
  }
  Serial.println("IMU OK (ICM-20948).");

  // -------- GPS Initialization (UART) --------
  Serial1.setRX(PIN_GPS_RX);
  Serial1.setTX(PIN_GPS_TX);
  Serial1.begin(9600);

  if (!gps.begin(Serial1)) {
    Serial.println("GPS not detected!");
    while (1);
  }
  Serial.println("GPS OK (u-blox MAX-M10S).");

  // Set GPS nav rate to 10 Hz (if supported)
  gps.setNavigationFrequency(10);

  // -------- SD Card Initialization (using SD library) --------
  if (!SD.begin(PIN_SD_CS)) {
    Serial.println("SD init failed!");
    while (1);
  }
  Serial.println("SD OK.");

  // Prepare log file
  logFile = SD.open("log.csv", FILE_WRITE);
  if (!logFile) {
    Serial.println("Cannot create log file!");
    while (1);
  }
  logFile.println("time_ms,ax,ay,az,gx,gy,gz,mx,my,mz,lat,lon,alt,speed_kmh,sats");
  logFile.flush();

  Serial.println("System initialized. Press Start button to begin logging.");
}

// -------------------------------------------------------------
void loop() {
  // Handle Start/Stop button (simple toggle with debounce)
  static bool lastButtonState = HIGH;
  bool currentState = digitalRead(PIN_STARTSTOP);

  if (currentState == LOW && lastButtonState == HIGH) {
    delay(50); // debounce
    if (digitalRead(PIN_STARTSTOP) == LOW) {
      loggingEnabled = !loggingEnabled;
      Serial.println(loggingEnabled ? "LOGGING STARTED" : "LOGGING STOPPED");
    }
  }
  lastButtonState = currentState;

  if (!loggingEnabled) {
    // Still feed GPS parser so it stays updated
    while (Serial1.available()) {
      tinyGPS.encode(Serial1.read());
    }
    return;
  }

  unsigned long now = millis();

  // -------- IMU @ ~100 Hz --------
  if (now - lastIMUTime >= 10) { // 10 ms = 100 Hz
    lastIMUTime = now;

    if (imu.dataReady()) {
      imu.getAGMT();  // read all accel, gyro, mag, temp

      float ax = imu.accX();
      float ay = imu.accY();
      float az = imu.accZ();
      float gx = imu.gyrX();
      float gy = imu.gyrY();
      float gz = imu.gyrZ();
      float mx = imu.magX();
      float my = imu.magY();
      float mz = imu.magZ();

      // -------- GPS reading --------
      while (Serial1.available()) {
        tinyGPS.encode(Serial1.read());
      }

      double lat  = tinyGPS.location.isValid() ? tinyGPS.location.lat() : 0.0;
      double lon  = tinyGPS.location.isValid() ? tinyGPS.location.lng() : 0.0;
      double alt  = tinyGPS.altitude.isValid() ? tinyGPS.altitude.meters() : 0.0;
      double spd  = tinyGPS.speed.isValid()    ? tinyGPS.speed.kmph()     : 0.0;
      int    sats = tinyGPS.satellites.isValid()? tinyGPS.satellites.value() : 0;

      // -------- WRITE CSV --------
      logFile.print(now);
      logFile.print(',');
      logFile.print(ax);  logFile.print(',');
      logFile.print(ay);  logFile.print(',');
      logFile.print(az);  logFile.print(',');
      logFile.print(gx);  logFile.print(',');
      logFile.print(gy);  logFile.print(',');
      logFile.print(gz);  logFile.print(',');
      logFile.print(mx);  logFile.print(',');
      logFile.print(my);  logFile.print(',');
      logFile.print(mz);  logFile.print(',');
      logFile.print(lat, 7); logFile.print(',');
      logFile.print(lon, 7); logFile.print(',');
      logFile.print(alt); logFile.print(',');
      logFile.print(spd); logFile.print(',');
      logFile.println(sats);
      logFile.flush();
    }
  }
}
