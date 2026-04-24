#include <Wire.h>
#include <ICM_20948.h>
#include <SparkFun_I2C_GPS_Arduino_Library.h>
#include <TinyGPSPlus.h>
#include "FS.h"
#include "SD_MMC.h"

ICM_20948_I2C imu;
I2CGPS myI2CGPS;
TinyGPSPlus gps;

#define SDA_PIN 23
#define SCL_PIN 22

// SDMMC custom pins
#define SD_CLK 15
#define SD_CMD 19
#define SD_D0 2
#define SD_D1 4
#define SD_D2 12
#define SD_D3 13

bool imuOK = false;
bool gpsOK = false;

unsigned long prevIMU = 0;
unsigned long prevGPS = 0;

const unsigned long imuInterval = 10;   // 100 Hz
const unsigned long gpsInterval = 200;  // 5 Hz

File dataFile;

// Latest GPS values
double lat = 0, lon = 0, alt = 0;
void setup() {
Serial.begin(115200);
  delay(2000);

  // I2C INIT
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000);

  // IMU INIT
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

  // GPS INIT
  if (myI2CGPS.begin()) {
    gpsOK = true;
    Serial.println("GPS detected.");
  } else {
    Serial.println("GPS not detected.");
  }

  // SD INIT 4-bit 
  SD_MMC.setPins(SD_CLK, SD_CMD, SD_D0, SD_D1, SD_D2, SD_D3);

  if (!SD_MMC.begin("/sdcard", true)) { // true = 4-bit mode
    Serial.println("SD Card Mount Failed");
    return;
  }

  Serial.println("SD Card Initialized");

  // Create file
  dataFile = SD_MMC.open("/data.csv", FILE_WRITE);
  if (dataFile) {
    dataFile.println("time,ax,ay,az,gx,gy,gz,mx,my,mz,lat,lon,alt");
    dataFile.close();
  }
}



void loop() {
 unsigned long now = millis();

  // ---------- IMU ----------
  if (now - prevIMU >= imuInterval) {
    prevIMU = now;

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
    Serial.print("ACC [");
    Serial.print(ax, 3); Serial.print(", ");
    Serial.print(ay, 3); Serial.print(", ");
    Serial.print(az, 3); Serial.println("]");

    Serial.print("GYRO [");
    Serial.print(gx, 3); Serial.print(", ");
    Serial.print(gy, 3); Serial.print(", ");
    Serial.print(gz, 3); Serial.println("]");

    Serial.print("MAG [");
    Serial.print(mx, 3); Serial.print(", ");
    Serial.print(my, 3); Serial.print(", ");
    Serial.print(mz, 3); Serial.println("]");

    Serial.println("------------------------");

    // Write to SD
    dataFile = SD_MMC.open("/data.csv", FILE_APPEND);
    if (dataFile) {
      dataFile.print(now);
      dataFile.print(",");

      dataFile.print(ax); dataFile.print(",");
      dataFile.print(ay); dataFile.print(",");
      dataFile.print(az); dataFile.print(",");

      dataFile.print(gx); dataFile.print(",");
      dataFile.print(gy); dataFile.print(",");
      dataFile.print(gz); dataFile.print(",");

      dataFile.print(mx); dataFile.print(",");
      dataFile.print(my); dataFile.print(",");
      dataFile.print(mz); dataFile.print(",");

      dataFile.print(lat, 7); dataFile.print(",");
      dataFile.print(lon, 7); dataFile.print(",");
      dataFile.println(alt);

      dataFile.close();
    }
  }

  // ---------- GPS ----------
  if (now - prevGPS >= gpsInterval) {
    prevGPS = now;

    if (myI2CGPS.available()) {
      lat = gps.location.lat();
      lon = gps.location.lng();
      alt = gps.altitude.meters();

      Serial.print("Lat: "); Serial.print(lat, 7);
      Serial.print(" Lon: "); Serial.print(lon, 7);
      Serial.print(" Alt: "); Serial.println(alt);
    } else {
      Serial.println("No GPS fix yet...");
    }
  }
}


