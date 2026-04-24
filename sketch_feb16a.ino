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

// FIXED I2C PINS 
#define SDA_PIN 21
#define SCL_PIN 22

// SPI SD pins
#define SD_CS   5
#define SD_SCK  18
#define SD_MISO 19
#define SD_MOSI 23

const int bufferSize = 50;

struct DataRow {
  unsigned long t;
  float ax, ay, az;
  float gx, gy, gz;
  float mx, my, mz;
  double lat, lon;
  float alt;
};

DataRow buffer[bufferSize];
int bufferIndex = 0;

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

  //  SPI INIT
  SPI.begin(SD_SCK, SD_MISO, SD_MOSI, SD_CS);

  if (!SD.begin(SD_CS)) {
    Serial.println("SD Card Mount Failed");
    while (1);
  }

  if (SD.cardType() == CARD_NONE) {
    Serial.println("No SD card attached");
    while (1);
  }

  Serial.println("SD Card Initialized");

  // Create file
  dataFile = SD.open("/data.csv", FILE_WRITE);

  if (!dataFile) {
    Serial.println("Failed to open file");
    while (1);
  }

  // Write header once
  if (dataFile.size() == 0) {
    dataFile.println("time,ax,ay,az,gx,gy,gz,mx,my,mz,lat,lon,alt");
    dataFile.flush();
  }

  Serial.println("File ready for logging");
}

void loop() {
  unsigned long now = millis();

  // ---------- IMU ----------
  if (now - prevIMU >= imuInterval) {
    prevIMU = now;

    imu.getAGMT();

    buffer[bufferIndex++] = {
      now,
      imu.accX(), imu.accY(), imu.accZ(),
      imu.gyrX(), imu.gyrY(), imu.gyrZ(),
      imu.magX(), imu.magY(), imu.magZ(),
      lat, lon, alt
    };
  }

  // ---------- WRITE BUFFER ----------
  if (bufferIndex >= bufferSize) {
    if (dataFile) {
      for (int i = 0; i < bufferSize; i++) {
        dataFile.print(buffer[i].t); dataFile.print(",");

        dataFile.print(buffer[i].ax); dataFile.print(",");
        dataFile.print(buffer[i].ay); dataFile.print(",");
        dataFile.print(buffer[i].az); dataFile.print(",");

        dataFile.print(buffer[i].gx); dataFile.print(",");
        dataFile.print(buffer[i].gy); dataFile.print(",");
        dataFile.print(buffer[i].gz); dataFile.print(",");

        dataFile.print(buffer[i].mx); dataFile.print(",");
        dataFile.print(buffer[i].my); dataFile.print(",");
        dataFile.print(buffer[i].mz); dataFile.print(",");

        dataFile.print(buffer[i].lat, 7); dataFile.print(",");
        dataFile.print(buffer[i].lon, 7); dataFile.print(",");
        dataFile.println(buffer[i].alt);
      }

      dataFile.flush();
    } else {
      Serial.println("Write failed!");
    }

    bufferIndex = 0;
  }

  // ---------- GPS ----------
  if (now - prevGPS >= gpsInterval) {
    prevGPS = now;

    if (myI2CGPS.available()) {
      lat = gps.location.lat();
      lon = gps.location.lng();
      alt = gps.altitude.meters();

      
      Serial.print("GPS: ");
      Serial.print(lat, 6);
      Serial.print(", ");
      Serial.println(lon, 6);
    }
  }
}
