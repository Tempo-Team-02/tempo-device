#include <Wire.h>
#include <ICM_20948.h>
#include <SparkFun_I2C_GPS_Arduino_Library.h>
#include <TinyGPSPlus.h>

ICM_20948_I2C imu;
I2CGPS myI2CGPS;
TinyGPSPlus gps;

#define SDA_PIN 23
#define SCL_PIN 22
#define RAD_TO_DEG 57.2957795f

bool imuOK = false;
bool gpsOK = false;

void setup() {
  Serial.begin(115200);
  delay(2000);

  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000);

  // ---- IMU INIT (UNCHANGED STRUCTURE) ----
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

  // ---- GPS INIT (ADAPTED FOR XA1110) ----
  if (myI2CGPS.begin()) {
    gpsOK = true;
    Serial.println("GPS detected.");
  } else {
    Serial.println("GPS not detected.");
  }
}

void loop() {

  // ---- IMU SECTION (UNCHANGED OUTPUT STYLE) ----
if (imuOK) {

  // Always read the full sensor set
  imu.getAGMT();

  // Accelerometer
  Serial.print("Accel (g): ");
  Serial.print(imu.accX());
  Serial.print(", ");
  Serial.print(imu.accY());
  Serial.print(", ");
  Serial.println(imu.accZ());

  // Gyroscope
  Serial.print("Gyro (dps): ");
  Serial.print(imu.gyrX());
  Serial.print(", ");
  Serial.print(imu.gyrY());
  Serial.print(", ");
  Serial.println(imu.gyrZ());

  // Magnetometer
  Serial.print("Mag (uT): ");
  Serial.print(imu.magX());
  Serial.print(", ");
  Serial.print(imu.magY());
  Serial.print(", ");
  Serial.println(imu.magZ());
}

  // ---- GPS SECTION (OUTPUT FORMAT PRESERVED) ----
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
  delay(1000);  // SAME TIMING STRUCTURE
}