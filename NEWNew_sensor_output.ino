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

unsigned long prevIMU = 0;
unsigned long prevGPS = 0;

const unsigned long imuInterval = 10;   // 10 ms = 100 Hz
const unsigned long gpsInterval = 200;  // 200 ms = 5 Hz

void setup(){
Serial.begin(115200);
delay(2000);
Wire.begin(SDA_PIN, SCL_PIN);
Wire.setClock(400000);
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

  // ---- GPS INIT 
  if (myI2CGPS.begin()) {
    gpsOK = true;
    Serial.println("GPS detected.");
  } else {
    Serial.println("GPS not detected.");
  }
}
void loop()
{
  unsigned long now = millis();

  // ---------- IMU SECTION ----------
  if (now - prevIMU >= imuInterval)
  {
    prevIMU = now;

    imu.getAGMT();

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

  // ---------- GPS SECTION ----------
  if (now - prevGPS >= gpsInterval)
  {
    prevGPS = now;

    if (myI2CGPS.available())
    {
      Serial.print("Lat: ");
      Serial.print(gps.location.lat(), 7);
      Serial.print(" Lon: ");
      Serial.print(gps.location.lng(), 7);
      Serial.print(" Alt(m) ");
      Serial.print(gps.altitude.meters());
      Serial.println();
    }
      else { 
        Serial.println("No GPS fix yet...");

      }
    }
  }
