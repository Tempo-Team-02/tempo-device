#include <Wire.h>
#include <ICM_20948.h>                        // IMU
#include <SparkFun_u-blox_GNSS_Arduino_Library.h> // GPS

// --------- Global objects ---------
ICM_20948_I2C imu;
SFE_UBLOX_GNSS gps;

bool imuOK = false;
bool gpsOK = false;

#define RAD_TO_DEG 57.2957795f

// -------------------------------------------------------------
void setup() {
  Serial.begin(115200);
  delay(2000);

  Serial.println();
  Serial.println("==== Feather RP2040 USB Host (STEMMA QT IMU+GPS) ====");
  Serial.println("Printing IMU+GPS+Orientation once per second...\n");

  // Start I²C on STEMMA QT pins
  Wire.begin();

  // -------- Initialize IMU (ICM-20948) --------
  Serial.println("Initializing IMU (ICM-20948)...");
  ICM_20948_Status_e imuStatus;

  imuStatus = imu.begin(Wire, ICM_20948_I2C_ADDR_AD1); // 0x69
  if (imuStatus == ICM_20948_Stat_Ok) {
    imuOK = true;
    Serial.println("IMU detected at I2C address 0x69.");
  } else {
    Serial.println("IMU not found at 0x69, trying 0x68...");
    imuStatus = imu.begin(Wire, ICM_20948_I2C_ADDR_AD0); // 0x68
    if (imuStatus == ICM_20948_Stat_Ok) {
      imuOK = true;
      Serial.println("IMU detected at I2C address 0x68.");
    } else {
      Serial.print("IMU failed, status = ");
      Serial.println(imuStatus);
      imuOK = false;
    }
  }

  // -------- Initialize GPS (MAX-M10S over I²C) --------
  Serial.println("Initializing GPS (MAX-M10S) over I2C (0x42)...");
  if (gps.begin(Wire) == false) {
    Serial.println("GPS NOT detected on I2C. gpsOK = false.");
    gpsOK = false;
  } else {
    gpsOK = true;
    Serial.println("GPS detected at address 0x42.");
    gps.setI2COutput(COM_TYPE_UBX);  // reduce NMEA spam
    gps.setNavigationFrequency(1);   // 1 Hz update
  }

  Serial.println("\nInitialization complete.\n");
}

// -------------------------------------------------------------
void loop() {
  static unsigned long lastPrint = 0;
  unsigned long now = millis();

  if (now - lastPrint < 1000) return;
  lastPrint = now;

  // ---------- VARIABLES TO POPULATE ----------
  float ax_raw=0, ay_raw=0, az_raw=0;
  float ax_ms2=0, ay_ms2=0, az_ms2=0;
  float gx=0, gy=0, gz=0;
  float mx=0, my=0, mz=0;
  double lat=0, lon=0;
  float alt_m=0, speed_kmh=0;
  uint8_t sats=0;
  float yaw_deg=0, pitch_deg=0, roll_deg=0;

  // ============================================================
  //  IMU SECTION (Accel / Gyro / Mag + Orientation)
  // ============================================================
  if (imuOK && imu.dataReady()) {

    imu.getAGMT(); // Read accel, gyro, mag

    // ---- RAW accelerometer (mg) ----
    ax_raw = imu.accX();
    ay_raw = imu.accY();
    az_raw = imu.accZ();

    // ---- Convert mg → m/s² ----
    ax_ms2 = ax_raw * 9.81f / 1000.0f;
    ay_ms2 = ay_raw * 9.81f / 1000.0f;
    az_ms2 = az_raw * 9.81f / 1000.0f;

    gx = imu.gyrX();
    gy = imu.gyrY();
    gz = imu.gyrZ();

    mx = imu.magX();
    my = imu.magY();
    mz = imu.magZ();

    // ---- Orientation estimation (simple tilt + compass) ----
    // 1) Compute pitch and roll from accelerometer
    // Convert accel to "g" units first (just for clarity)
    float ax_g = ax_raw / 1000.0f;
    float ay_g = ay_raw / 1000.0f;
    float az_g = az_raw / 1000.0f;

    // pitch: rotation around Y axis
    float pitch_rad = atan2f(-ax_g, sqrtf(ay_g*ay_g + az_g*az_g));
    // roll: rotation around X axis
    float roll_rad  = atan2f(ay_g, az_g);

    pitch_deg = pitch_rad * RAD_TO_DEG;
    roll_deg  = roll_rad  * RAD_TO_DEG;

    // 2) Tilt-compensated compass heading (yaw)
    // Convert roll/pitch to radians (already are) and apply tilt compensation
    float sinRoll  = sinf(roll_rad);
    float cosRoll  = cosf(roll_rad);
    float sinPitch = sinf(pitch_rad);
    float cosPitch = cosf(pitch_rad);

    // Tilt compensation of magnetometer
    float mx_comp = mx * cosPitch + mz * sinPitch;
    float my_comp = mx * sinRoll * sinPitch + my * cosRoll - mz * sinRoll * cosPitch;

    float yaw_rad = atan2f(-my_comp, mx_comp); // negative sign to match compass-like heading
    yaw_deg = yaw_rad * RAD_TO_DEG;
    if (yaw_deg < 0) yaw_deg += 360.0f;       // normalize to 0–360
  }

  // ============================================================
  //  GPS SECTION
  // ============================================================
  if (gpsOK && gps.getPVT()) {

    int32_t rawLat    = gps.getLatitude();      // 1e-7 degrees
    int32_t rawLon    = gps.getLongitude();
    int32_t rawAltMM  = gps.getAltitude();      // mm
    int32_t rawSpdMMs = gps.getGroundSpeed();   // mm/s

    lat       = rawLat / 1e7;
    lon       = rawLon / 1e7;
    alt_m     = rawAltMM  / 1000.0f;
    speed_kmh = (rawSpdMMs / 1000.0f) * 3.6f;
    sats      = gps.getSIV();
  }

  // ============================================================
  //  SERIAL OUTPUT
  // ============================================================
  Serial.println("--------------------------------------------------");
  Serial.print("Time (ms): ");
  Serial.println(now);

  Serial.println("IMU:");
  Serial.print("  Accel RAW (mg): ");
  Serial.print(ax_raw); Serial.print(", ");
  Serial.print(ay_raw); Serial.print(", ");
  Serial.println(az_raw);

  Serial.print("  Accel (m/s^2): ");
  Serial.print(ax_ms2,4); Serial.print(", ");
  Serial.print(ay_ms2,4); Serial.print(", ");
  Serial.println(az_ms2,4);

  Serial.print("  Gyro (dps): ");
  Serial.print(gx); Serial.print(", ");
  Serial.print(gy); Serial.print(", ");
  Serial.println(gz);

  Serial.print("  Mag (uT): ");
  Serial.print(mx); Serial.print(", ");
  Serial.print(my); Serial.print(", ");
  Serial.println(mz);

  Serial.println("Orientation (deg):");
  Serial.print("  Yaw (heading): ");
  Serial.println(yaw_deg, 2);
  Serial.print("  Pitch: ");
  Serial.println(pitch_deg, 2);
  Serial.print("  Roll: ");
  Serial.println(roll_deg, 2);

  Serial.println("GPS:");
  Serial.print("  Lat, Lon: ");
  Serial.print(lat,7); Serial.print(", ");
  Serial.println(lon,7);
  Serial.print("  Alt (m): ");
  Serial.println(alt_m);
  Serial.print("  Speed (km/h): ");
  Serial.println(speed_kmh);
  Serial.print("  Satellites: ");
  Serial.println(sats);

  // ============================================================
  //  CSV OUTPUT (copy/paste directly into Excel!) (later integration on microsd)
  //  Format:
  //  CSV,time_ms,ax_raw,ay_raw,az_raw,ax_ms2,ay_ms2,az_ms2,gx,gy,gz,
  //      mx,my,mz,lat,lon,alt_m,speed_kmh,sats,yaw_deg,pitch_deg,roll_deg
  // ============================================================
  Serial.print("CSV,");
  Serial.print(now); Serial.print(",");
  Serial.print(ax_raw); Serial.print(",");
  Serial.print(ay_raw); Serial.print(",");
  Serial.print(az_raw); Serial.print(",");
  Serial.print(ax_ms2); Serial.print(",");
  Serial.print(ay_ms2); Serial.print(",");
  Serial.print(az_ms2); Serial.print(",");
  Serial.print(gx); Serial.print(",");
  Serial.print(gy); Serial.print(",");
  Serial.print(gz); Serial.print(",");
  Serial.print(mx); Serial.print(",");
  Serial.print(my); Serial.print(",");
  Serial.print(mz); Serial.print(",");
  Serial.print(lat,7); Serial.print(",");
  Serial.print(lon,7); Serial.print(",");
  Serial.print(alt_m); Serial.print(",");
  Serial.print(speed_kmh); Serial.print(",");
  Serial.print(sats); Serial.print(",");
  Serial.print(yaw_deg); Serial.print(",");
  Serial.print(pitch_deg); Serial.print(",");
  Serial.println(roll_deg);
}
