#include <SPI.h>
#include <SD.h>

SPIClass sdSPI(VSPI);

#define SD_CS   5
#define SD_SCK  18
#define SD_MISO 19
#define SD_MOSI 23
#define SD_DET 4

void setup() {
  Serial.begin(115200);
  delay(2000);
  Serial.println("Starting SD test");

  sdSPI.begin(SD_SCK, SD_MISO, SD_MOSI, SD_CS);
  SD.begin(SD_CS, sdSPI);
  
  if (!sdSPI.begin(SD_SCK, SD_MISO, SD_MOSI, SD_CS)) {
    Serial.println("SD FAIL");
    return;
  }

  Serial.println("SD OK");
  
    File f = SD.open("/test.txt", FILE_WRITE);
  
  if (f) {
    f.println("Testing");
    f.close();
    Serial.println("Write success");
  }
else {
  Serial.println("Open failed");
}
}

void loop() {}