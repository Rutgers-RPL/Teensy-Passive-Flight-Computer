#include <Arduino.h>
#include <BMI088.h>
#include <SdFat.h>

typedef struct realPacket {
  int magic;
  float time;
  float latitude;
  float longitude;
  float altitude;
  float accx;
  float accy;
  float accz;
  float avelx;
  float avely;
  float avelz;
  float temp;
  float pressure;
};

/* accel object */
Bmi088Accel accel(Wire,0x18);
/* gyro object */
Bmi088Gyro gyro(Wire,0x68);

SdFs sd;
FsFile file;

unsigned long offset = 0;
unsigned long previousLog = 0;
unsigned long previousTime = 0;

void setup() {
  int status;
  Serial.begin(115200);

  while(!Serial) {}
  /* start the sensors */
  status = accel.begin();
  if (status < 0) {
    Serial.println("Accel Initialization Error");
    Serial.println(status);
    while (1) {}
  }
  status = gyro.begin();
  if (status < 0) {
    Serial.println("Gyro Initialization Error");
    Serial.println(status);
    while (1) {}
  }

  if (!sd.begin(SdioConfig(FIFO_SDIO))) {
      Serial.println("SD Begin Failed");
  }
  Serial.println("\nFIFO SDIO mode.");

  Serial.println("Starting ...");
  file = sd.open("datalog.txt", O_RDWR | O_CREAT | O_AT_END);
}

void loop() {
  unsigned long currentLog = micros();

  /* read the accel */
  accel.readSensor();
  /* read the gyro */
  gyro.readSensor();
  /* print the data */

  realPacket data = {0xBEEF00D, (currentLog-offset), 0.00, 0.00, 1000.0, accel.getAccelX_mss(), accel.getAccelY_mss(), accel.getAccelZ_mss(),
                      gyro.getGyroX_rads(), gyro.getGyroY_rads(), gyro.getGyroZ_rads(), accel.getTemperature_C(), 1000.0};

  if (file) {
    file.write((const uint8_t *)&data, sizeof(data));
    Serial.write((const uint8_t *)&data, sizeof(data)));
  } else {
    Serial.println("Error opening datalog.txt");
  }

  if (Serial.available()) {
    if (!file.close()) {
      Serial.println("Error");
    }
    Serial.println("Stopped!");
    while (true);
  }
  previousLog = currentLog;
}