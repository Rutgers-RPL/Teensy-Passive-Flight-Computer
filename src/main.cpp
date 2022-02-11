#include <Arduino.h>
#include <BMI088.h>
#include <SdFat.h>

/* accel object */
Bmi088Accel accel(Wire,0x18);
/* gyro object */
Bmi088Gyro gyro(Wire,0x68);

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
}

void loop() {
  // put your main code here, to run repeatedly:
  /* read the accel */
  accel.readSensor();
  /* read the gyro */
  gyro.readSensor();
  /* print the data */
  Serial.print(accel.getAccelX_mss());
  Serial.print("\t");
  Serial.print(accel.getAccelY_mss());
  Serial.print("\t");
  Serial.print(accel.getAccelZ_mss());
  Serial.print("\t");
  Serial.print(gyro.getGyroX_rads());
  Serial.print("\t");
  Serial.print(gyro.getGyroY_rads());
  Serial.print("\t");
  Serial.print(gyro.getGyroZ_rads());
  Serial.print("\t");
  Serial.print(accel.getTemperature_C());
  Serial.print("\n");
  /* delay to help with printing */
  delay(20);
}