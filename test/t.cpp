#include <Arduino.h>
#include <BMI088.h>
#include <DFRobot_BMP3XX.h>
#include <SdFat.h>
#include <MEFKcRP.h>
#include <FastCRC.h>

void setup() {
  Serial3.begin(115200);
  Serial.begin(115200);
}

void loop() {
  Serial.println("Hello");
  Serial3.println("Hello");

  delay(1000);
}