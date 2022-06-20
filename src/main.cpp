#include <Arduino.h>
#include <SdFat.h>
#include <FastCRC.h>
#include <filters.h>
#include <Quaternion.h>
#include <Vec3.h>
#include <Ahrs.h>
#include <Senors.h>

#define _g_ (9.80665)

typedef struct {
  byte delim;
  float time;
  int code;
  float voltage;
  float accx;
  float accy;
  float accz;
  float avelx;
  float avely;
  float avelz;
  float magx;
  float magy;
  float magz;
  float altitude;
  float pressure;
  float temp;
  float w;
  float x;
  float y;
  float z;
  unsigned int checksum;
} __attribute__((packed)) realPacket;

FastCRC32 CRC32;

unsigned long offset = 0;
unsigned long previousTime = 0;

double am[3];
double wm[3];
int count;

const int led = 13;
long blinkCounter;
bool ledOn;

Ahrs thisahrs;
Sensors sen;
SdFs sd;

void setup() {
  Serial.begin(9600);
  Serial3.begin(9600);
  while(!Serial3) {}
  Serial3.flush();
  sen.beginSD();
  if (!sen.f) {
    Serial.println("Failed writing file");
    while(1){};
  }
  Serial.println("Starting ...");
}

Quaternion orientation = Quaternion();
long lastTime = micros();
double threshold = 0.05;

void loop() {
  if (millis() - blinkCounter >= 500) {
    if (ledOn) {
      ledOn = false;
      digitalWrite(led, LOW);
    } else {
      ledOn = true;
      digitalWrite(led, HIGH);
    }
    blinkCounter = millis();
  }

  /* read the accel */
  Vec3 acc = sen.readAccel();

  /* read the mag */
  Vec3 mag = sen.readMag();

  /* read the gyr */
  Vec3 gyr = sen.readGyro();

  thisahrs.update(acc,gyr,mag);
  Quaternion groundToSensorFrame = thisahrs.q;

  realPacket data = {0x7E, (micros()-offset), 0, sen.readVoltage(), thisahrs.aglobal.b, thisahrs.aglobal.c, thisahrs.aglobal.d,
                      gyr.x, gyr.y, gyr.z, mag.x, mag.y, mag.z, sen.readAltitude(), sen.readTemperature(),
                      groundToSensorFrame.a, groundToSensorFrame.b, groundToSensorFrame.c, groundToSensorFrame.d};

  data.checksum = CRC32.crc32((const uint8_t *)&data+sizeof(short), sizeof(realPacket) - 6);
  if (sen.f) {
    //f.write((const uint8_t *)&data, sizeof(data));
    Serial.println(data.voltage);
    sen.f.println(data.voltage);
  } else {
    data.code = -1;
  }

  if (count % 10 == 0) {
    //Serial.write((const uint8_t *)&data, sizeof(data));

    Serial3.write((const uint8_t *)&data, sizeof(data));
    sen.f.close();
    sen.f = sen.sd.open(fileName, FILE_WRITE);
  }

  count += 1;
}