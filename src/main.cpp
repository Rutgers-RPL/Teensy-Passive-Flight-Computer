#include <Arduino.h>
#include <SdFat.h>
#include <FastCRC.h>
#include <Quaternion.h>
#include <Vec3.h>
#include <Ahrs.h>
#include <Sensors.h>

#define _g_ (9.80665)

typedef struct {
  short magic;
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
int start;

const int led = 13;
long blinkCounter;
bool ledOn;

Ahrs thisahrs;
Sensors sen;

void setup() {
  Serial.begin(115200);
  Serial3.begin(115200);
  //while(!Serial) {}
  while(!Serial3) {}
  sen.beginSD();
  Serial.println("test");
  Serial3.flush();
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
  orientation = thisahrs.q;

  Quaternion groundToSensorFrame = orientation;

  realPacket data = {0xBEEF, (micros()-offset) / 1000000.0, 0, sen.readVoltage(), thisahrs.aglobal.b, thisahrs.aglobal.c, thisahrs.aglobal.d,
                      gyr.x, gyr.y, gyr.z, mag.x, mag.y, mag.z, baro.readAltitudeM(),
                      (baro.readTempC()) / 1.0, groundToSensorFrame.a, groundToSensorFrame.b, groundToSensorFrame.c, groundToSensorFrame.d};


  //Serial.printf("(%f, %f, %f)\n", data.accx, data.accy, data.accz);

  data.checksum = CRC32.crc32((const uint8_t *)&data+sizeof(short), sizeof(realPacket) - 6);
  
  if (sen.sdexists && sen.f) {
    sen.f.print(data.time); sen.f.print(","); sen.f.print(data.code); sen.f.print(","); sen.f.print(data.voltage); sen.f.print(","); sen.f.print(data.accx); sen.f.print(",");
    sen.f.print(data.accy); sen.f.print(","); sen.f.print(data.accz); sen.f.print(","); sen.f.print(data.avelx); sen.f.print(","); sen.f.print(data.avely); sen.f.print(",");
    sen.f.print(data.avelz); sen.f.print(","); sen.f.print(data.magx); sen.f.print(","); sen.f.print(data.magy); sen.f.print(","); sen.f.print(data.magz); sen.f.print(",");
    sen.f.print(data.altitude); sen.f.print(","); sen.f.print(data.temp); sen.f.print(","); sen.f.print(data.w); sen.f.print(","); sen.f.print(data.x); sen.f.print(",");
    sen.f.print(data.y); sen.f.print(","); sen.f.print(data.z); sen.f.println(",");
  } else {
    data.code = -1;
  }

  if (count % 15 == 0) {
    Serial.write((const uint8_t *)&data, sizeof(data));
    Serial3.write((const uint8_t *)&data, sizeof(data));

    if (sen.sdexists) {
      sen.f.close();
      sen.f = sen.sd.open(sen.fileName, FILE_WRITE);
    }
  }

  count += 1;
}