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

SdFs sd;
FsFile file;

FastCRC32 CRC32;

unsigned long offset = 0;
unsigned long previousTime = 0;

double am[3];
double wm[3];
#define FILE_BASE_NAME "Data_"
const uint8_t BASE_NAME_SIZE = sizeof(FILE_BASE_NAME) - 1;
char fileName[] = FILE_BASE_NAME "00.csv";
int count;
int start;

const int led = 13;
long blinkCounter;
bool ledOn;

Ahrs thisahrs;
Sensors sen;

void setup() {
  Serial.begin(9600);
  Serial3.begin(9600);
  //while(!Serial) {}
  while(!Serial3) {}
  Serial3.flush();

  sen = Sensors();

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

  realPacket data = {0x7E, (micros()-offset), 0, thisahrs.aglobal.b, thisahrs.aglobal.c, thisahrs.aglobal.d,
                      gyr.x, gyr.y, gyr.z, mag.x, mag.y, mag.z, baro.readAltitudeM(), (accel.getTemperature_C() + baro.readTempC()) / 2.0,
                      groundToSensorFrame.a, groundToSensorFrame.b, groundToSensorFrame.c, groundToSensorFrame.d};

  data.checksum = CRC32.crc32((const uint8_t *)&data+sizeof(short), sizeof(realPacket) - 6);
  
  if (file) {
    file.print(data.time); file.print(","); file.print(data.code); file.print(","); file.print(data.accx); file.print(","); file.print(data.accy); file.print(","); file.print(data.accz); file.print(",");
    file.print(data.avelx); file.print(","); file.print(data.avely); file.print(","); file.print(data.avelz); file.print(","); file.print(data.altitude); file.print(","); file.print(data.pressure); file.print(",");
    file.print(data.temp); file.print(","); file.print(data.w); file.print(","); file.print(data.x); file.print(","); file.print(data.y); file.print(","); file.print(data.z); file.print(",");
    file.print(data.checksum); file.println(",");
  } else {
    data.code = -1;
  }

  if (count % 10 == 0) {
    Serial.write((const uint8_t *)&data, sizeof(data));
    Serial3.write((const uint8_t *)&data, sizeof(data));

    // file.close();
    // file = sd.open(fileName, FILE_WRITE);
  }

  count += 1;
}