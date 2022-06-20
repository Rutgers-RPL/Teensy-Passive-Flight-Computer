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
  short magic;
  int code;
  float time;
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

  // /* start the sensors */
  // status = accel.begin();
  // if (status < 0) {
  //   Serial.println("Accel Initialization Error");
  //   Serial.println(status);
  //   while (1) {}
  // }
  // status = gyro.begin();
  // if (status < 0) {
  //   Serial.println("Gyro Initialization Error");
  //   Serial.println(status);
  //   while (1) {}
  // }
  // status = baro.begin();
  // if (status < 0) {
  //   if(ERR_DATA_BUS == status) {
  //     Serial.println("Data bus error!!!");
  //   }else if(ERR_IC_VERSION == status){
  //     Serial.println("Chip versions do not match!!!");
  //   }
  //   while (1) {}
  // }
  // while(!baro.setSamplingMode(baro.eUltraPrecision)){
  //   Serial.println("Set samping mode fail, retrying....");
  //   delay(1000);
  // }

  // bmm150.begin();
  // bmm150.setOperationMode(BMM150_POWERMODE_NORMAL);
  // bmm150.setPresetMode(BMM150_PRESETMODE_HIGHACCURACY);
  // bmm150.setRate(BMM150_DATA_RATE_10HZ);
  // bmm150.setMeasurementXYZ();

  // if (!sd.begin(SdioConfig(FIFO_SDIO))) {
  //     Serial.println("SD Begin Failed");
  // }
  // Serial.println("\nFIFO SDIO mode.");
  //   while (sd.exists(fileName)) {
  //     if (fileName[BASE_NAME_SIZE + 1] != '9') {
  //       fileName[BASE_NAME_SIZE + 1]++;
  //     } else if (fileName[BASE_NAME_SIZE] != '9') {
  //       fileName[BASE_NAME_SIZE + 1] = '0';
  //       fileName[BASE_NAME_SIZE]++;
  //     } else if (fileName[BASE_NAME_SIZE] != '9') {
  //       fileName[BASE_NAME_SIZE + 1] = '0';
  //       fileName[BASE_NAME_SIZE]++;
  //     } else {
  //       Serial.println("Can't create file name");
  //       // realPacket data;
  //       // data.code = -1;
        
  //       return;
  //     }
  // }
  // file = sd.open(fileName, FILE_WRITE);
  // if (!file) {
  //   Serial.println(F("open failed."));
  //   return;
  // }
  // Serial.print(F("opened: "));
  // Serial.println(fileName);

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

  // double dt = ((double) (micros() - lastTime)) / 1000000;
  // if(acc.magnitude()<_g_+threshold && acc.magnitude()>_g_-threshold){
  //   Vec3 down = -1*acc;
  //   down.normalize();
  //   Vec3 east = down.cross(mag);
  //   east.normalize();
  //   Vec3 north = east.cross(down);
  //   north.normalize();

  //   orientation = dcm2quat(north, east, down);
  // } else {
  //   orientation = Quaternion::from_euler_rotation(gyro.getGyroY_rads()*dt, gyro.getGyroX_rads()*dt, -1*gyro.getGyroZ_rads()*dt) * orientation;
  // }
  // lastTime = micros();

  realPacket data = {0xBEEF, (micros()-offset), 0, acc.x, acc.y, acc.z,
                      gyr.x, gyr.y, gyr.z, mag.x, mag.y, mag.z, baro.readAltitudeM(),
                      (baro.readTempC()) / 1};

  Quaternion groundToSensorFrame = orientation;
  data.w = groundToSensorFrame.a;
  data.x = groundToSensorFrame.b;
  data.y = groundToSensorFrame.c;
  data.z = groundToSensorFrame.d;

  data.accx = thisahrs.aglobal.b;
  data.accy = thisahrs.aglobal.c;
  data.accz = thisahrs.aglobal.d;

  //Serial.printf("(%f, %f, %f)\n", data.accx, data.accy, data.accz);
  

  data.checksum = CRC32.crc32((const uint8_t *)&data+sizeof(short), sizeof(realPacket) - 6);
  
  // if (file) {
  //   file.print(data.time); file.print(","); file.print(data.code); file.print(","); file.print(data.accx); file.print(","); file.print(data.accy); file.print(","); file.print(data.accz); file.print(",");
  //   file.print(data.avelx); file.print(","); file.print(data.avely); file.print(","); file.print(data.avelz); file.print(","); file.print(data.altitude); file.print(","); file.print(data.pressure); file.print(",");
  //   file.print(data.temp); file.print(","); file.print(data.w); file.print(","); file.print(data.x); file.print(","); file.print(data.y); file.print(","); file.print(data.z); file.print(",");
  //   file.print(data.checksum); file.println(",");
  // } else {
  //   data.code = -1;
  // }
  //Serial.println(sen.getCompassDegree());
  if (count % 10 == 0) {
    Serial.write((const uint8_t *)&data, sizeof(data));
    Serial3.write((const uint8_t *)&data, sizeof(data));

    // file.close();
    // file = sd.open(fileName, FILE_WRITE);
  }

  count += 1;
}