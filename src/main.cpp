#include <Arduino.h>
#include <BMI085.h>
#include <DFRobot_BMP3XX.h>
#include <DFRobot_BMM150.h>
#include <SdFat.h>
#include <FastCRC.h>
#include <filters.h>
#include <Quaternion.h>
#include <Vec3.h>
#include <Ahrs.h>

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
  float altitude;
  float pressure;
  float temp;
  float w;
  float x;
  float y;
  float z;
  unsigned int checksum;
} __attribute__((packed)) realPacket;

const float accxOffset = 0;
const float accyOffset = 0;
const float acczOffset = 0;

const float cutoff_freq = 2.0;
const float acutoff_freq = 1.3;
const float sampling_time = 0.004725;
IIR::ORDER  order  = IIR::ORDER::OD3; // Order (OD1 to OD4)

Filter accx(cutoff_freq, sampling_time, order);
Filter accy(cutoff_freq, sampling_time, order);
Filter accz(cutoff_freq, sampling_time, order);
Filter avelx(acutoff_freq, sampling_time, order);
Filter avely(acutoff_freq, sampling_time, order);
Filter avelz(acutoff_freq, sampling_time, order);
Filter alt(cutoff_freq, sampling_time, order);

/* accel object */
Bmi085Accel accel(Wire,0x18);
/* gyro object */
Bmi085Gyro gyro(Wire,0x68);
/* baro object */
DFRobot_BMP390L_I2C baro(&Wire, baro.eSDOVDD);
/*mag*/
DFRobot_BMM150_I2C bmm150(&Wire, 0x13);

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

float lPosX = 0;
float lPosY = 0;
float lPosZ = 0;
float lVelX = 0;
float lVelY = 0;
float lVelZ = 0;
double posX = 0;
double posY = 0;
double posZ = 0;
double velX = 0;
double velY = 0;
double velZ = 0;

Quaternion dcm2quat(Vec3 north, Vec3 east, Vec3 down){
  // dcm2quat https://intra.ece.ucr.edu/~farrell/AidedNavigation/D_App_Quaternions/Rot2Quat.pdf
  int maximum = 1;
  double q = (1+north.x+east.y+down.z);
  double o = (1+north.x-east.y-down.z);
  if (o > q){
    q = o;
    maximum = 2;
  }
  o = (1-north.x+east.y-down.z);
  if (o > q){
    q = o;
    maximum = 3;
  }
  o = (1-north.x-east.y+down.z);
  if (o > q){
    q = o;
    maximum = 4;
  }

  Quaternion output = Quaternion();
  switch(maximum){
    case 1:
      output.a = 0.5*sqrt(q);
      output.b = (down.y-east.z)/(4*output.a);
      output.c = (north.z-down.x)/(4*output.a);
      output.d = (east.x-north.y)/(4*output.a);
      break;
    case 2:
      output.b = 0.5*sqrt(q);
      output.a = (down.y-east.z)/(4*output.b);
      output.c = (north.y+east.x)/(4*output.b);
      output.d = (north.z+down.x)/(4*output.b);
      break;
    case 3:
      output.c = 0.5*sqrt(q);
      output.a = (north.z-down.x)/(4*output.c);
      output.b = (north.y+east.x)/(4*output.c);
      output.d = (east.z+down.y)/(4*output.c);
      break;
    case 4:
      output.d = 0.5*sqrt(q);
      output.a = (east.x-north.y)/(4*output.d);
      output.b = (north.z+down.x)/(4*output.d);
      output.c = (east.z+down.y)/(4*output.d);
      break;
  }

  output.normalize();
  return output;
}

Ahrs thisahrs;

void setup() {
  int status;
  Serial.begin(9600);
  Serial3.begin(9600);
  //while(!Serial) {}
  while(!Serial3) {}
  Serial3.flush();
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
  status = baro.begin();
  if (status < 0) {
    if(ERR_DATA_BUS == status) {
      Serial.println("Data bus error!!!");
    }else if(ERR_IC_VERSION == status){
      Serial.println("Chip versions do not match!!!");
    }
    while (1) {}
  }
  while(!baro.setSamplingMode(baro.eUltraPrecision)){
    Serial.println("Set samping mode fail, retrying....");
    delay(1000);
  }

  bmm150.begin();
  bmm150.setOperationMode(BMM150_POWERMODE_NORMAL);
  bmm150.setPresetMode(BMM150_PRESETMODE_HIGHACCURACY);
  bmm150.setRate(BMM150_DATA_RATE_10HZ);
  bmm150.setMeasurementXYZ();

  if (!sd.begin(SdioConfig(FIFO_SDIO))) {
      Serial.println("SD Begin Failed");
  }
  Serial.println("\nFIFO SDIO mode.");
    while (sd.exists(fileName)) {
      if (fileName[BASE_NAME_SIZE + 1] != '9') {
        fileName[BASE_NAME_SIZE + 1]++;
      } else if (fileName[BASE_NAME_SIZE] != '9') {
        fileName[BASE_NAME_SIZE + 1] = '0';
        fileName[BASE_NAME_SIZE]++;
      } else if (fileName[BASE_NAME_SIZE] != '9') {
        fileName[BASE_NAME_SIZE + 1] = '0';
        fileName[BASE_NAME_SIZE]++;
      } else {
        Serial.println("Can't create file name");
        // realPacket data;
        // data.code = -1;
        
        return;
      }
  }
  file = sd.open(fileName, FILE_WRITE);
  if (!file) {
    Serial.println(F("open failed."));
    return;
  }
  Serial.print(F("opened: "));
  Serial.println(fileName);

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
  accel.readSensor();
  /* print the data */
  Vec3 acc = Vec3(accel.getAccelY_mss(),accel.getAccelX_mss(),accel.getAccelZ_mss());

  sBmm150MagData_t magData = bmm150.getGeomagneticData();
  Vec3 mag(magData.x,magData.y,magData.z);

  gyro.readSensor();
  Vec3 gyr = Vec3(gyro.getGyroX_rads(),gyro.getGyroY_rads(),gyro.getGyroZ_rads());

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

  realPacket data = {0xBEEF, (micros()-offset), 0, accel.getAccelZ_mss() + accxOffset, accel.getAccelY_mss() + accyOffset, accel.getAccelX_mss() + acczOffset,
                      gyro.getGyroZ_rads(), gyro.getGyroY_rads(), gyro.getGyroX_rads(), alt.filterIn(baro.readAltitudeM()),  baro.readPressPa(),
                      (accel.getTemperature_C() + baro.readTempC()) / 2};

  Quaternion groundToSensorFrame = orientation.conj();
  data.w = groundToSensorFrame.a;
  data.x = groundToSensorFrame.b;
  data.y = groundToSensorFrame.c;
  data.z = groundToSensorFrame.d;

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

    file.close();
    file = sd.open(fileName, FILE_WRITE);
  }

  count += 1;
}