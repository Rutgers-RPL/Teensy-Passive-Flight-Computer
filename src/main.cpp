#include <Arduino.h>
#include <BMI085.h>
#include <DFRobot_BMP3XX.h>
#include <SdFat.h>
#include <MEFKcRP.h>
#include <FastCRC.h>
#include <filters.h>
#include <Quaternion.h>

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

const float accxOffset = -0.015;
const float accyOffset = -0.045;
const float acczOffset = -0.1;


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
long csCounter;
String callSign = "@@@___KD2ZFK___@@@___KD2ZFK___@@@___KD2ZFK___@@@";
const int led = 13;
long blinkCounter;
bool ledOn;

long lastTime;

long intTime;
double posX = 0;
double posY = 0;
double posZ = 0;
double velX = 0;
double velY = 0;
double velZ = 0;

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
        realPacket data;
        data.code = -1;
        
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

  // EKF Initialize
  OrientationEstimator();
  float sampingPeriodus = baro.getSamplingPeriodUS();
  Serial.print("samping period : ");
  Serial.print(sampingPeriodus);
  Serial.println(" us");

  float sampingFrequencyHz = 1000000 / sampingPeriodus;
  Serial.print("samping frequency : ");
  Serial.print(sampingFrequencyHz);
  Serial.println(" Hz");

  // Callsign Transmission
  Serial3.println(callSign);
  csCounter = millis();
  blinkCounter = millis();
  ledOn = true;

  int count = 0;
  lastTime = millis();
  intTime = micros();
}

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
  // Callsign Transmission
  if (millis() - csCounter >= 540000) {
    Serial3.println();
    Serial3.println(callSign);
    csCounter = millis();
  }

  /* read the accel */
  accel.readSensor();
  /* read the gyro */
  gyro.readSensor();
  /* print the data */
  lastTime = millis();

  realPacket data = {0xBEEF, (micros()-offset), 0, accel.getAccelZ_mss() + accxOffset, accel.getAccelY_mss() + accyOffset, accel.getAccelX_mss() + acczOffset,
                      gyro.getGyroZ_rads(), gyro.getGyroY_rads(), gyro.getGyroX_rads(), alt.filterIn(baro.readAltitudeM()),  baro.readPressPa(),
                      (accel.getTemperature_C() + baro.readTempC()) / 2};

  
  // x y z
  float am[3] = {data.accx, data.accy, data.accz};
  float wm[3] = {data.avelx, data.avely, data.avelz};
  float q[4];
  updateIMU(am, wm, (micros()-previousTime) / 1000000.0);
  get_q(q);
  
  /*
  auto quat = Quaternion();
  quat.a = q[0];
  quat.b = q[1];
  quat.c = q[2];
  quat.d = q[3];
  
  auto acc = Quaternion(data.accx, data.accy, data.accz);
  acc.a = 0;

  auto vec = quat.rotate(acc) + Quaternion(0, 0, -9.81);

  double delta = ((double) (micros() - intTime)) / 1000000;
  if ((micros() - offset) <= 5000000) {
    
  } else {
    vec.b = accx.filterIn(vec.b);
    if (abs(vec.b) > 0.05) {
      Serial.println("bob");
      velX += delta * vec.b;
    }
    vec.c = accy.filterIn(vec.c);
    if (abs(vec.c) > 0.05) {
      Serial.println("bob");
      velY += delta * vec.c;
    }
    vec.d = accz.filterIn(vec.d);
    if (abs(vec.d) > 0.05) {
      Serial.println("bob");
      velZ += delta * vec.d;
    }

    if (abs(velX) > 0.1) {
      Serial.println("nob");
      posX += delta * velX;
    }
    if (abs(velY) > 0.1) {
      Serial.println("nob");
      posY += delta * velY;
    }
    if (abs(velZ) > 0.1) {
      Serial.println("nob");
      posZ += delta * velZ;
    }

    intTime = micros();

    Serial.printf("ACCEL x: %f   y: %f    z: %f", vec.b, vec.c, vec.d);
    Serial.println();
    Serial.printf("VELOC x: %f   y: %f    z: %f", velX, velY, velZ);
    Serial.println();
    Serial.printf("POSIT x: %f   y: %f    z: %f", posX, posY, posZ);
    Serial.println();
  }

  intTime = micros();
  */

  data.w = q[0];
  data.x = q[1];
  data.y = q[2];
  data.z = q[3];

  previousTime = micros();

  data.checksum = CRC32.crc32((const uint8_t *)&data+sizeof(short), sizeof(realPacket) - 6);
  
  if (file) {
    file.print(data.time); file.print(","); file.print(data.code); file.print(","); file.print(data.accx); file.print(","); file.print(data.accy); file.print(","); file.print(data.accz); file.print(",");
    file.print(data.avelx); file.print(","); file.print(data.avely); file.print(","); file.print(data.avelz); file.print(","); file.print(data.altitude); file.print(","); file.print(data.pressure); file.print(",");
    file.print(data.temp); file.print(" ,"); file.print(data.w); file.print(","); file.print(data.x); file.print(","); file.print(data.y); file.print(","); file.print(data.z); file.print(",");
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