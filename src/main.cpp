#include <Arduino.h>
#include <BMI088.h>
#include <DFRobot_BMP3XX.h>
#include <SdFat.h>
#include <MEFKcRP.h>
#include <FastCRC.h>

typedef struct {
  int magic;
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
} __attribute__((packed)) realPacket;

/* accel object */
Bmi088Accel accel(Wire,0x18);
/* gyro object */
Bmi088Gyro gyro(Wire,0x68);
/* baro object */
DFRobot_BMP390L_I2C baro(&Wire, baro.eSDOVDD);

SdFs sd;
FsFile file;

unsigned long offset = 0;
unsigned long previousTime = 0;

double lastTime = micros();
double am[3];
double wm[3];
#define FILE_BASE_NAME "Data_"
const uint8_t BASE_NAME_SIZE = sizeof(FILE_BASE_NAME) - 1;
char fileName[] = FILE_BASE_NAME "00.csv";
int count;
int start;
long csCounter;
String callSign = "__KD2YPN__";

void setup() {
  int status;
<<<<<<< HEAD
  Serial.begin(115200);
  Serial3.begin(115200);
=======
  Serial.begin(500000);
  Serial3.begin(500000);
>>>>>>> refs/remotes/origin/main
  while(!Serial) {}
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
  baro.calibratedAbsoluteDifference(20.0);

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
      } else {
        Serial.println("Can't create file name");
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
}

void loop() {
  
  int packetsize = 60;

  // Callsign Transmission
  if (millis() - csCounter >= 540000) {
    packetsize = 10;
    Serial3.print(packetsize);
    Serial3.print(callSign);
    packetsize = 60;
    csCounter = millis();
  }

  // Reset Orientation
  if (Serial.available()) {
    if (Serial.readString() == "reset") {
      reset_orientation();
    } else if (Serial.readString() == "quit") {
      if (!file.close()) {
        Serial.println("Error Stopping.");
      }
      Serial.println("Stopped!");
      while (true);
    }
  }

  /* read the accel */
  accel.readSensor();
  /* read the gyro */
  gyro.readSensor();
  /* print the data */

<<<<<<< HEAD
  realPacket data = {0xBEEFF00D, (micros()-offset), accel.getAccelZ_mss(), accel.getAccelY_mss(), -1 * accel.getAccelX_mss(),
                      -1 * gyro.getGyroZ_rads(), -1 * gyro.getGyroY_rads(), gyro.getGyroX_rads(), baro.readAltitudeM(),  baro.readPressPa(),
=======
  realPacket data = {0xBEEFF00D, (micros()-offset), accel.getAccelX_mss(), accel.getAccelY_mss(), accel.getAccelZ_mss(),
                      gyro.getGyroX_rads(), gyro.getGyroY_rads(), gyro.getGyroZ_rads(), baro.readAltitudeM(), baro.readPressPa(),
>>>>>>> refs/remotes/origin/main
                      (accel.getTemperature_C() + baro.readTempC()) / 2};
  // x y z
  float am[3] = {data.accx, data.accy, -1 * data.accz};
  float wm[3] = {data.avelx, data.avely, data.avelz};
  float q[4];
  
  updateIMU(am, wm, (micros()-previousTime) / 1000000.0);
  get_q(q);
  
  data.w = q[0];
  data.x = q[1];
  data.y = q[2];
  data.z = q[3];

  previousTime = micros();

  if (file) {
    /*
    Serial.print("w"); Serial.print(data.w); Serial.print("w");
    Serial.print("a"); Serial.print(data.x); Serial.print("a");
    Serial.print("b"); Serial.print(data.y); Serial.print("b");
    Serial.print("c"); Serial.print(data.z); Serial.println("c");
    */
    file.write((const uint8_t *)&data, sizeof(data));
    Serial.write((const uint8_t *)&data, sizeof(data));
    Serial3.flush();
    Serial3.write((const uint8_t *)&packetsize, sizeof(packetsize));
    Serial3.write((const uint8_t *)&data, sizeof(data));
  } else {
    Serial.println("Error opening datalog.txt");
  }

  // count++;
  // if (count%100==0){
  //   double time2 = micros();
  //   double delta = (time2-lastTime) / 1000000.0;
  //   lastTime = time2;
  //   Serial.println("\nfps: ");
  //   Serial.println(100/delta);
  //   Serial.println("---------------");
  // }
}