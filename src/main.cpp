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
double am[3];
double wm[3];
#define FILE_BASE_NAME "Data_"
const uint8_t BASE_NAME_SIZE = sizeof(FILE_BASE_NAME) - 1;
char fileName[] = FILE_BASE_NAME "00.csv";
int count;
int start;

void setup() {
  int status;
  Serial.begin(9600);
  Serial3.begin(9600);
  while(!Serial) {}
  while(!Serial3) {}
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
  while( !baro.setSamplingMode(baro.eUltraPrecision)){
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
}

void loop() {
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


  realPacket data = {0xBEEFF00D, (micros()-offset), accel.getAccelX_mss(), accel.getAccelY_mss(), accel.getAccelZ_mss(),
                      gyro.getGyroX_rads(), gyro.getGyroY_rads(), gyro.getGyroZ_rads(), baro.readAltitudeM(),  baro.readPressPa(),
                      (accel.getTemperature_C() + baro.readTempC()) / 2};
  // x y z
  float am[3] = {data.accx, data.accy, data.accz};
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
    file.write((const uint8_t *)&data, sizeof(data));
    Serial.write((const uint8_t *)&data, sizeof(data));
    Serial3.flush();
    Serial3.write((const uint8_t *)&data, sizeof(data));
    //Serial.print(data.altitude); Serial.print("\t"); Serial.println(data.pressure);
    //Serial3.print(data.altitude); Serial3.print("\t"); Serial3.println(data.pressure);
    //Serial.print(q[0]); Serial.print("\t");  Serial.print(q[1]); Serial.print("\t");  Serial.print(q[2]); Serial.print("\t");  Serial.print(q[3]);
    //Serial.println();
    //Serial.print("w"); Serial.print(q[0]); Serial.print("wa"); Serial.print(q[1]);  Serial.print("ab");  Serial.print(q[2]); Serial.print("bc");  Serial.print(q[3]);  Serial.print("c");
    //Serial.println();  
  } else {
    Serial.println("Error opening datalog.txt");
  }
}