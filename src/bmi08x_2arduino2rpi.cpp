//#define DEBUG_MODE
#define DEVICE_ID 14
#define LED_PIN 13

#include <Arduino.h>
#include <BMI088.h>
#include <IPM_BMI08X.h>
#include <MessageManager.h>

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

/* accel object */
Bmi088Accel accel(Wire,0x18);
/* gyro object */
Bmi088Gyro gyro(Wire,0x68);

IPM_BMI08X mAG = IPM_BMI08X( DEVICE_ID );
MessageManager MM( 2 );

// variables to store the measurements
int16_t ax, ay, az;
int16_t wx, wy, wz;
int16_t T;

bool blinkState = false;


void setup() {
  // initialize serial communication
  Serial.begin(115200);
  while( !Serial );

  int status = 0;
  // initialize device
  Serial.println("Initializing I2C devices...");
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
  
  // configure Arduino LED pin for output
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, blinkState);
  
  // test
  //test_measurementFrequency();
}

void loop() {
  // read raw accel/gyro measurements from device
  ax = accel.getAccelX_mss();
  ay = accel.getAccelY_mss();
  az = accel.getAccelZ_mss();
  wx = gyro.getGyroX_rads();
  wy = gyro.getGyroY_rads();
  wz = gyro.getGyroZ_rads();
  T = accel.getTemperature_C();

  // we set the information packet
  mAG.set_a( ax , ay , az );
  mAG.set_w( wx , wy , wz );
  mAG.set_T( T );
  
#if defined DEBUG_MODE
  Serial.print( ax );   Serial.print("\t");   Serial.print( ay );   Serial.print("\t");   Serial.print( az );   Serial.print("\t\t");
  Serial.print( wx );   Serial.print("\t");   Serial.print( wy );   Serial.print("\t");   Serial.print( wz );
  Serial.println();
#else
  // we prepare the message
  int8_t* toWrite = MM.prepare_message( mAG.get_length() , mAG.get_bytes() );
  // and we send it
  Serial.write( (byte*)toWrite , MM.get_messageOutLength() );
#endif
}

void test_measurementFrequency() {
  int N = 1000;
  float dt0 = 0.0;
  float dt1 = 0.0;
  float dt2 = 0.0;
  float dt3 = 0.0;
  for(int i=0; i<N; i++){
    unsigned long t0 = micros();
    ax = accel.getAccelX_mss();
    ay = accel.getAccelY_mss();
    az = accel.getAccelZ_mss();
    wx = gyro.getGyroX_rads();
    wy = gyro.getGyroY_rads();
    wz = gyro.getGyroZ_rads();
    unsigned long t1 = micros();
    ax = accel.getAccelX_mss();
    ay = accel.getAccelY_mss();
    az = accel.getAccelZ_mss();
    unsigned long t2 = micros();
    wx = gyro.getGyroX_rads();
    wy = gyro.getGyroY_rads();
    wz = gyro.getGyroZ_rads();
    unsigned long t3 = micros();
    T = accel.getTemperature_C();
    unsigned long t4 = micros();
    dt0 += t1-t0;
    dt1 += t2-t1;
    dt2 += t3-t2;
    dt3 += t4-t3;
  }
  Serial.print( 1.0e6/(dt0/N) , 6 );
  Serial.print( " " );
  Serial.print( 1.0e6/(dt1/N) , 6 );
  Serial.print( " " );
  Serial.print( 1.0e6/(dt2/N) , 6 );
  Serial.print( " " );
  Serial.print( 1.0e6/(dt3/N) , 6 );
  Serial.println();
}