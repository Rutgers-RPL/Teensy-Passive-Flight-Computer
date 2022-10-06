/**
 * @file main.cpp
 * @author Shivam Patel (shivam.patel94@rutgers.edu), Carlton Wu (carlton.wu@rutgers.edu), William Freitag (william.h.freitag@gmail.com)
 * @brief This runs the main data collection, processing, and transmission loop for the Minerva II flight computer
 * @version 1.0
 * @date 2022-09-14
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#include <Arduino.h>
#include <SdFat.h> //For the SD card
#include <FastCRC.h> //For the checksum
#include <Quaternion.h> //Class that allows for Quaternions
#include <Vec3.h> //Allows us to use 3-D vectors
#include <Ahrs.h> //Altitude heading reference system
#include <Sensors.h> //Library to interface with sensors

#define _g_ (9.80665) //Gravity

typedef struct {
  short magic; // 2 bytes - 2 Defines the beginning/end of a sentence
  float time; // 4 bytes - 6 Time when packet was sent
  int code; // 4 bytes - 10 Think error codes
  float voltage; // 4 bytes - 14 Voltage coming out of the battery of the teensy
  float accx; // 4 bytes - 18 Acceleration in the x
  float accy; // 4 bytes - 22 Acceleration in the y
  float accz; // 4 bytes - 26 Acceleration in the z
  float avelx; // 4 bytes - 30 Angular velocity in the x
  float avely; // 4 bytes - 34 Angular velocity in the y
  float avelz; // 4 bytes - 38 Angular velocity in the z
  float magx; // 4 bytes - 42 Magnetometer in each direction (detecting magnetic fields)
  float magy; // 4 bytes - 46 
  float magz; // 4 bytes - 50
  float altitude; // 4 bytes - 54 Altitude the system is at based on the ambient pressure
  float temp; // 4 bytes - 58 Temperature
  float w; // 4 bytes - 62 Next 4 are the terms in the quaternion
  float x; // 4 bytes - 66 
  float y; // 4 bytes - 70
  float z; // 4 bytes - 74
  unsigned int checksum; // 4 bytes - 78 For making sure the information is in tact
} __attribute__((packed)) realPacket; //Structure for the packets of data being sent. __attribute__((packed)) is used to save space

FastCRC32 CRC32; // Using this for the check sums

unsigned long offset = 0; //Offest in miliseconds
unsigned long previousTime = 0; //TO-DO: Delete

double am[3]; //TO-DO: Delete
double wm[3]; //TO-DO: Delete
int count;
int start; //TO-DO: Delete

const int led = 13; //Pin where the LED is connected to
long blinkCounter; 
bool ledOn;

Ahrs thisahrs; //Creates a new Altitude heading reference system object named ahrs 
Sensors sen; //Creates a new instamce of the sensor class

void setup() {
  Serial.begin(115200); //For communication with the USB for the Teensy
  Serial3.begin(115200); //For communication with radio
  //while(!Serial) {}
  while(!Serial3) {}
  sen.beginSD();
  Serial.println("test");
  Serial3.flush(); //No residual data
  Serial.println("Starting ...");
}

Quaternion orientation = Quaternion(); //Creates an instance of the Quaternion class, which allows us to tell the rockets' orientation
long lastTime = micros(); //TO-DO: Delete
double threshold = 0.05; //TO-DO: Delete

void loop() {
  if (millis() - blinkCounter >= 500) { //LED blinks to confirm that it is working
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

  thisahrs.update(acc,gyr,mag); //Update values in AHRS 
  orientation = thisahrs.q; //Updates the orientation of the Quaternion

  Quaternion groundToSensorFrame = orientation; //Update to current position

  realPacket data = {0xBEEF, (micros()-offset) / 1000000.0, 0, sen.readVoltage(), thisahrs.aglobal.b, thisahrs.aglobal.c, thisahrs.aglobal.d,
                      gyr.x, gyr.y, gyr.z, mag.x, mag.y, mag.z, baro.readAltitudeM(),
                      (baro.readTempC()) / 1.0, groundToSensorFrame.a, groundToSensorFrame.b, groundToSensorFrame.c, groundToSensorFrame.d};
  // Send

  //Serial.printf("(%f, %f, %f)\n", data.accx, data.accy, data.accz);

  data.checksum = CRC32.crc32((const uint8_t *)&data+sizeof(short), sizeof(realPacket) - 6);
  
  if (sen.sdexists && sen.f) {
    sen.f.print(data.time); sen.f.print(","); sen.f.print(data.code); sen.f.print(","); sen.f.print(data.voltage); sen.f.print(","); sen.f.print(data.accx); sen.f.print(",");
    sen.f.print(data.accy); sen.f.print(","); sen.f.print(data.accz); sen.f.print(","); sen.f.print(data.avelx); sen.f.print(","); sen.f.print(data.avely); sen.f.print(",");
    sen.f.print(data.avelz); sen.f.print(","); sen.f.print(data.magx); sen.f.print(","); sen.f.print(data.magy); sen.f.print(","); sen.f.print(data.magz); sen.f.print(",");
    sen.f.print(data.altitude); sen.f.print(","); sen.f.print(data.temp); sen.f.print(","); sen.f.print(data.w); sen.f.print(","); sen.f.print(data.x); sen.f.print(",");
    sen.f.print(data.y); sen.f.print(","); sen.f.print(data.z); sen.f.println(",");
    //If the SD Card is connected/inserted, print out the data collected from the sensor to the file 
  } else {
    data.code = -1; //Error meaining the SD card is not connected properly
    data.checksum = CRC32.crc32((const uint8_t *)&data+sizeof(short), sizeof(realPacket) - 6); //Compute the checksum
  
  }

  if (count % 15 == 0) {
    Serial.write((const uint8_t *)&data, sizeof(data)); //Writes data to the USB serial bus from the Teensy and print it to the Serial monitor
    Serial3.write((const uint8_t *)&data, sizeof(data)); //Writes data to the Radio and print it out to the Serial monitor

    if (sen.sdexists) { //If the SD card is working
      sen.f.close(); //Write the data to the file (can't write while the file is open)
      sen.f = sen.sd.open(sen.fileName, FILE_WRITE); //Open the file up again so you can write to it
    }
  }

  count += 1;
}