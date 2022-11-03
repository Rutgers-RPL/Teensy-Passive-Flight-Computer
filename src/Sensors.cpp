
#include "Sensors.h"
#include <Arduino.h>
//#include <SdFat.h>
//#include <BMI085.h>
//#include <DFRobot_BMP3XX.h>
//#include <DFRobot_BMM150.h>
// #include <Quaternion.h>
//#include <Vec3.h>
// #include <Mat3x3.h>
// #include <EEPROM.h>
// #include <filters.h>
#include <string.h>



#define FILE_BASE_NAME "FlightLog_"
//const uint8_t BASE_NAME_SIZE = sizeof(FILE_BASE_NAME) - 1;


/* accel object */
Bmi085Accel accel(Wire,0x18);
/* gyro object */
Bmi085Gyro gyro(Wire,0x68);
/* baro object */
DFRobot_BMP390L_I2C baro(&Wire, baro.eSDOVDD);
/* mag object*/
DFRobot_BMM150_I2C bmm150(&Wire, 0x13);

//const uint8_t batteryPin = 22;

const float sampling_time = 0.05;
IIR::ORDER order = IIR::ORDER::OD3;

const float accel_cf = 20.0;
const float gyro_cf = 20.0;
const float mag_cf = 20.0;
const float pressure_cf = 20.0;
const float altitude_cf = 20.0;
const float temperature_cf = 20.0;

Filter accelFilterX(accel_cf, sampling_time, order);
Filter accelFilterY(accel_cf, sampling_time, order);
Filter accelFilterZ(accel_cf, sampling_time, order);
Filter gyroFilter(gyro_cf, sampling_time, order);
Filter magFilter(mag_cf, sampling_time, order);
Filter pressureFilter(pressure_cf, sampling_time, order);
Filter altitudeFilter(altitude_cf, sampling_time, order);
Filter temperatureFilter(temperature_cf, sampling_time, order);
Quaternion imuRot = Quaternion::from_euler_rotation(0, 0, 0);
Quaternion magRot = Quaternion::from_euler_rotation(0, 0, (-1 * PI)/ 2.0);
Quaternion allRot = Quaternion::from_euler_rotation(PI/2.0, 0, 0);

    Vec3 Accelerometer::readAccel()
    {
        
            /* read the accel */
            accel.readSensor();
            Quaternion q(accel.getAccelX_mss(), accel.getAccelY_mss(), accel.getAccelZ_mss());
            q = imuRot.rotate(q);
            q = allRot.rotate(q);
            /* print the data */
            accelFilterX.filterIn(q.b);
            accelFilterY.filterIn(q.c);
            accelFilterZ.filterIn(q.d);
            return Vec3(q.b, q.c, q.d);
        }
        Accelerometer::Accelerometer(){}

        Vec3 Accelerometer::readFilteredAccel()
        {
            Vec3 vec = readAccel();
            return Vec3(accelFilterX.filterIn(vec.x),accelFilterY.filterIn(vec.y),accelFilterZ.filterIn(vec.z));
        }
        

    

        Vec3 Gyro::readGyro()
        {
            gyro.readSensor();

            Quaternion q(gyro.getGyroX_rads(),gyro.getGyroY_rads(),gyro.getGyroZ_rads());
            q = imuRot.rotate(q);
            q = allRot.rotate(q);
            return Vec3(q.b, q.c, q.d);
        }

        Vec3 Gyro::readFilteredGyro()
        {

        }
    



    
        Vec3 Magnetometer::readMag()
        {
            sBmm150MagData_t magData = bmm150.getGeomagneticData();
            Quaternion q(magData.x, magData.y, magData.z);
            q = magRot.rotate(q);
            q = allRot.rotate(q);
            return Vec3(q.b, q.c, q.d);
        }

        Vec3 Magnetometer::readFilteredMag()
        {
            
        }


    
        float Barometer::readPressure()
        {
            return baro.readPressPa();
        }

        float Barometer::readFilteredPressure()
        {

        }

        float Barometer::readAltitude()
        {
            return baro.readAltitudeM();
        }

        float Barometer::readFilteredAltitude()
        {

        }

        
        float systate::readVoltage()
        {
            return (15.721519 * ((double) analogRead(batteryPin) / 1023.0));
        }

        float systate::readFilteredVoltage()
        {

        }


    


        Sensors::Sensors()
        {

            //short x = readShort(0);
            //short y = readShort(2);
            //short z = readShort(4); //carlton says none of this should be here


            int status;
            status = accel.begin();
            if (status < 0) 
            {
                Serial.println("Accel Initialization Error");
                Serial.println(status);
                while (1) {}
            }
            status = gyro.begin();
            if (status < 0) 
            {
                Serial.println("Gyro Initialization Error");
                Serial.println(status);
                while (1) {}
            }
            status = baro.begin();
            if (status < 0) 
            {
                if(ERR_DATA_BUS == status)
                 {
                Serial.println("Data bus error!!!");
                }else if(ERR_IC_VERSION == status){
                Serial.println("Chip versions do not match!!!");
                }
                while (1) {}
            }
            while(!baro.setSamplingMode(baro.eUltraPrecision))
            {
                Serial.println("Set samping mode fail, retrying....");
                delay(1000);
            }

            bmm150.begin();
            bmm150.setOperationMode(BMM150_POWERMODE_NORMAL);
            bmm150.setPresetMode(BMM150_PRESETMODE_HIGHACCURACY);
            bmm150.setRate(BMM150_DATA_RATE_30HZ);
            bmm150.setMeasurementXYZ();

            Serial.println("Sensor initialization complete...");
        }

 
       

        //Deprecated(not needed)
        // float readTemperature(){
        //     return (baro.readTempC() + accel.getTemperature_C()) / 2.0;
        // }

        // float readFilteredTemperature(){

        // }
       
        
        //just leave here, no home currently





        //rip sd stuff
        // void beginSD() {
        //     if (!sd.begin(SdioConfig(FIFO_SDIO))) {
        //         Serial.println("SD Begin Failed");
        //     } else {
        //         Serial.println("\nFIFO SDIO mode.");
        //         while (sd.exists(fileName)) {
        //             if (fileName[BASE_NAME_SIZE + 3] != '9') {
        //                 fileName[BASE_NAME_SIZE + 3]++;
        //             } else if (fileName[BASE_NAME_SIZE + 2] != '9') {
        //                 fileName[BASE_NAME_SIZE + 3] = '0';
        //                 fileName[BASE_NAME_SIZE + 2]++;
        //             } else if (fileName[BASE_NAME_SIZE + 1] != '9') {
        //                 fileName[BASE_NAME_SIZE + 2] = '0';
        //                 fileName[BASE_NAME_SIZE + 3] = '0';
        //                 fileName[BASE_NAME_SIZE + 1]++;
        //             } else if (fileName[BASE_NAME_SIZE] != '9') {
        //                 fileName[BASE_NAME_SIZE + 1] = '0';
        //                 fileName[BASE_NAME_SIZE + 2] = '0';
        //                 fileName[BASE_NAME_SIZE + 3] = '0';
        //                 fileName[BASE_NAME_SIZE]++;
        //             } else {
        //                 Serial.println("Can't create file name");
        //             }
        //         }
        //         //Serial.println("wtf");
        //         f = sd.open(fileName, FILE_WRITE);
        //         Serial.print("Writing to: ");
        //         Serial.println(fileName);
        //         if (!f) {
        //             Serial.println("Failed opening file.");
        //         }
        //         sdexists = true;
        //     }
        // }


    // private:

    //     short int readShort(short address)
    //     {
    //         short b1 = EEPROM.read(address);
    //         short b2 = EEPROM.read(address+1);
    //         return (b1<<8)+b2;
    //     }

    //     void writeShort(short address, short data)
    //     {
    //         EEPROM.write(address, data>>8);
    //         EEPROM.write(address+1, (data<<8)>>8);
    //     }


