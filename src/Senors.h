#ifndef Sensors_H
#define Sensors_H
#include <Arduino.h>
#include <BMI085.h>
#include <DFRobot_BMP3XX.h>
#include <DFRobot_BMM150.h>
#include <Quaternion.h>
#include <Vec3.h>
#include <EEPROM.h>


/* accel object */
Bmi085Accel accel(Wire,0x18);
/* gyro object */
Bmi085Gyro gyro(Wire,0x68);
/* baro object */
DFRobot_BMP390L_I2C baro(&Wire, baro.eSDOVDD);
/*mag*/
DFRobot_BMM150_I2C bmm150(&Wire, 0x13);

class Sensors{
    public:
        Quaternion rot;

        Sensors(){
            short x = readShort(0);
            short y = readShort(2);
            short z = readShort(4);

            rot = Quaternion::from_euler_rotation(0, 0, 0);
            rot.d = -1;

            int status;
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
            bmm150.setRate(BMM150_DATA_RATE_30HZ);
            bmm150.setMeasurementXYZ();

            Serial.println("Sensor initialization complete...");
        }

        Vec3 readAccel(){
            /* read the accel */
            accel.readSensor();
            /* print the data */
            return Vec3(accel.getAccelX_mss(),accel.getAccelY_mss(),accel.getAccelZ_mss());
        }

        Vec3 readGyro(){
            gyro.readSensor();
            return Vec3(gyro.getGyroX_rads(),gyro.getGyroY_rads(),gyro.getGyroZ_rads());
        }

        Vec3 readMag(){
            sBmm150MagData_t magData = bmm150.getGeomagneticData();
            Quaternion q(magData.x, magData.y, magData.z);
            q = rot.rotate(q);
            return Vec3(q.b, q.c, q.d);
        }

        float getCompassDegree(){
            float compass = 0.0;
            Vec3 mag = readMag();
            compass = atan2(mag.x, mag.y);
            if (compass < 0) {
                compass += 2 * PI;
            }
            if (compass > 2 * PI) {
                compass -= 2 * PI;
            }
            return compass * 180 / M_PI;
        }

    private:

        short int readShort(short address)
        {
            short b1 = EEPROM.read(address);
            short b2 = EEPROM.read(address+1);
            return (b1<<8)+b2;
        }

        void writeShort(short address, short data)
        {
            EEPROM.write(address, data>>8);
            EEPROM.write(address+1, (data<<8)>>8);
        }

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


};

#endif