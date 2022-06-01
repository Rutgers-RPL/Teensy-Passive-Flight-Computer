#ifndef Vec3_H
#define Vec3_H
#include <Arduino.h>
class Vec3{
    public:
        double x;
        double y;
        double z;

        Vec3(double tx, double ty, double tz){
            x = tx;
            y = ty;
            z = tz;
        }

        double magnitude(){
            return sqrt(pow(x,2)+pow(y,2)+pow(z,2));
        }
        double normalize(){
            double m = magnitude();
            x/=m;
            y/=m;
            z/=m;
        }
        Vec3 cross(Vec3 vec){
            return Vec3(y*vec.z-z*vec.y,z*vec.x-x*vec.z,x*vec.y-y*vec.x);
        }
        double dot(Vec3 vec){
            return x*vec.x+y*vec.y+z*vec.z;
        }
        Vec3 operator*(const double s) const{ 
            return Vec3(x * s, y * s, z*s); 
        }

        friend Vec3 operator*(const double s, const Vec3& v);
};
Vec3 operator*(const double s, const Vec3& v){
    return Vec3(v.x * s, v.y * s,v.z * s);
}
double angleBetweenVec3(Vec3 a,Vec3 b){
    return (180/PI)*acos(a.dot(b)/(a.magnitude()*b.magnitude()));// read somewhere that this isnt the fastest arccos function available
}
#endif