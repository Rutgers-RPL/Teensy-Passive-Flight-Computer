#include "Mat3x3.h"
#include "Vec3.h"
#include "stdio.h"

int main() {
    Vec3 pogvecy(1, 2, 3);
    Mat3x3 pogmaty(Vec3(1, 2, 3), Vec3(4, 5, 6), Vec3(7, 8, 9));

    Vec3 baddy = pogmaty * pogvecy;

    printf("%f\t%f\t%f\n", baddy.x, baddy.y, baddy.z);          
}