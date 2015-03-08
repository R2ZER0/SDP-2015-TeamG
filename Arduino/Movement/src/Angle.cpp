/* Angle.cpp */
#include "Angle.h"

float Angle::normalise(float a) {
    while(a > PI) { a -= PI; }
    while(a < (-PI)) { a += PI; }
    return a;
}