#ifndef RAYCAST_H_
#define RAYCAST_H_

#include "Geometry.h"
#include <iostream>

float signum(int x);
float mod(float value, float modulus);
float intbound(float s, int ds);
void Raycast(const chisel::Vec3& start, const chisel::Vec3& end, const chisel::Point3& min, const chisel::Point3& max, chisel::Point3List* output);

#endif // RAYCAST_H_ 
