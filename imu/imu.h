#ifndef __include_mpu_h
#define __include_mpu_h
// Uses library: https://github.com/rfetick/MPU6050_light

#include <Arduino.h>
#include "Wire.h"
#include <MPU6050_light.h>

extern MPU6050 mpu;
extern void setup_imu();
#endif
