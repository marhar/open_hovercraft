#include "imu.h"
// call mpu.update()

MPU6050 mpu(Wire);

void setup_imu() {
  Wire.begin();
  mpu.begin();
  mpu.calcGyroOffsets();
}
