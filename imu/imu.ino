#include <Arduino.h>
#include "imu.h"
#include "kalman.h"


#include "Wire.h"
#include <MPU6050_light.h>

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
  Serial.begin(9600);
  setup_imu();
  digitalWrite(LED_BUILTIN, LOW);
  
}

Kalman1d myfilter(2, 2, 0.9);

#define P(x) Serial.print(x)

#ifndef NO_MONITOR
#define MONITOR(v) Serial.print(F(" " #v ":")); Serial.print(v)
#define MONITOR2(n, v) Serial.print(F(" " n ":")); Serial.print(v)
#define MONITOR_END Serial.println()
#endif

float estimated_az = 0;
void loop() {
  float az = mpu.getAngleZ();
  estimated_az = myfilter.updateEstimate(az);
  mpu.update();
  delay(100);
  //MONITOR("ACCEL\t"));P(mpu.getAccX());
  //MONITOR("\t"));P(mpu.getAccY());
  //MONITOR("\t"));P(mpu.getAccZ());
  //MONITOR("\tANGLE\t"));P(mpu.getAngleX());
  //MONITOR("\t"));P(mpu.getAngleY());
  //MONITOR("\t"));PL(mpu.getAngleZ());
  MONITOR(az);
  MONITOR(estimated_az);
  Serial.println();
}
