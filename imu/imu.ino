#include <Arduino.h>
#include <ArduinoMonitor.h>
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

float estimated_az = 0;
void loop() {
  float az = mpu.getAngleZ();
  estimated_az = myfilter.updateEstimate(az);
  mpu.update();
  //MONITOR2("AccX", mpu.getAccX());
  //MONITOR2("AccY", mpu.getAccY());
  //MONITOR2("AccZ", mpu.getAccZ());
  //MONITOR2("AngX", mpu.getAngleX());
  //MONITOR2("AngY", mpu.getAngleY());
  //MONITOR2("AngZ", mpu.getAngleZ());
  MONITOR(az);
  MONITOR(estimated_az);
  MONITOR_ENDL();
}
