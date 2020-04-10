#include <Arduino.h>
#include "imu.h"


#include "Wire.h"
#include <MPU6050_light.h>

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
  Serial.begin(115200);
  setup_imu();
  digitalWrite(LED_BUILTIN, LOW);
  
}

#define P(x) Serial.print(x)
#define PL(x) Serial.println(x)
void loop() {
  mpu.update();
  delay(100);
  P(F("ACCEL\t"));P(mpu.getAccX());
  P(F("\t"));P(mpu.getAccY());
  P(F("\t"));P(mpu.getAccZ());
  P(F("\tANGLE\t"));P(mpu.getAngleX());
  P(F("\t"));P(mpu.getAngleY());
  P(F("\t"));PL(mpu.getAngleZ());
}

