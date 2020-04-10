// OpenHover
#include <Servo.h>
#include "imu.h"
#include "ppm.h"

Servo s1;
Servo s2;
Servo s3;

void setup() {
  Serial.begin(115200);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
  setup_imu();
  setup_ppm();
  //setup_servo();
  digitalWrite(LED_BUILTIN, LOW);

  s1.attach(9);
  s2.attach(10);
  s3.attach(11);
  
}

#define P(x) Serial.print(x)
#define PL(x) Serial.println(x)
#define P2(x) Serial.print('\t'); Serial.print(x)
#define PL2(x) Serial.print('\t'); Serial.println(x)

void loop() {
  static int mode;

  /////ppm_get()
  mpu.update();
  delay(100);
    Serial.print(F("ACCEL\t"));Serial.print(mpu.getAccX());
    Serial.print("\t");Serial.print(mpu.getAccY());
    Serial.print("\t");Serial.print(mpu.getAccZ());
  
    Serial.print(F("\tANGLE\t"));Serial.print(mpu.getAngleX());
    Serial.print("\t");Serial.print(mpu.getAngleY());
    Serial.print("\t");Serial.println(mpu.getAngleZ());
}
