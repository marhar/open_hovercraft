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

  s1.write(10);
  s2.write(10);
  s3.write(10);
  
}

#define P(x) Serial.print(x)
#define PL(x) Serial.println(x)
#define P2(x) Serial.print('\t'); Serial.print(x)
#define PL2(x) Serial.print('\t'); Serial.println(x)

#define PV0(a,b) P(a);P(b)
#define PV(a,b) P('\t'); PV0(a,b)
#define NL() Serial.print('\n')

/////////////////////// PID stuff
float pVal = 0;
float iVal = 0;
float dVal = 0;
float target = 0;
float cumError;
float maxCorr;
float minCorr;

void loop() {
  // input
  mpu.update();
  float ang = mpu.getAngleZ();
  int thr = read_channel_percent(3);
  int rud = read_channel_percent(4);
  int mode_switch = read_channel_percent(8);

  // update
  int m1;
  int m2;
  if (mode_switch > 50) {
    // switch up, manual mode
    m1 = thr + rud;
    m2 = thr - rud;
  }
  else if (mode_switch > -50) {
    // switch middle, rate mode (??)
    goto heading_mode;
    m1 = thr + rud;
    m2 = thr - rud;
  }
  else {
    // switch down, heading mode
    heading_mode:
    m1 = thr + rud;
    m2 = thr - rud;
  }

  // output
  s1.write(map(m1, -100, 100, 0, 180));
  s2.write(map(m2, -100, 100, 0, 180));
  if (1) {
    PV0(F("ang:"), ang);
    PV(F("thr:"), thr);
    PV(F("rud:"), rud);
    PV(F("m1:"), m1);
    PV(F("m2:"), m2);
    NL();
  }
  delay(100);
}


/*
pVal * current_error

pVal
iVal
dVal
terget
cumErr
maxCorr = 100% power
minCorr = 15% power
target = 100   set point
loop
  read currVal
  err = target - currVal
  
  pCorrection = pVal * error
  
  cumError += error
  iCorr = iVal * cumError
  
  slope = error - lastErr
  dCorr = dVal * slope
  lastErr = err

  corr = pCorr + iCorr + dCorr
  if (corr > maxCorr) corr = maxCorr
  if (corr < minCorr) corr = minCorr

  emit(correction)
  


*/