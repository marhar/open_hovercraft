// OpenHover
#include <Servo.h>
#include "imu.h"
#include "ppm.h"

Servo s_lmotor;
Servo s_rmotor;
Servo s_lifter;
Servo s_lmonitor;
Servo s_rmonitor;

void x_lmotor(int x) { s_lmotor.write(x); s_lmonitor.write(x); }
void x_rmotor(int x) { s_rmotor.write(x); s_rmonitor.write(x); }
void x_lifter(int x) { s_lifter.write(x); }

void setup() {
  Serial.begin(115200);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
  setup_imu();
  setup_ppm();
  //setup_servo();
  digitalWrite(LED_BUILTIN, LOW);

  s_lmotor.attach(3);
  s_rmotor.attach(9);
  s_lifter.attach(10);
  s_lmonitor.attach(11);

  x_lmotor(0);
  x_rmotor(0);
  x_lifter(0);
}

// output a value in plotter-compatible format. usage: MON("x:", x);
#define P(x) Serial.print(x)
#define MON(n, v) P('\t'); P(F(n)); P(v);
#define NL() P('\n')

// junk below?
#define PL(x) Serial.println(x)
#define P2(x) Serial.print('\t'); Serial.print(x)
#define PL2(x) Serial.print('\t'); Serial.println(x)

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
  x_lmotor(map(m1, -100, 100, 0, 180));
  x_rmotor(map(m2, -100, 100, 0, 180));
  if (1) {
    MON("ang:", ang);
    MON("thr:", thr);
    MON("rud:", rud);
    MON("m1:", m1);
    MON("m2:", m2);
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
