// OpenHover

#include <Servo.h>
#include "imu.h"
#include "ppm.h"
#include "tiny_kalman.h"

// Configuration stuff.

#define MIN_SERVO 1000
#define MAX_SERVO 2000

#define THR_CHANNEL 3
#define RUD_CHANNEL 4
#define MODE_CHANNEL 8    // TODO change to 6

#define LOOP_HERTZ 50

enum { SWITCH_UP = 1, SWITCH_MIDDLE, SWITCH_DOWN };

// output a value in plotter-compatible format. usage: MONITOR(x);
#define MONITOR(v) Serial.print(" " #v ":"); Serial.print(v)
#define MONITOR2(name, v) Serial.print(" " name ":"); Serial.print(v)

// Outputs.

Servo s_lmotor;
Servo s_rmotor;
Servo s_lifter;
Servo s_lmonitor;
Servo s_rmonitor;

// Some motor output stuff

void setmotor(Servo &s, int v, int reverse=0) {
  if (v < MIN_SERVO)
    v = MIN_SERVO;
  else if (v > MAX_SERVO)
    v = MAX_SERVO;
  //if (reverse)
  //  v = MAX_SERVO - v;
  s.write(v);
}

void x_lmotor(int x) { setmotor(s_lmotor,x); setmotor(s_lmonitor, x, 1); }
void x_rmotor(int x) { setmotor(s_rmotor,x); setmotor(s_rmonitor, x); }
void x_lifter(int x) { setmotor(s_lifter,x); }

// Some input stuff.

int switch_position(int percentage) {
  // map a switch percentage to up/middle/down.
  if (percentage > 50)
    return SWITCH_DOWN;
  else if (percentage > -50)
    return SWITCH_MIDDLE;
  else
    return SWITCH_UP;
}

float smooth(float x, float new_x, int nsamples) {
  return (x * (nsamples - 1.0) + new_x) / (float)nsamples;
}

void setup() {
  // Turn on blue LED while in setup.
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);

  Serial.begin(115200);
  setup_imu();
  setup_ppm();
  //setup_servo();
  s_lmotor.attach(3);
  s_rmotor.attach(9);
  s_lmonitor.attach(A1);
  s_rmonitor.attach(A2);
  s_lifter.attach(10);
  x_lmotor(0);
  x_rmotor(0);
  x_lifter(0);
  digitalWrite(LED_BUILTIN, LOW);
}


// loop() persistent data

float target_angle = 0.0;
SimpleKalmanFilter gyroz_filter(2, 2, 0.01);

// TODO: move all vars out or in

void loop() {
  uint32_t now = micros();
  uint32_t wait_until = now + 1000000/LOOP_HERTZ;
  mpu.update();

  // TODO: make this configurable?
  // TODO: can we nuke floats from channels?
  float thr = read_channel_percent(THR_CHANNEL);
  float rud = read_channel_percent(RUD_CHANNEL) / 10.0; // low rates!
  int flight_mode = switch_position(read_channel_percent(MODE_CHANNEL));

  // TODO: on angle read, normalize to 360 degrees?
  float raw_angle = mpu.getAngleZ();
  float current_angle = gyroz_filter.updateEstimate(raw_angle);

  int m1;
  int m2;

  // TODO: tidy up where pid vars go
  // TODO: make pid vars tunable/displayable by bluetooth
  // TODO: when initialized, dont do anything until THR stick down
  static float pCoefficient = .2;
  static float dCoefficient = .02;
  static float accumulated_error = 0;
  float err = target_angle - current_angle;
  if (abs(err) > 1.0)
    accumulated_error += err;
  float pCorrection = pCoefficient * err;
  float dCorrection = dCoefficient * accumulated_error;

  float total_correction = pCorrection + dCorrection;

  float motor_delta = total_correction;
  
  m1 = thr + rud - motor_delta/2;
  m2 = thr - rud + motor_delta/2;

  static int pcount;
  pcount++;
  if (1 && pcount > 10) {
    pcount = 0;
    //MONITOR(target_angle);
    MONITOR(current_angle);
    //MONITOR(err);
    //MONITOR(pCorrection);
    //MONITOR(dCorrection);
    //MONITOR(accumulated_error);
    //MONITOR(total_correction);
    //MONITOR(motor_delta);
    //MONITOR(m1);
    //MONITOR(m2);
    Serial.println("");
    // do we need extra time to handle graphing?
    delay(50);
  }

  // middle pos = manual flight mode
  if (flight_mode == SWITCH_MIDDLE) {
    m1 = thr + rud;
    m2 = thr - rud;
  }

  // bottom pos = stop everything
  if (flight_mode == SWITCH_DOWN || thr < -98) {
    // if throttle is off, lets reset some stuff
    m1 = m2 = -100;  // force motors off
    target_angle = current_angle;  // set new target angle  //?REDO
    accumulated_error = 0;  //?REDO
  }

  x_lmotor(map(m1, -100, 100, MIN_SERVO, MAX_SERVO));
  x_rmotor(map(m2, -100, 100, MIN_SERVO, MAX_SERVO));

  while (micros() < wait_until) {
  }
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
