// OpenHover
// get ArrbotMonitor from https://github.com/marhar/ArrbotMonitor
//     baud      pgm int    hz
//    57600  monitor usb   227

#define BAUD 9600

#include <Servo.h>
#include <ArrbotMonitor.h>
#include "imu.h"
#include "ppm.h"
#include "kalman.h"


// TX Settings:
//  Ch 3.  Thr
//  Ch 4.  Rud
//  Ch 5.  S1    lift motor
//  Ch 6.  SD    flight mode gyro/manual/off

// Configuration stuff.

#define MIN_SERVO 1000
#define MAX_SERVO 2000
#define THR_CHANNEL 3
#define RUD_CHANNEL 4
#define LIFT_CHANNEL 5
#define MODE_CHANNEL 6

enum { SWITCH_UP = 1, SWITCH_MIDDLE, SWITCH_DOWN };

// Outputs.

Servo s_lmotor;
Servo s_rmotor;
int s_lifter;
Servo s_lmonitor;
Servo s_rmonitor;

// Some motor output stuff

void setmotor(Servo &s, int v, int reverse=0) {
  if (v < MIN_SERVO)
    v = MIN_SERVO;
  else if (v > MAX_SERVO)
    v = MAX_SERVO;
  if (reverse)
    v = MAX_SERVO - (v - MIN_SERVO);
  s.write(v);
}

void x_lmotor(int x) { setmotor(s_lmotor,x); setmotor(s_lmonitor, x, 1); }
void x_rmotor(int x) { setmotor(s_rmotor,x); setmotor(s_rmonitor, x); }
void x_lifter(int x) { 
  if (x == SWITCH_UP) {
    digitalWrite(s_lifter,HIGH);
    }
  else {
    digitalWrite(s_lifter,LOW); 
  }
}

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
  digitalWrite(LED_BUILTIN, HIGH);   // led on, setup started
  pinMode(10,OUTPUT);

  Serial.begin(BAUD);
  //setup_servo();
  s_lmotor.attach(3);
  s_rmotor.attach(9);
  s_lmonitor.attach(A1);
  s_rmonitor.attach(A2);
  s_lifter = 10;
  x_lmotor(0);
  x_rmotor(0);
  x_lifter(SWITCH_DOWN);
  setup_ppm();
  setup_imu();
  mpu.setGyroOffsets(0, 0, 0);
  digitalWrite(LED_BUILTIN, LOW);  // led off, finished setup
}

// loop() persistent data

// loop stats
uint32_t old_sec;
uint32_t loop_count;
int loop_hz;

int watcher = 0;    // watch mode -- start off quiet
float target_angle = 0.0;
Kalman1d gyroz_filter(2, 2, 0.15);

enum {STRAIGHT = 1, TURNING, STOPPED};
int action_state = STOPPED;
#define DEAD_WIDTH 5                // width in percent of rudder dead zone

// TODO: move all vars out or in

void loop() {
  uint32_t now = micros();
  ++loop_count;
  if (now >= old_sec + 1000000) {
    old_sec = now;
    loop_hz = loop_count;
    loop_count = 0;
  }
  mpu.update();
  
  // TODO: make this configurable?
  // TODO: can we nuke floats from channels?
  float thr = (read_channel_percent(THR_CHANNEL)+100)/2;
  float rud = ((read_channel_percent(RUD_CHANNEL)+100) / 200.0); // add low rates?
  if (thr < 0) {
    digitalWrite(LED_BUILTIN, HIGH);
  } else {
    digitalWrite(LED_BUILTIN, LOW);
  }
  int lifter = switch_position(read_channel_percent(LIFT_CHANNEL));
  int flight_mode = switch_position(read_channel_percent(MODE_CHANNEL));
  // TODO: when initialized, dont do anything until THR stick down
  // TODO: on angle read, normalize to 360 degrees?
  float raw_angle = mpu.getAngleZ();
  float current_angle = gyroz_filter.updateEstimate(raw_angle);

  int m1;  // motor 1 -- left thrust
  int m2;  // motor 2 -- right thrust
  int m3;  // motor 3 -- lifter

  m3 = lifter;
  // TODO: tidy up where pid vars go
  static float pCoef = 20;
  static float iCoef = .05;
  static float dCoef = .1;
  
  static float last_err;
  static float accumulated_error = 0;
  float err = target_angle - current_angle;
  accumulated_error += err;
  float pCorr = pCoef/100.0 * err;
  float iCorr = iCoef/100.0 * accumulated_error;
  float dCorr = dCoef/100.0 * (err / last_err);
  float total_correction = pCorr + iCorr + dCorr;
  last_err = err;
  
  float motor_delta = total_correction;

// TODO: figure out what MOTOR_DELTA_DIVISOR means
#define MOTOR_DELTA_DIVISOR 20
  m1 = thr + rud/2 - motor_delta/MOTOR_DELTA_DIVISOR;
  m2 = thr - rud/2 + motor_delta/MOTOR_DELTA_DIVISOR;

  if (1 && Serial.available()) {
    byte k = Serial.read();
    switch (k) {
    case 'p': pCoef = Serial.parseFloat(); break;
    case 'i': iCoef = Serial.parseFloat(); break;
    case 'd': dCoef = Serial.parseFloat(); break;
    // TODO: figure out character read so wl, wp will work
    case 'w': watcher = Serial.parseInt(); break;
    case 'b': Serial.begin(Serial.parseInt()); break; // doesn't work?
}
  }
  // TODO: figure out good command for w[1234]
  // w1=pid w2 = motors w3=sensors w4=channels
  static int watch_count = 0;
  watch_count++;
  if (watch_count > 10) {
    watch_count = 0;
  switch (watcher) {
  case 1://'p':  // PID loop
    DISPLAY2("hz", loop_hz);
    DISPLAY2("p", pCoef);
    DISPLAY2("i", iCoef);
    DISPLAY2("d", dCoef);
    MONITOR2("pc", pCorr);
    MONITOR2("ic", iCorr);
    MONITOR2("dc", dCorr);
    MONITOR2("t", total_correction);
    MONITOR2("e", err);
    MONITOR_ENDL();
    break;
  case 2://'m':  // motors
    MONITOR(m1);
    MONITOR(m2);
    MONITOR(m3);
    MONITOR_ENDL();
    break;
  case 3://'s':  // sensors
    MONITOR(raw_angle);
    MONITOR(current_angle);
    MONITOR_ENDL();
    break;
  case 4://'c':  // RC Channels
    MONITOR2("ch1", read_channel_percent(1));
    MONITOR2("ch2", read_channel_percent(2));
    MONITOR2("ch3", read_channel_percent(3));
    MONITOR2("ch4", read_channel_percent(4));
    MONITOR2("ch5", read_channel_percent(5));
    MONITOR2("ch6", read_channel_percent(6));
    MONITOR2("ch7", read_channel_percent(7));
    MONITOR2("ch8", read_channel_percent(8));
    MONITOR_ENDL();
    break;
  case 5://'l':  // loop stats
    DISPLAY(loop_hz);
    MONITOR(loop_hz);
    MONITOR_ENDL();
    break;
  }
  }


  // middle pos = manual flight mode
  if (action_state == TURNING || flight_mode == SWITCH_MIDDLE) {
    m1 = thr*rud-100;
    m2 = thr*(1-rud)-100;
  }

  // bottom pos = stop everything
  if (flight_mode == SWITCH_DOWN || thr < -98) {
    // if throttle is off, lets reset some stuff
    m1 = m2 = -100;  // force thrust motors off
    action_state = STOPPED;
    target_angle = current_angle;  // set new target angle  //?REDO
    accumulated_error = 0;  //?REDO
  }

  x_lmotor(map(m1, -100, 100, MIN_SERVO, MAX_SERVO));
  x_rmotor(map(m2, -100, 100, MIN_SERVO, MAX_SERVO));
  x_lifter(lifter);

  // TODO: do we want to re-add rate limiting here?
}
