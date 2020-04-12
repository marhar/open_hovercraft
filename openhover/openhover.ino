// OpenHover
#include <Servo.h>

#include "imu.h"
#include "ppm.h"

Servo s_lmotor;
Servo s_rmotor;
Servo s_lifter;
Servo s_lmonitor;
Servo s_rmonitor;

#define MIN_SERVO 1000
#define MAX_SERVO 2000

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

void setup() {
  Serial.begin(115200);
  pinMode(LED_BUILTIN, OUTPUT);

  // Turn on blue LED while in setup.
  digitalWrite(LED_BUILTIN, HIGH);
  setup_ppm();
  //setup_servo();
  digitalWrite(LED_BUILTIN, LOW);

  s_lmotor.attach(3);
  s_rmotor.attach(9);
  s_lmonitor.attach(A1);
  s_rmonitor.attach(A2);
    s_lifter.attach(10);

  x_lmotor(0);
  x_rmotor(0);
  x_lifter(0);
  setup_imu();
}

enum { SWITCH_UP = 1, SWITCH_MIDDLE, SWITCH_DOWN };

int switch_position(int percentage) {
  if (percentage > 50)
    return SWITCH_DOWN;
  else if (percentage > -50)
    return SWITCH_MIDDLE;
  else
    return SWITCH_UP;
}

// output a value in plotter-compatible format. usage: MON("x:", x);
#define P(x) Serial.print(x)
#define MON(n, v) P(' '); P(F(n)); P(v);


#define THR_CHANNEL 3
#define RUD_CHANNEL 4
#define MODE_CHANNEL 8

#define MODE_MANUAL 1
#define MODE_HEADING_HOLD 2
#define MODE_RATES 3 // ???

float old_angle;
float target_angle = 0.0;

#define LOOP_HERTZ 50
void loop() {
  // TODO: more precise loop control
  uint32_t now = micros;
  uint32_t wait_until = now + 1000000/LOOP_HERTZ;
  int mode;
  mpu.update();
  float current_angle = mpu.getAngleZ();
  float thr = read_channel_percent(THR_CHANNEL);
  float rud = read_channel_percent(RUD_CHANNEL) / 10.0; // low rates!
  float mode_switch = read_channel_percent(MODE_CHANNEL);

  switch (switch_position(read_channel_percent(MODE_CHANNEL))) {
    case SWITCH_UP:
      mode = MODE_HEADING_HOLD;
      break;
    case SWITCH_MIDDLE:
      //mode = MODE_RATES;
      mode = MODE_HEADING_HOLD;
      break;
    case SWITCH_DOWN:
      mode = MODE_MANUAL;
      break;
  }

  int m1;
  int m2;
  if (mode == MODE_MANUAL) {
    m1 = thr + rud;
    m2 = thr - rud;
  }
  else if (mode == MODE_RATES) {
    goto heading_mode; // for now just treat as heading mode
  }
  else { // mode == MODE_HEADING_HOLD
    // TODO: when initialized, dont do anything until THR stick down
    heading_mode:
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
    if (pcount > 10) {
    pcount = 0;
    MON("tar:", target_angle);
    MON("ang:", current_angle);
    MON("err:", err);
    MON("pcor:", pCorrection);
    MON("dcor:", dCorrection);
    MON("accum:", accumulated_error);
    MON("tcor:", total_correction);
    MON("del:", motor_delta);
    MON("m1:", m1);
    MON("m2:", m2);
    Serial.println("");
    delay(50);
    }

    if (thr < -98) {
      // if throttle off, lets reset some stuff
      m1 = m2 = -100;  // force motors off
      target_angle = current_angle;  // set new target angle
      accumulated_error = 0;
    }
  }

  x_lmotor(map(m1, -100, 100, MIN_SERVO, MAX_SERVO));
  x_rmotor(map(m2, -100, 100, MIN_SERVO, MAX_SERVO));
  // TODO: change this into a simple loop that doesn't return
  old_angle = current_angle;
  while (micros() < wait_until)
    ;
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
