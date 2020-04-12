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

int switchpos(int val) {
  return 0;
}

// output a value in plotter-compatible format. usage: MON("x:", x);
#define P(x) Serial.print(x)
#define MON(n, v) P(' '); P(F(n)); P(v);
#define NL() P('\n')

/////////////////////// PID stuff
float pVal = 0;
float iVal = 0;
float dVal = 0;
float target = 0;
float cumError;
float maxCorr;
float minCorr;

#define THR_CHANNEL 3
#define RUD_CHANNEL 4
#define MODE_CHANNEL 8

#define MODE_MANUAL 1
#define MODE_HEADING_HOLD 2
#define MODE_RATES 3 // ???

float old_angle;
float target_angle = 0.0;

void loop() {
  // TODO: more precise loop control
  int mode;
  mpu.update();
  float angle = mpu.getAngleZ();
  int thr = read_channel_percent(THR_CHANNEL);
  int rud = read_channel_percent(RUD_CHANNEL) / 10; // low rates!
  int mode_switch = read_channel_percent(MODE_CHANNEL);
  if (mode_switch > 50)
    mode = MODE_MANUAL;
  else if (mode_switch > -50)
    mode = MODE_RATES;
  else
    mode = MODE_HEADING_HOLD;
    

  // update
  int m1;
  int m2;
  // TODO: macroize switch position
  if (mode == MODE_MANUAL) {
    // switch down
    m1 = thr + rud;
    m2 = thr - rud;
  }
  else if (mode == MODE_RATES) {
    goto heading_mode; // for now just treat as heading mode
    m1 = thr + rud;
    m2 = thr - rud;
  }
  else { // mode == MODE_HEADING_HOLD

    // TODO: when initialized, dont do anything until THR stick down
    heading_mode:
    static float pVal = .1;
    float err = target_angle - angle;
    float pCorrection = pVal * err;

    float correction = pCorrection;
    
    MON("oang:", old_angle);
    MON("ang:", angle);
    MON("tar:", target_angle);
    MON("err:", err);
    MON("pcor:", pCorrection);
    MON("corr:", correction);
    
    m1 = thr + rud - correction;
    m2 = thr - rud + correction;

    if (thr < -98) {
      // if throttle off, lets reset some stuff
      m1 = m2 = -100;  // force motors off
      target = angle;  // set new target angle
    }
  }

  // output
  x_lmotor(map(m1, -100, 100, MIN_SERVO, MAX_SERVO));
  x_rmotor(map(m2, -100, 100, MIN_SERVO, MAX_SERVO));
  if (1) {
    MON("thr:", thr);
    MON("rud:", rud);
    MON("m1:", m1);
    MON("m2:", m2);
    P('\n');
    delay(70);
  }
  delay(30);
  old_angle = angle;
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
