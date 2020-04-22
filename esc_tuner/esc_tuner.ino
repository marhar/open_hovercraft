// Tune the ESCs

#include <Servo.h>
#include <ArrbotMonitor.h>
#include "ppm.h"

Servo s_lmotor;
Servo s_rmotor;
Servo s_lmonitor;
Servo s_rmonitor;

#define MIN_SERVO 1000
#define MAX_SERVO 2000

void setmotor(Servo &s, int v) {
  if (v < MIN_SERVO)
    s.write(MIN_SERVO);
  else if (v > MAX_SERVO)
    s.write(MAX_SERVO);
  else
    s.write(v);
}

void x_lmotor(int x) { setmotor(s_lmotor,x); setmotor(s_lmonitor, x); }
void x_rmotor(int x) { setmotor(s_rmotor,x); setmotor(s_rmonitor, x); }

void setup() {
  Serial.begin(9600);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
  setup_ppm();

  s_lmotor.attach(3);
  s_rmotor.attach(9);
  s_lmonitor.attach(A1);
  s_rmonitor.attach(A2);

  x_lmotor(MIN_SERVO);
  x_rmotor(MIN_SERVO);
  digitalWrite(LED_BUILTIN, LOW);
}

#define THR_CHAN 3
void loop() {
  int thr = read_channel_time(THR_CHAN);
  MONITOR(thr);
  MONITOR_ENDL();
  x_lmotor(thr);
  x_rmotor(thr);
}
