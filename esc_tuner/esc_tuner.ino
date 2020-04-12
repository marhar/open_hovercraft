// Tune the ESCs

#include <Servo.h>
#include "ppm.h"

Servo s_lmotor;
Servo s_rmotor;
Servo s_lmonitor;
Servo s_rmonitor;

#define MINSERVO 1000
#define MAXSERVO 2000

void setmotor(Servo &s, int v) {
  if (v < MINSERVO)
    s.write(MINSERVO);
  else if (v > MAXSERVO)
    s.write(MAXSERVO);
  else
    s.write(v);
}

void x_lmotor(int x) { setmotor(s_lmotor,x); setmotor(s_lmonitor, x); }
void x_rmotor(int x) { setmotor(s_rmotor,x); setmotor(s_rmonitor, x); }

void setup() {
  Serial.begin(115200);
  setup_ppm();

  s_lmotor.attach(3);
  s_rmotor.attach(9);
  s_lmonitor.attach(A1);
  s_rmonitor.attach(A2);

  x_lmotor(0);
  x_rmotor(0);
}

// output a value in plotter-compatible format. usage: MON("x:", x);
#define P(x) Serial.print(x)
#define MON(n, v) P('\t'); P(F(n)); P(v);

#define THR_CHAN 3
void loop() {
  int thr = read_channel_time(THR_CHAN);
  x_lmotor(thr);
  x_rmotor(thr);
  if (1) {
    MON("thr:", thr);
    P('\n');
  }
  delay(100);
}
