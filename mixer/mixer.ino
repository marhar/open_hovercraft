#include <Arduino.h>
#include <Servo.h>
#include "mixer.h"
#include "ppm.h"


Servo s1;
Servo s2;

void setup() {
  Serial.begin(9600);
  setup_ppm();
  setup_mixer();
  s1.attach(9);
  s2.attach(10);
}

#define P2(x) Serial.print('\t'); Serial.print(x)
#define PL2(x) Serial.print('\t'); Serial.println(x)

#define THROTTLE_CHANNEL 3
#define RUDDER_CHANNEL 4
unsigned int junk;
void loop() {

  //print_channels(junk++);
  //print_percents(junk++);
  int thr = channel_percent(THROTTLE_CHANNEL-1);
  int rud = channel_percent(RUDDER_CHANNEL-1);

  int m1 = min(100, thr + rud);
  int m2 = max(-100, thr - rud);
  P2(thr);P2(rud);P2(m1);PL2(m2);
  // TODO: handle dead zone, don't write unless diff > 1%
  s1.write(map(m1, -100, 100, 10, 135));
  s2.write(map(m2, -100, 100, 45, 135));
  delay(100);
}
