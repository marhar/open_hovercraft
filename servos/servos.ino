#include <Servo.h>

#include "ppm.h"

Servo s1;
Servo s2;
Servo s3;

void setup() {
  setup_ppm();
  Serial.begin(115200);
  s1.attach(9);
  s2.attach(10);
  s3.attach(11);
}

unsigned int junk;
void loop() {
  //print_channels(junk++);
  print_percents(junk++);
  // TODO: make canonical limits, put into servo.h?
  // TODO: make canonical header with channels, output names?
  s1.write(map(channel_times[1-1], MIN_TIME, MAX_TIME, 10, 170));
  s2.write(map(channel_times[4-1], MIN_TIME, MAX_TIME, 10, 170));
  s3.write(map(channel_times[6-1], MIN_TIME, MAX_TIME, 10, 170));
  delay(100);
}
