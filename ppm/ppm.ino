#include <Arduino.h>
#include "ppm.h"

void setup() {
  Serial.begin(9600);
  setup_ppm();
}

void loop() {
  print_channels(true);
  //print_percents();
  delay(100);
}
