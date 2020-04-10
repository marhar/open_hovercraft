#include <Arduino.h>
#include "ppm.h"

void setup() {
  Serial.begin(115200);
  setup_ppm();
}

unsigned int junk;
void loop() {
  //print_channels(junk++);
  print_percents(junk++);
  delay(100);
}
