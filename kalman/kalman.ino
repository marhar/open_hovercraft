#include <ArrbotMonitor.h>
#include "kalman.h"

void setup() {
  Serial.begin(9600);
}

const long SERIAL_REFRESH_TIME = 100;
long refresh_time;

Kalman1d myfilter(2, 2, 0.01);
Kalman1d myfilter2(2, 2, 0.01);
void loop() {
  float real_value = analogRead(A0)/1024.0 * 100.0;
  float measured_value = real_value + random(-100,100)/100.0;
  float estimated_value = myfilter.updateEstimate(measured_value);
  float estimated_value2 = myfilter.updateEstimate(real_value);

  if (millis() > refresh_time) {
    MONITOR(real_value);
    MONITOR(measured_value);
    MONITOR(estimated_value);
    MONITOR(estimated_value2);
    MONITOR_ENDL();
    refresh_time = millis() + SERIAL_REFRESH_TIME;
  }

}
