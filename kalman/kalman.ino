#include "kalman.h"

void setup() {
  Serial.begin(115200);
}

#define P(x) Serial.print(x)
#define MON(n,v) Serial.print(F(n)); Serial.print(v); Serial.print(' ');

const long SERIAL_REFRESH_TIME = 100;
long refresh_time;

Kalman1d myfilter(2, 2, 0.01);
Kalman1d myfilter2(2, 2, 0.01);
void loop() {
  float real_value = analogRead(A0)/1024.0 * 100.0;
  float measured_value = real_value + random(-100,100)/100.0;
  float estimated_value = myfilter.updateEstimate(measured_value);
  float estimated_value2 = myfilter.updateEstimate(real_value);

  // send to Serial output every 100ms
  // use the Serial Ploter for a good visualization
  if (millis() > refresh_time) {
    MON("real:",  real_value);
    MON("measured:", measured_value);
    MON("estimated:", estimated_value);
    MON("estimated2:", estimated_value2);
    Serial.println();
    
    refresh_time = millis() + SERIAL_REFRESH_TIME;
  }

}
