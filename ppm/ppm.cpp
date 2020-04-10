#include <Arduino.h>
#include "ppm.h"

unsigned long channel_times[NCHANNELS];
unsigned long last_tick;

int read_channel_time(int channel_num) {
  return channel_times[channel_num - 1];
}

int read_channel_percent(int channel_num) {
  unsigned long v = channel_times[channel_num - 1];
  if (v < MIN_TIME)
    return -100;
  if (v > MAX_TIME)
    return 100;
  return map(v, MIN_TIME, MAX_TIME, -100, 100);
}

int channel_percent(int i) {
  // adding 2 makes the midpoint hit 0% more often 
  unsigned long v = channel_times[i] + 2;
  if (v < MIN_TIME)
    return -100;
  if (v > MAX_TIME)
    return 100;
  return map(v, MIN_TIME, MAX_TIME, -100, 100);
}

void print_channels(int stuff) {
  Serial.print(stuff);
  Serial.print('\t');
  for (int i = 0; i < NCHANNELS; ++i) {
    Serial.print('\t');
    Serial.print(channel_times[i]);
  }
  Serial.print('\n');
}

void print_percents(int stuff) {
  Serial.print(stuff);
  Serial.print('\t');
  for (int i = 1; i <= NCHANNELS; ++i) {
    Serial.print('\t');
    Serial.print(read_channel_percent(i));
  }
  Serial.print('\n');
}

void handle_tick()  {
  static int current_channel = 0;
  unsigned long tick = micros();
  unsigned long diff = tick - last_tick;
  last_tick = tick;

  if (diff > 2500) {
    current_channel = 0;
    return;
  }
  if (current_channel < NCHANNELS)
    channel_times[current_channel++] = diff;
}

void setup_ppm() {
  last_tick = micros();
  delay(10);
  // TODO: handle tick wraparound
  pinMode(2, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(2), handle_tick, FALLING);
}
