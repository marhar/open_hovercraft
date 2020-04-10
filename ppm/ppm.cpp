#include <Arduino.h>
#include "ppm.h"

uint32_t channel_times[NCHANNELS];
uint32_t last_tick;

int read_channel_time(int channel_num) {
  return channel_times[channel_num - 1];
}

int read_channel_percent(int channel_num) {
  uint32_t v = channel_times[channel_num - 1];
  if (v < MIN_TIME)
    return -100;
  if (v > MAX_TIME)
    return 100;
  return map(v, MIN_TIME, MAX_TIME, -100, 100);
}

void print_percents(void) {
  for (int i = 1; i <= NCHANNELS; ++i) {
    Serial.print(F("ch"));
    Serial.print(i);
    Serial.print(F(":"));
    Serial.print(read_channel_percent(i));
    Serial.print(F("\t"));
  }
  Serial.print('\n');
}


void print_channels(bool as_percents = false) {
  for (int i = 1; i <= NCHANNELS; ++i) {
    Serial.print(F("ch"));
    Serial.print(i);
    Serial.print(F(":"));
    if (as_percents)
      Serial.print(read_channel_percent(i));
    else
      Serial.print(read_channel_time(i));
    Serial.print(F("\t"));
  }
  Serial.print('\n');
}

#define FRAME_SEPARATION 2500

void handle_tick()  {
  static int current_channel = 0;
  uint32_t tick = micros();
  uint32_t diff;
  if (tick > last_tick)
    // handle clock wraparound
    diff = 1 + tick + ~last_tick;
  else
    diff = last_tick - tick;
  last_tick = tick;

  if (diff > FRAME_SEPARATION) {
    current_channel = 0;
    return;
  }
  if (current_channel < NCHANNELS)
    channel_times[current_channel++] = diff;
}

void setup_ppm() {
  // start off with everything in a mid-position
  for (int i = 0; i < NCHANNELS; ++i)
      channel_times[i] = 1500;
  last_tick = micros();
  delay(10);
  pinMode(2, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(2), handle_tick, FALLING);
}
