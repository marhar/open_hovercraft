#ifndef __ppm_h
#define __ppm_h

#define MIN_TIME 996
#define MAX_TIME 2016
#define NCHANNELS 8

extern int read_channel_time(int channel_num);
extern int read_channel_percent(int channel_num);
void print_channels(bool as_percents = false);
extern void setup_ppm();

#endif
