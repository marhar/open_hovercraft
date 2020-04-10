#ifndef __ppm_h
#define __ppm_h

#define MIN_TIME 996
#define MAX_TIME 2016
#define NCHANNELS 8

extern unsigned long channel_times[NCHANNELS];

extern int read_channel_time(int channel_num);
extern int read_channel_percent(int channel_num);
extern int channel_percent(int i);
extern void print_channels(int stuff);
extern void print_percents(int stuff);
extern void setup_ppm();

#endif
