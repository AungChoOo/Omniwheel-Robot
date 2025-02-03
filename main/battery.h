#ifndef _BATTERY_H_
#define _BATTERY_H_

#include <stdio.h>

#define CE_pin              1

#define RefVoltage          3.3
#define MaxVoltage          4.2
#define Calibration         0.64

struct Battery{
    float voltage_level, volts_perc;
};

extern struct Battery batt;

void battery_start(void);
float mapfloat(float val, float in_min, float in_max, float out_min, float out_max);

#endif /* _BATTERY_H_ */