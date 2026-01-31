#ifndef MEMS_INS_GPS_CONSTANTS_H
#define MEMS_INS_GPS_CONSTANTS_H
#include <math.h>

constexpr double Rad2Deg = 180.0 / M_PI;
constexpr double Deg2Rad = M_PI / 180.0;
constexpr double MinTimeDelta = 0.02;
constexpr double Hour2Sec = 3600.0;
constexpr double TIME_ALIGN_ERR = 0.001;

#endif //MEMS_INS_GPS_CONSTANTS_H