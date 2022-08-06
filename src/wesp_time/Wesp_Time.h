#ifndef WESP_TIME_H
#define WESP_TIME_H

#include <stdlib.h>
#include "wesp_gps/Wesp_GPS.h"

typedef struct Wesp_Time {
    uint8_t second;
    uint8_t minute;
    uint8_t hour;
    uint8_t day;
    uint8_t month;
    uint16_t year;
} Wesp_Time_t;

void GetCurrentDateTime(Wesp_Time* datetime);

#endif // WESP_TIME_H