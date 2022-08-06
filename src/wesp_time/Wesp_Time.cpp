#include "Wesp_Time.h"

void GetCurrentDateTime(Wesp_Time* datetime) {
    if (Wesp_GPS.isPeripherialAvailable()) {
        GPS_Timestamp_t ts = Wesp_GPS.updateAndFetchCurrentTime();

        datetime->second = ts.second;
        datetime->minute = ts.minute;
        datetime->hour = ts.hour;
        datetime->day = ts.day;
        datetime->month = ts.month;
        datetime->year = ts.year;
    }
    else {
        // from internal rtc, synced via ntp hopefully
        time_t now;
        struct tm timeinfo;

        time(&now);
        localtime_r(&now, &timeinfo);

        datetime->second = timeinfo.tm_sec;
        datetime->minute = timeinfo.tm_min;
        datetime->hour = timeinfo.tm_hour;
        datetime->day = timeinfo.tm_mday;
        datetime->month = timeinfo.tm_mon + 1;
        datetime->year = timeinfo.tm_year + 1900;
    }
}