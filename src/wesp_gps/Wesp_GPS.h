#ifndef Wesp_GPS_H
#define Wesp_GPS_H

#include "freertos/FreeRTOS.h"
#include <SparkFun_Ublox_Arduino_Library.h>

#define GPS_COORDS_DIVISOR 10000000.0
#define GPS_ALT_DIVISOR 1000.0

typedef struct GPS_Timestmap {
    uint8_t second;
    uint8_t minute;
    uint8_t hour;
    uint8_t day;
    uint8_t month;
    uint16_t year;
} GPS_Timestamp_t;

class Wesp_GPS_Class {
    private:
        SFE_UBLOX_GPS gps; 
        bool peripherialAvailable;
        bool fixed;
        uint8_t fixType;
        float lon;
        float lat;
        float alt;
        byte siv;
        uint16_t year;
        uint8_t month;
        uint8_t day;
        uint8_t hour;
        uint8_t min;
        uint8_t sec;
        char datetimestamp[20];
        char currentCoordinates[30]; 
        boolean isFixTypeGpsFix(uint8_t fixType);
        bool checkForFix();

    public:
        Wesp_GPS_Class();
        void init();
        bool isPeripherialAvailable();
        bool hasFix();
        const char* getCurrentCoordinates();
        float& getLon();
        float& getLat();
        float& getAlt();
        const char* getDatetimeStamp();
        void logCurrentCoordinates();
        void updateFixAndCoordinates();
        GPS_Timestamp_t updateAndFetchCurrentTime(); 
        bool enablePowerSaveMode();
        bool waitForFix(uint32_t timeToWaitMs);

};

extern Wesp_GPS_Class Wesp_GPS;

#endif // Wesp_GPS_H