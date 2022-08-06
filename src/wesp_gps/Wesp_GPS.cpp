#include <Wire.h>
#include "Wesp_GPS.h"
#include "utils/Wesp_Utils.h"

#define WAIT_TIME_FOR_GPS_POWER_SAVE_MODE 2000

Wesp_GPS_Class::Wesp_GPS_Class() {
    this->peripherialAvailable = false;
    this->fixed = false;
    this->fixType = 0;
    this->lon = 0.0;
    this->lat = 0.0;
    this->alt = 0.0;
    this->siv = 0xff;
    this->year = 2020;
    this->month = 12;
    this->day = 7;
    this->hour = 23;
    this->min = 27;
    this->sec = 30;
    
    sprintf(this->currentCoordinates, "n/a");
    sprintf(this->datetimestamp, "n/a");
}

void Wesp_GPS_Class::init() {
    if (!WU_TakeI2cLock(pdMS_TO_TICKS(5000))) {
        log_w("Failed to get the I2C lock for 5 sec... something is probably wrong...");
    }
    log_d("Setting up the GPS peripherial...");
    if (this->gps.begin() == false) {
        log_w("GPS peripherial is unavailable.");
        this->peripherialAvailable = false;
    }
    else {
        log_i("GPS peripherial is available, configuring...");

        this->gps.setI2COutput(COM_TYPE_UBX);
        this->gps.saveConfiguration();

        this->peripherialAvailable = true;

        this->gps.powerSaveMode(false, 2000);

        log_i("Done configuring GPS peripheral.");
    }
    WU_ReleaseI2cLock();
}

bool Wesp_GPS_Class::isPeripherialAvailable() {
    return this->peripherialAvailable;
}

bool Wesp_GPS_Class::hasFix() {
    return this->fixed;
}

void Wesp_GPS_Class::logCurrentCoordinates() {
    if (this->fixed) {
        sprintf(this->currentCoordinates, "%f,%f,%f", this->lon, this->lat, this->alt);
    }

    sprintf(this->datetimestamp, "%d-%02d-%02d %02d:%02d:%02d", this->year, this->month, this->day, this->hour, this->min, this->sec);

    log_i("GPS: Fix: %d, %s - Lon: %.6f Lat: %.6f Alt: %.2f - SIV: %d", 
             this->fixType, this->datetimestamp, this->lon, this->lat, this->alt, this->siv);
}

boolean Wesp_GPS_Class::isFixTypeGpsFix(uint8_t fixType) {
    // 0=no, 3=3D, 4=GNSS+Deadreckoning
    return fixType > 0;
}

bool Wesp_GPS_Class::checkForFix() {
    if (this->peripherialAvailable) {
        WU_TakeI2cLock(pdMS_TO_TICKS(1000));
        // 0=no, 3=3D, 4=GNSS+Deadreckoning
        if (this->gps.getFixType() > 0) {
            if (this->fixed == false) {
                log_i("GPS fix acquired.");
            }
            this->fixed = true;
        }
        else {
            if (this->fixed) {
                log_w("GPS fix lost.");
            }
            this->fixed = false;
        }
        WU_ReleaseI2cLock();

        return this->fixed;
    }
    return false;
}

void Wesp_GPS_Class::updateFixAndCoordinates() {
    if (!this->peripherialAvailable) {
        return;
    }

    if (!WU_TakeI2cLock(pdMS_TO_TICKS(5000))) {
        log_w("Waited 5 sec for i2c lock to update gps fix, something is probably wrong...");
    }

    this->fixType = this->gps.getFixType(); // 0=no, 3=3D, 4=GNSS+Deadreckoning
    this->year = this->gps.getYear();
    this->month = this->gps.getMonth();
    this->day = this->gps.getDay();
    this->hour = this->gps.getHour();
    this->min = this->gps.getMinute();
    this->sec = this->gps.getSecond();

    if (checkForFix()) {
        this->lon = this->gps.getLongitude() / GPS_COORDS_DIVISOR;
        this->lat = this->gps.getLatitude() / GPS_COORDS_DIVISOR;
        this->alt = this->gps.getAltitude() / GPS_ALT_DIVISOR;
        this->siv = this->gps.getSIV();

        sprintf(this->currentCoordinates, "%f,%f,%f", this->lon, this->lat, this->alt);
    }

    WU_ReleaseI2cLock();

    this->logCurrentCoordinates();

    //Wesp_Ble.setGPSData(this->getCurrentCoordinates(), this->lat, this->lon, this->alt, this->getDatetimeStamp());
}

const char* Wesp_GPS_Class::getCurrentCoordinates() {
    return this->currentCoordinates;
}

float& Wesp_GPS_Class::getLon() {
    return this->lon;
}

float& Wesp_GPS_Class::getLat() {
    return this->lat;
}

float& Wesp_GPS_Class::getAlt() {
    return this->alt;
}

const char* Wesp_GPS_Class::getDatetimeStamp() {
    return this->datetimestamp;
}



GPS_Timestamp_t Wesp_GPS_Class::updateAndFetchCurrentTime() {
    GPS_Timestamp_t ts = {
        1,
        0,
        0,
        1,
        1,
        1970 
    };

    if (!this->isPeripherialAvailable()) {
        return ts;
    }

    updateFixAndCoordinates();

    ts.second = this->sec;
    ts.minute = this->min;
    ts.hour = this->hour;
    ts.day = this->day;
    ts.month = this->month;
    ts.year = this->year;

    return ts;
}

bool Wesp_GPS_Class::enablePowerSaveMode() {
    if (!this->peripherialAvailable) {
        return false;
    }
    log_i("GPS going into powersave mode.");
    WU_TakeI2cLock(pdMS_TO_TICKS(1000));
    bool powerSaveResult = this->gps.powerSaveMode(true, WAIT_TIME_FOR_GPS_POWER_SAVE_MODE);
    WU_ReleaseI2cLock();
    return powerSaveResult;
}

bool Wesp_GPS_Class::waitForFix(uint32_t timeToWaitMs) {
    if (!this->peripherialAvailable) {
        return false;
    }
    uint32_t currentTimeMs = pdTICKS_TO_MS(xTaskGetTickCount());
    uint32_t endTimeMs = currentTimeMs + timeToWaitMs;
    uint8_t numSats = 0;
    bool gpsHasFix = false;
    bool gpsHasGoodFix = false;
    log_i("Waiting up to %f sec for the GPS module to acquire a fix with sufficient Sattelites.", timeToWaitMs / 1000.0);
    while(currentTimeMs < endTimeMs) {
        log_d("Checking gps fix...");
        this->updateFixAndCoordinates();

        if (this->hasFix()) {
            gpsHasFix = true;
            log_i("GPS has a fix with number of satellites: %d", this->siv);

            if (this->siv >= 4 && this->siv < 255) {
                log_i("GPS has a good fix.");
                gpsHasGoodFix = true;
                break;
            }
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
        currentTimeMs = pdTICKS_TO_MS(xTaskGetTickCount());
    }

    if (!gpsHasGoodFix) {
        log_w("GPS has a fix, but its not that great (sats <= 4) : %d", numSats);
    }

    return gpsHasFix;
}

Wesp_GPS_Class Wesp_GPS;