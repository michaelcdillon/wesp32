#ifndef WESP_SENSORS_H
#define WESP_SENSORS_H

#include <stdio.h>
#include <stdlib.h>
#include <driver/adc.h>
#include <driver/gpio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "SparkFunBME280.h"
#include "ArduinoJson.h"
#include "wesp_gps/Wesp_GPS.h"

#define SOIL_MST_PIN        (GPIO_NUM_34)     // MM A0 = ESP32 GPI34/ADC1:6
#define SOIL_PWR_PIN        (GPIO_NUM_15)     // MM G0 = ESP32 GPIO15/ADC2:3
#define SOLAR_CHARGE_PIN    SOIL_MST_PIN
#define SOLAR_FAULT_PIN     SOIL_PWR_PIN
#define WIND_DIR_PIN        (ADC1_CHANNEL_7)  // MM A1 = ESP32 GPI35/ADC1:7
#define WIND_SPD_PIN        (GPIO_NUM_14)     // MM D0 = ESP32 GPIO14
#define RAIN_PIN            (GPIO_NUM_27)     // MM D1 = ESP32 GPIO27
#define LIGHTNING_ITR_PIN   (GPIO_NUM_17)     // MM G3 = ESP32 GPIO17
#define VIN_BATT_PIN        (ADC1_CHANNEL_3)  // MM BATT_VIN/3 = ESP32 GPI39/ADC1:3
#define VIN_BATT_MULTIPLIER 3

enum Wind_Dir_Nice {
    NORTH,
    NORTH_NORTH_EAST,
    NORTH_EAST,
    EAST_NORTH_EAST,
    EAST,
    EAST_SOUTH_EAST,
    SOUTH_EAST,
    SOUTH_SOUTH_EAST,
    SOUTH,
    SOUTH_SOUTH_WEST,
    SOUTH_WEST,
    WEST_SOUTH_WEST,
    WEST,
    WEST_NORTH_WEST,
    NORTH_WEST,
    NORTH_NORTH_WEST
};

enum Solar_Status {
    OK_NOT_CHARGING,
    BAD_BATTERY_FAULT,
    OK_CHARGING,
    OK_CHARGING_PAUSED_TEMP,
    UNKNOWN
};

typedef struct Wind_Dir {
    float degree;
    Wind_Dir_Nice wind_dir_nice;
    float dir_mv_min;
    float dir_mv_max;
} Wind_Dir_t;

extern const char* WindDirNiceToStr(Wind_Dir_Nice dir);
extern void MonitorWeatherSensorInterruptsTask(void* arg);
extern void UpdateWeatherDataAndReportTask(void* arg);

class Wesp_Sensors_Class {
    private:
        BME280* atmosphericSensor;
        bool wind_spd_last_state;
        TickType_t wind_spd_debounce_timeout;
        TickType_t wind_spd_ts_debounced;
        bool rain_last_state;
        TickType_t rain_debounce_timeout;
        TickType_t rain_ts_debounced;
        uint32_t wind_dir_mv;
        float batt_vin_v;
        bool solar_charge;
        bool solar_fault;
        Solar_Status solar_status;
        void setSolarStatus();
        uint32_t get_raw_wind_dir();
        uint32_t get_wind_dir_mv();
        float get_batt_vin_v();
        volatile Wind_Dir_t* convert_wind_dir_from_mv(uint32_t wind_dir_mv);
        volatile Wind_Dir_t* last_wind_dir;
        uint8_t cur_sec;
        uint8_t cur_2m_sec;
        uint8_t cur_min;
        uint8_t cur_hour;
        uint8_t day;
        uint8_t month;
        uint16_t year;
        float wind_speed_avg_120s_samples[120];
        float wind_dir_avg_120s_samples[120];
        float wind_gust_10m_samples[10];
        float wind_gust_dir_10m_samples[10];
        float rain_accum_60m_samples[60];
        float rain_accum_day;
        float rain_accum_hour;
        float wind_dir_instantaneous;                       // 0-360 instantaneous wind direction 
        volatile float wind_speed_kmph_instantaneous;             // kmph instantaneous wind speed
        float wind_gust_kmph_day;             // kmph current wind gust, using software specific time period
        float wind_gust_dir_day;              // 0-360 using software specific time period
        float wind_speed_kmph_2m_avg;               // kmph 2 minute average wind speed mph
        float wind_dir_2m_avg;                      // 0-360 2 minute average wind direction
        float wind_gust_kmph_10m_top;             // kmph past 10 minutes wind gust mph
        float wind_gust_dir_10m_top;              // 0-360 past 10 minutes wind gust direction
        float humidity;                             // % relative humidity
        float temp_c;                               // temp in deg c
        float pressure;                             // raw pressure in Pa
        float barometric_pressure_in;               // inches of mecury at current altitude
        float dew_point_c;                          // dew point in deg c
        char timestampStr[20];
        uint32_t reportWeatherDataDelay;
        bool wakeAtmosphericWaitForMeasurement();
        void resetCountersForNewDay();

    public:
        Wesp_Sensors_Class();
        void init(GPS_Timestamp_t ts);
        volatile Wind_Dir_t* getWindDir();
        float get_wind_speed_kmph();
        void stopSensorInterrupts();
        void startSensorInterrupts();
        void debounceWindSpdItrEvent(bool level, TickType_t tickStamp);
        void windSpdEventWaitTimeout();
        void debounceRainItrEvent(bool level, TickType_t tickStamp);
        void handleLightningItr(bool level, TickType_t tickStamp);
        void handleSolarChargeItr(bool level, TickType_t tickStamp);
        void handleSolarFaultItr(bool level, TickType_t tickStamp);
        float getTempC();
        float getTempF();
        float getHumidity();
        float getPressure();
        float getDewPoiont();
        void updateWeatherDataTaskLoop();
        void updateWindData();
        void calcWeatherData();
        void reportWeatherData();
};

extern Wesp_Sensors_Class Wesp_Sensors;

#endif // WESP_SENSORS_H
