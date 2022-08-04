#include "Wesp_Sensors.h"
#include "wesp_gps/Wesp_GPS.h"
#include "wesp_mqtt/Wesp_MQTT.h"
#include "wesp_config/Wesp_Config.h"
#include "utils/Wesp_Utils.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include <esp32-hal-log.h>
#include <esp_adc_cal.h>
#include <driver/adc.h>
#include <driver/gpio.h>
#include "ArduinoJson.h"

#define NUM_WESP_JSON_DATA_MEMBERS 20

// debounce wind speed time: 10ms. this gives enough time for 100 wind
// sensor events per second, which equates to 149.2 mph
#define WIND_SPD_1_RPS_KPH 2.4 // 1 revolution per second is 2.4 KM/h 
#define WIND_SPD_1_RPS_MPH 1.492 // 1 revolution per second is 1.492 KM/h
#define RAIN_EVENT_INCREMENT_IN 0.011 // 1 bucket dump in the rain sensor is 0.011 in of precip
#define RAIN_EVENT_INCREMENT_MM 0.2794 // 1 bucket dump in the rain sensor is 0.2794mm of precip
#define WIND_SPD_DEBOUNCE_MS 10
#define RAIN_DEBOUNCE_MS 50
#define WEATHER_DATA_UPDATE_PERIOD_MS 100
#define ATMOSPHERIC_SENSOR_WAIT_UNTIL_MEASSURING_START_P_MS  1 
#define ATMOSPHERIC_SENSOR_WAIT_UNTIL_MEASSURING_FINISH_P_MS  3
#define REPORT_TIME_INTERVAL_SLOW_MS 60000
#define REPORT_TIME_INTERVAL_FAST_MS 5000
#define CALC_WX_DATA_INTERVAL_MS 1000
#define WIND_SPEED_FAST_REPORTING_THRESHOLD_KMPH 20.0

#define GPIO_INPUT_PIN_SEL  ((1ULL<<WIND_SPD_PIN) | (1ULL<<RAIN_PIN) | (1ULL<<LIGHTNING_ITR_PIN) | (1ULL<<VIN_BATT_PIN) | (1ULL<<SOLAR_CHARGE_PIN) | (1ULL<<SOLAR_FAULT_PIN))
#define GPIO_OUTPUT_PIN_SEL 0

#define NUM_BATT_VIN_ADC_SAMPLES 5
#define NUM_WIND_DIR_ADC_SAMPLES 5
#define WIND_DIR_R1 10000
#define WIND_DIR_SV 3.3
#define NUM_WIND_DIRS 16
#define WIND_DIR_DEG_0_R 33000
#define WIND_DIR_DEG_22_5_R 6570
#define WIND_DIR_DEG_45_R 8200 
#define WIND_DIR_DEG_67_5_R 891 
#define WIND_DIR_DEG_90_R 1000 
#define WIND_DIR_DEG_112_5_R 688 
#define WIND_DIR_DEG_135_R 2200 
#define WIND_DIR_DEG_157_5_R 1410 
#define WIND_DIR_DEG_180_R 3900 
#define WIND_DIR_DEG_202_5_R 3140 
#define WIND_DIR_DEG_225_R 16000 
#define WIND_DIR_DEG_247_5_R 14120 
#define WIND_DIR_DEG_270_R 120000 
#define WIND_DIR_DEG_292_5_R 42120 
#define WIND_DIR_DEG_315_R 64900 
#define WIND_DIR_DEG_337_5_R 21880 

#define DEG_R_RESISTOR_TOLERANCE 0.06
#define DEG_R_RESISTOR_TOLERANCE_MED 0.1
#define DEG_R_RESISTOR_TOLERANCE_LRG 0.16
#define DEG_R_RESISTOR_TOLERANCE_XL 0.2
#define CALC_WIND_DIR_DEG_MV_MIN(DEG_R, TOL_R) (((WIND_DIR_SV * (DEG_R - (DEG_R * TOL_R)))/(WIND_DIR_R1 + (DEG_R - (DEG_R * TOL_R)))) * 1000.0)
#define CALC_WIND_DIR_DEG_MV_MAX(DEG_R, TOL_R) (((WIND_DIR_SV * (DEG_R + (DEG_R * TOL_R)))/(WIND_DIR_R1 + (DEG_R + (DEG_R * TOL_R)))) * 1000.0)

#define WIND_DIR_DEG_0_MV_MIN (CALC_WIND_DIR_DEG_MV_MIN(WIND_DIR_DEG_0_R, DEG_R_RESISTOR_TOLERANCE))
#define WIND_DIR_DEG_0_MV_MAX (CALC_WIND_DIR_DEG_MV_MAX(WIND_DIR_DEG_0_R, DEG_R_RESISTOR_TOLERANCE))
#define WIND_DIR_DEG_22_5_MV_MIN (CALC_WIND_DIR_DEG_MV_MIN(WIND_DIR_DEG_22_5_R, DEG_R_RESISTOR_TOLERANCE))
#define WIND_DIR_DEG_22_5_MV_MAX (CALC_WIND_DIR_DEG_MV_MAX(WIND_DIR_DEG_22_5_R, DEG_R_RESISTOR_TOLERANCE))
#define WIND_DIR_DEG_45_MV_MIN (CALC_WIND_DIR_DEG_MV_MIN(WIND_DIR_DEG_45_R, DEG_R_RESISTOR_TOLERANCE))
#define WIND_DIR_DEG_45_MV_MAX (CALC_WIND_DIR_DEG_MV_MAX(WIND_DIR_DEG_45_R, DEG_R_RESISTOR_TOLERANCE))
#define WIND_DIR_DEG_67_5_MV_MIN (CALC_WIND_DIR_DEG_MV_MIN(WIND_DIR_DEG_67_5_R, DEG_R_RESISTOR_TOLERANCE))
#define WIND_DIR_DEG_67_5_MV_MAX (CALC_WIND_DIR_DEG_MV_MAX(WIND_DIR_DEG_67_5_R, DEG_R_RESISTOR_TOLERANCE))
#define WIND_DIR_DEG_90_MV_MIN (CALC_WIND_DIR_DEG_MV_MIN(WIND_DIR_DEG_90_R, DEG_R_RESISTOR_TOLERANCE))
#define WIND_DIR_DEG_90_MV_MAX (CALC_WIND_DIR_DEG_MV_MAX(WIND_DIR_DEG_90_R, DEG_R_RESISTOR_TOLERANCE_LRG))
#define WIND_DIR_DEG_112_5_MV_MIN (CALC_WIND_DIR_DEG_MV_MIN(WIND_DIR_DEG_112_5_R, DEG_R_RESISTOR_TOLERANCE))
#define WIND_DIR_DEG_112_5_MV_MAX (CALC_WIND_DIR_DEG_MV_MAX(WIND_DIR_DEG_112_5_R, DEG_R_RESISTOR_TOLERANCE_LRG))
#define WIND_DIR_DEG_135_MV_MIN (CALC_WIND_DIR_DEG_MV_MIN(WIND_DIR_DEG_135_R, DEG_R_RESISTOR_TOLERANCE))
#define WIND_DIR_DEG_135_MV_MAX (CALC_WIND_DIR_DEG_MV_MAX(WIND_DIR_DEG_135_R, DEG_R_RESISTOR_TOLERANCE))
#define WIND_DIR_DEG_157_5_MV_MIN (CALC_WIND_DIR_DEG_MV_MIN(WIND_DIR_DEG_157_5_R, DEG_R_RESISTOR_TOLERANCE))
#define WIND_DIR_DEG_157_5_MV_MAX (CALC_WIND_DIR_DEG_MV_MAX(WIND_DIR_DEG_157_5_R, DEG_R_RESISTOR_TOLERANCE_LRG))
#define WIND_DIR_DEG_180_MV_MIN (CALC_WIND_DIR_DEG_MV_MIN(WIND_DIR_DEG_180_R, DEG_R_RESISTOR_TOLERANCE))
#define WIND_DIR_DEG_180_MV_MAX (CALC_WIND_DIR_DEG_MV_MAX(WIND_DIR_DEG_180_R, DEG_R_RESISTOR_TOLERANCE))
#define WIND_DIR_DEG_202_5_MV_MIN (CALC_WIND_DIR_DEG_MV_MIN(WIND_DIR_DEG_202_5_R, DEG_R_RESISTOR_TOLERANCE))
#define WIND_DIR_DEG_202_5_MV_MAX (CALC_WIND_DIR_DEG_MV_MAX(WIND_DIR_DEG_202_5_R, DEG_R_RESISTOR_TOLERANCE))
#define WIND_DIR_DEG_225_MV_MIN (CALC_WIND_DIR_DEG_MV_MIN(WIND_DIR_DEG_225_R, DEG_R_RESISTOR_TOLERANCE))
#define WIND_DIR_DEG_225_MV_MAX (CALC_WIND_DIR_DEG_MV_MAX(WIND_DIR_DEG_225_R, DEG_R_RESISTOR_TOLERANCE))
#define WIND_DIR_DEG_247_5_MV_MIN (CALC_WIND_DIR_DEG_MV_MIN(WIND_DIR_DEG_247_5_R, DEG_R_RESISTOR_TOLERANCE))
#define WIND_DIR_DEG_247_5_MV_MAX (CALC_WIND_DIR_DEG_MV_MAX(WIND_DIR_DEG_247_5_R, DEG_R_RESISTOR_TOLERANCE))
#define WIND_DIR_DEG_270_MV_MIN (CALC_WIND_DIR_DEG_MV_MIN(WIND_DIR_DEG_270_R, DEG_R_RESISTOR_TOLERANCE))
#define WIND_DIR_DEG_270_MV_MAX (CALC_WIND_DIR_DEG_MV_MAX(WIND_DIR_DEG_270_R, DEG_R_RESISTOR_TOLERANCE_XL))
#define WIND_DIR_DEG_292_5_MV_MIN (CALC_WIND_DIR_DEG_MV_MIN(WIND_DIR_DEG_292_5_R, DEG_R_RESISTOR_TOLERANCE))
#define WIND_DIR_DEG_292_5_MV_MAX (CALC_WIND_DIR_DEG_MV_MAX(WIND_DIR_DEG_292_5_R, DEG_R_RESISTOR_TOLERANCE))
#define WIND_DIR_DEG_315_MV_MIN (CALC_WIND_DIR_DEG_MV_MIN(WIND_DIR_DEG_315_R, DEG_R_RESISTOR_TOLERANCE))
#define WIND_DIR_DEG_315_MV_MAX (CALC_WIND_DIR_DEG_MV_MAX(WIND_DIR_DEG_315_R, DEG_R_RESISTOR_TOLERANCE_LRG))
#define WIND_DIR_DEG_337_5_MV_MIN (CALC_WIND_DIR_DEG_MV_MIN(WIND_DIR_DEG_337_5_R, DEG_R_RESISTOR_TOLERANCE))
#define WIND_DIR_DEG_337_5_MV_MAX (CALC_WIND_DIR_DEG_MV_MAX(WIND_DIR_DEG_337_5_R, DEG_R_RESISTOR_TOLERANCE))

#define LIGHTNING_MASK_DISTURBERS true
#define LIGHTNING_NOISE_FLOOR 2
#define LIGHTNING_WATCH_DOG_VAL 2
#define LIGHTNING_SPIKE 2
#define LIGHTNING_THRESHOLD 1

static esp_adc_cal_characteristics_t adc1_chars;

SparkFun_AS3935 lightningSensor;

Wind_Dir_t Wind_Dirs[NUM_WIND_DIRS] = {
    {
        0.0,
        Wind_Dir_Nice::NORTH,
        WIND_DIR_DEG_0_MV_MIN,
        WIND_DIR_DEG_0_MV_MAX
    },
    {
        22.5,
        Wind_Dir_Nice::NORTH_NORTH_EAST,
        WIND_DIR_DEG_22_5_MV_MIN,
        WIND_DIR_DEG_22_5_MV_MAX
    },
    {
        45.0,
        Wind_Dir_Nice::NORTH_EAST,
        WIND_DIR_DEG_45_MV_MIN,
        WIND_DIR_DEG_45_MV_MAX
    },
    {
        67.5,
        Wind_Dir_Nice::EAST_NORTH_EAST,
        WIND_DIR_DEG_67_5_MV_MIN,
        WIND_DIR_DEG_67_5_MV_MAX
    },
    {
        90.0,
        Wind_Dir_Nice::EAST,
        WIND_DIR_DEG_90_MV_MIN,
        WIND_DIR_DEG_90_MV_MAX
    },
    {
        112.5,
        Wind_Dir_Nice::EAST_SOUTH_EAST,
        WIND_DIR_DEG_112_5_MV_MIN,
        WIND_DIR_DEG_112_5_MV_MAX
    },
    {
        135.0,
        Wind_Dir_Nice::SOUTH_EAST,
        WIND_DIR_DEG_135_MV_MIN,
        WIND_DIR_DEG_135_MV_MAX
    },
    {
        157.5,
        Wind_Dir_Nice::SOUTH_SOUTH_EAST,
        WIND_DIR_DEG_157_5_MV_MIN,
        WIND_DIR_DEG_157_5_MV_MAX
    },
    {
        180.0,
        Wind_Dir_Nice::SOUTH,
        WIND_DIR_DEG_180_MV_MIN,
        WIND_DIR_DEG_180_MV_MAX
    },
    {
        202.5,
        Wind_Dir_Nice::SOUTH_SOUTH_WEST,
        WIND_DIR_DEG_202_5_MV_MIN,
        WIND_DIR_DEG_202_5_MV_MAX
    },
    {
        225.0,
        Wind_Dir_Nice::SOUTH_WEST,
        WIND_DIR_DEG_225_MV_MIN,
        WIND_DIR_DEG_225_MV_MAX
    },
    {
        247.5,
        Wind_Dir_Nice::WEST_SOUTH_WEST,
        WIND_DIR_DEG_247_5_MV_MIN,
        WIND_DIR_DEG_247_5_MV_MAX
    },
    {
        270.0,
        Wind_Dir_Nice::WEST,
        WIND_DIR_DEG_270_MV_MIN,
        WIND_DIR_DEG_270_MV_MAX
    },
    {
        292.5,
        Wind_Dir_Nice::WEST_NORTH_WEST,
        WIND_DIR_DEG_292_5_MV_MIN,
        WIND_DIR_DEG_292_5_MV_MAX
    },
    {
        315.0,
        Wind_Dir_Nice::NORTH_WEST,
        WIND_DIR_DEG_315_MV_MIN,
        WIND_DIR_DEG_315_MV_MAX
    },
    {
        337.5,
        Wind_Dir_Nice::NORTH_NORTH_WEST,
        WIND_DIR_DEG_337_5_MV_MIN,
        WIND_DIR_DEG_337_5_MV_MAX
    }
};

const char* WindDirNiceToStr(Wind_Dir_Nice dir) {
    switch(dir) {
        case NORTH:
            return "N";
        case NORTH_NORTH_EAST:
            return "NNE";
        case NORTH_EAST:
            return "NE";
        case EAST_NORTH_EAST:
            return "ENE";
        case EAST:
            return "E";
        case EAST_SOUTH_EAST:
            return "ESE";
        case SOUTH_EAST:
            return "SE";
        case SOUTH_SOUTH_EAST:
            return "SSE";
        case SOUTH:
            return "S";
        case SOUTH_SOUTH_WEST:
            return "SSW";
        case SOUTH_WEST:
            return "SW";
        case WEST_SOUTH_WEST:
            return "WSW";
        case WEST:
            return "W";
        case WEST_NORTH_WEST:
            return "WNW";
        case NORTH_WEST:
            return "NW";
        case NORTH_NORTH_WEST:
            return "NNW";
        default:
            return "N";
    }
}

const char* SolarStatusToNiceStr(Solar_Status solar_Status) {
    switch(solar_Status) {
        case BAD_BATTERY_FAULT:
            return "B_BAT_F";
        case OK_CHARGING:
            return "OK_C";
        case OK_CHARGING_PAUSED_TEMP:
            return "OK_C_P_T";
        case OK_NOT_CHARGING:
            return "OK_NC";
        case UNKNOWN:
        default:
            return "UNK";
    }
}

static xQueueHandle sensorEventQueue = NULL;

typedef struct SensorEvent {
    TickType_t tickStamp;
    uint32_t gpioPinNum;
    bool level;
} SensorEvent_t; 

static void IRAM_ATTR handleWeatherSensorInterrupts(void* arg) {
    SensorEvent_t event = {
        xTaskGetTickCountFromISR(),
        (uint32_t) arg,
        (gpio_get_level((gpio_num_t) (uint32_t) arg) == 1)
    };

    xQueueSendFromISR(sensorEventQueue, &event, NULL);
}

void MonitorWeatherSensorInterruptsTask(void* arg) {
    SensorEvent_t sensorEvent;
    for(;;) {
        if (xQueueReceive(sensorEventQueue, &sensorEvent, portMAX_DELAY)) {
            switch(sensorEvent.gpioPinNum) {
                case WIND_SPD_PIN:
                    Wesp_Sensors.debounceWindSpdItrEvent(sensorEvent.level, sensorEvent.tickStamp);
                    break;
                case RAIN_PIN:
                    Wesp_Sensors.debounceRainItrEvent(sensorEvent.level, sensorEvent.tickStamp);
                    break;
                case LIGHTNING_ITR_PIN:
                    Wesp_Sensors.handleLightningItr(sensorEvent.level, sensorEvent.tickStamp);
                    break;
                case SOLAR_CHARGE_PIN:
                    Wesp_Sensors.handleSolarChargeItr(sensorEvent.level, sensorEvent.tickStamp);
                    break;
                case SOLAR_FAULT_PIN:
                    Wesp_Sensors.handleSolarFaultItr(sensorEvent.level, sensorEvent.tickStamp);
                    break;    
                default:
                    log_w("Unknown interrupt on gpio: %d - edge: %b", sensorEvent.level, sensorEvent.gpioPinNum);
            }
        } else {
            // we didn't get a sensor event on the queue, lets zero out current wind
            Wesp_Sensors.windSpdEventWaitTimeout();
        }
    }

    vTaskDelete(NULL);
}

void UpdateWeatherDataAndReportTask(void* arg) {
    log_i("Starting update weather data task.");

    Wesp_Sensors.updateWeatherDataTaskLoop();
    
    vTaskDelete(NULL);
}

Wesp_Sensors_Class::Wesp_Sensors_Class() {
    this->atmosphericSensor = new BME280();
    this->wind_spd_last_state = false;
    this->wind_spd_debounce_timeout = 0;
    this->wind_spd_ts_debounced = 0;
    this->rain_last_state = false;
    this->rain_debounce_timeout = 0;
    this->rain_ts_debounced = 0;
    this->solar_charge = false;
    this->solar_fault = false;
    this->reportWeatherDataDelay = REPORT_TIME_INTERVAL_FAST_MS;  // fast to start so reporting is immediate
    
    this->resetCountersForNewDay();
}

void Wesp_Sensors_Class::resetCountersForNewDay() {
    this->rain_accum_day = 0.0;
    this->rain_accum_hour = 0.0;
    this->last_wind_dir = &Wind_Dirs[0];
    this->wind_speed_kmph_instantaneous = 0.0;                      
    this->wind_gust_kmph_day = 0.0;
    this->wind_gust_dir_day = Wind_Dirs[0].degree;
    this->wind_speed_kmph_2m_avg = 0.0;
    this->wind_dir_2m_avg = 0.0;
    this->wind_gust_kmph_10m_top = 0.0;
    this->wind_gust_dir_10m_top = Wind_Dirs[0].degree;
    this->humidity = 0.0;
    this->temp_c = 0.0;
    this->pressure = 0.0;
    this->barometric_pressure_in = 0.0;
    this->dew_point_c = 0.0;

    uint8_t i;
    for (i = 0; i < 120; i++ ) {
        this->wind_speed_avg_120s_samples[i] = 0.0;
        this->wind_dir_avg_120s_samples[i] = 0.0;

    }
    for (i = 0; i < 10; i++) {
        this->wind_gust_10m_samples[i] = 0.0;
        this->wind_gust_dir_10m_samples[i] = 0.0;
    }

    for (i = 0; i < 60; i++) { 
        this->rain_accum_60m_samples[i] = 0.0;
    }
}

void Wesp_Sensors_Class::init(GPS_Timestamp_t ts) {
    log_d("Wind dir settings:");
    for (uint8_t i = 0; i < NUM_WIND_DIRS; i++) {
        Wind_Dir_t* dir = &Wind_Dirs[i];
        log_d("%s (%f) - MIN: %f MAX: %f", 
            WindDirNiceToStr(dir->wind_dir_nice),
            dir->degree, 
            dir->dir_mv_min,
            dir->dir_mv_max
        );
    }

    log_i("Initializing weather sensors...");
    // init adc for wind direction
    //Characterize ADC at particular atten
    esp_adc_cal_value_t val_type = esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_11db, ADC_WIDTH_BIT_12, 0, &adc1_chars);
    //Check type of calibration value used to characterize ADC
    if (val_type == ESP_ADC_CAL_VAL_EFUSE_VREF) {
        log_i("Using esp32 efuse: vref for adc1 calibration");
    } else if (val_type == ESP_ADC_CAL_VAL_EFUSE_TP) {
        log_i("using two point method for adc1 calibration.");
    } else {
        log_i("Using default method for adc1 calibration.");
    }
    ESP_ERROR_CHECK(adc1_config_width(ADC_WIDTH_BIT_12));
    ESP_ERROR_CHECK(adc1_config_channel_atten(WIND_DIR_PIN, ADC_ATTEN_11db));
    ESP_ERROR_CHECK(adc1_config_channel_atten(VIN_BATT_PIN, ADC_ATTEN_11db));

    this->setupLightningSensor();

    // setup gpio for wind speed, rain, and lightning sensors

    log_d("setting gpio outputs.");
    //zero-initialize the config structure.
    gpio_config_t io_conf = {};
    //disable interrupt
    io_conf.intr_type = GPIO_INTR_DISABLE;
    //set as output mode
    io_conf.mode = GPIO_MODE_OUTPUT;
    //bit mask of the pins that you want to set,e.g.GPIO18/19
    if (GPIO_OUTPUT_PIN_SEL != 0x00) {
        io_conf.pin_bit_mask = GPIO_OUTPUT_PIN_SEL;
    }
    //disable pull-down mode
    io_conf.pull_down_en = gpio_pulldown_t::GPIO_PULLDOWN_DISABLE;
    //disable pull-up mode
    io_conf.pull_up_en = gpio_pullup_t::GPIO_PULLUP_DISABLE;
    //configure GPIO with the given settings
    gpio_config(&io_conf);

    log_d("setting gpio inputs");
    //interrupt of rising edge
    io_conf.intr_type = GPIO_INTR_ANYEDGE;
    //set as input mode
    io_conf.mode = GPIO_MODE_INPUT;
    //bit mask of pins you want to set
    io_conf.pin_bit_mask = GPIO_INPUT_PIN_SEL;
    //enable pull-up mode
    io_conf.pull_up_en = gpio_pullup_t::GPIO_PULLUP_ENABLE;
    gpio_config(&io_conf);

    // get initial values for solar state
    this->solar_status = Solar_Status::UNKNOWN;
    this->solar_fault = (gpio_get_level(SOLAR_FAULT_PIN) == 1);
    this->solar_charge = (gpio_get_level(SOLAR_CHARGE_PIN) == 1);
    this->setSolarStatus();

    //create a queue to handle gpio event from isr
    sensorEventQueue = xQueueCreate(50, sizeof(SensorEvent_t));

    // set the time
    this->year = ts.year;
    this->month = ts.month;
    this->day = ts.day;
    this->cur_hour = ts.hour;
    this->cur_min = ts.minute;
    this->cur_sec = ts.second;
    this->cur_2m_sec = this->cur_sec;

    //install gpio isr service
    gpio_install_isr_service(0); // ESP_INTR_FLAG_DEFAULT

    log_d("setting up the atmospheric sensor.");

    // atmospheric sensor setup
    WU_TakeI2cLock(pdMS_TO_TICKS(1000));
    this->atmosphericSensor->beginI2C();
    this->atmosphericSensor->setMode(MODE_SLEEP);
    WU_ReleaseI2cLock();
    log_i("Weather sensors initialiized.");
}

void Wesp_Sensors_Class::setupLightningSensor() {
    if (this->lightningSensorAvailable) {
        log_w("Lightning sensor was already setup.");
    }

    if (!lightningSensor.beginSPI(LIGHTNING_CS_PIN)) {
        log_e("Failed to setup the lightning sensor.");
    }
    log_i("Lightning sensor setup, configuring now...");

    lightningSensor.maskDisturber(LIGHTNING_MASK_DISTURBERS);
    lightningSensor.setIndoorOutdoor(OUTDOOR);
    lightningSensor.setNoiseLevel(LIGHTNING_NOISE_FLOOR);
    lightningSensor.watchdogThreshold(LIGHTNING_WATCH_DOG_VAL);
    lightningSensor.spikeRejection(LIGHTNING_SPIKE); 
    lightningSensor.lightningThreshold(LIGHTNING_THRESHOLD);

    log_i("Lightning sensor configured.");
}

void Wesp_Sensors_Class::startSensorInterrupts() {
    gpio_isr_handler_add(WIND_SPD_PIN, handleWeatherSensorInterrupts, (void*) WIND_SPD_PIN);
    gpio_isr_handler_add(RAIN_PIN, handleWeatherSensorInterrupts, (void*) RAIN_PIN);
    gpio_isr_handler_add(LIGHTNING_ITR_PIN, handleWeatherSensorInterrupts, (void*) LIGHTNING_ITR_PIN);
}

void Wesp_Sensors_Class::stopSensorInterrupts() {
    gpio_isr_handler_remove(WIND_SPD_PIN);
    gpio_isr_handler_remove(RAIN_PIN);
    gpio_isr_handler_remove(LIGHTNING_ITR_PIN);
}

uint32_t Wesp_Sensors_Class::get_raw_wind_dir() {
    uint32_t raw_wind_cumulative = 0;
    for (uint8_t i = 0; i < NUM_WIND_DIR_ADC_SAMPLES; i++) {
        raw_wind_cumulative += adc1_get_raw(WIND_DIR_PIN);
    }
    return (uint32_t) raw_wind_cumulative / NUM_WIND_DIR_ADC_SAMPLES;
}

uint32_t Wesp_Sensors_Class::get_wind_dir_mv() {
    int32_t raw_adc = this->get_raw_wind_dir();
    this->wind_dir_mv = esp_adc_cal_raw_to_voltage(raw_adc, &adc1_chars);
    return this->wind_dir_mv;
}

float Wesp_Sensors_Class::get_batt_vin_v() {
    int32_t raw_batt_vin_cumulative = 0;
    for (uint8_t i = 0; i < NUM_BATT_VIN_ADC_SAMPLES; i++) {
        raw_batt_vin_cumulative += adc1_get_raw(VIN_BATT_PIN);
    }
    uint32_t avg_batt_vin = (uint32_t) raw_batt_vin_cumulative / NUM_BATT_VIN_ADC_SAMPLES;
    this->batt_vin_v = (esp_adc_cal_raw_to_voltage(avg_batt_vin, &adc1_chars) * VIN_BATT_MULTIPLIER) / 1000.0;
    return this->batt_vin_v;
}

volatile Wind_Dir_t* Wesp_Sensors_Class::convert_wind_dir_from_mv(uint32_t wind_dir_mv) {
    //log_d("Converting wind mv to dir: %d", wind_dir_mv);
    for (uint8_t i = 0; i < NUM_WIND_DIRS; i++) {
        if (wind_dir_mv <= Wind_Dirs[i].dir_mv_max && wind_dir_mv >= Wind_Dirs[i].dir_mv_min) {
            this->last_wind_dir = &Wind_Dirs[i];
            return this->last_wind_dir;
        }
    }
    if (this->last_wind_dir == NULL) {
        this->last_wind_dir = &Wind_Dirs[0];
    }
    //log_w("Fell out of the conversion loop, returning last wind dir: %f", this->last_wind_dir->degree);
    return this->last_wind_dir;
}

volatile Wind_Dir* Wesp_Sensors_Class::getWindDir() {
    return this->convert_wind_dir_from_mv(
        this->get_wind_dir_mv()
    );
}

void Wesp_Sensors_Class::debounceRainItrEvent(bool level, TickType_t tickstamp) {
    if (this->rain_debounce_timeout == 0 && !this->rain_last_state && level) {
        // first rising edge start a new debounce cycle
        this->rain_last_state = level;
        this->rain_debounce_timeout = tickstamp + pdMS_TO_TICKS(RAIN_DEBOUNCE_MS);
        this->rain_ts_debounced = tickstamp;

        this->rain_accum_day += RAIN_EVENT_INCREMENT_MM;
        this->rain_accum_60m_samples[this->cur_min] += RAIN_EVENT_INCREMENT_MM;
    }
    else if (tickstamp < this->rain_debounce_timeout){
        // in a debounce cycle pretty much do nothing
    }
    else if (tickstamp >= this->rain_debounce_timeout && !level) {
        // after the debounce period and we got the falling edge.
        this->rain_last_state = level;
        this->rain_debounce_timeout = 0;
    }
    else {
        // if we get here it was a rising edge itr outside or 
        // equal to the end of the debounce period. we should warn and reset the states.
        this->rain_last_state = level;
        this->rain_debounce_timeout = 0;
        log_w("rain sensor itr (ts: %d) outside of debounce period (%d).", tickstamp, this->rain_debounce_timeout);
    }
}

void Wesp_Sensors_Class::debounceWindSpdItrEvent(bool level, TickType_t tickstamp) {
    if (this->wind_spd_debounce_timeout == 0 && !this->wind_spd_last_state && level) {
        //log_i("Wind speed event at %d - last: %d", tickstamp, this->wind_spd_ts_debounced);
        // first rising edge start a new debounce cycle
        this->wind_spd_last_state = level;
        this->wind_spd_debounce_timeout = tickstamp + pdMS_TO_TICKS(WIND_SPD_DEBOUNCE_MS);

        uint32_t t_since_last_wind_sensor_hit = this->wind_spd_ts_debounced != 0 ? (tickstamp - this->wind_spd_ts_debounced) : 0;
        //log_d("Wind speed ticks since last event: %d", t_since_last_wind_sensor_hit);
        if (t_since_last_wind_sensor_hit == 0 || t_since_last_wind_sensor_hit > 5000) {
            this->wind_speed_kmph_instantaneous = 0.0;
        } else {
            this->wind_speed_kmph_instantaneous = (1.0 / (pdTICKS_TO_MS(t_since_last_wind_sensor_hit) / 1000.0)) * WIND_SPD_1_RPS_KPH;
        }
        
        //log_d("Calc'd wind speed: %f", this->wind_speed_kmph);

        this->wind_spd_ts_debounced = tickstamp;
    }
    else if (tickstamp < this->wind_spd_debounce_timeout){
        // in a debounce cycle pretty much do nothing
        //log_d("in debounce wind skipping: %d - %d, until %d", level, tickstamp, this->wind_spd_debounce_timeout);
    }
    else if (tickstamp >= this->wind_spd_debounce_timeout && !level) {
        // after the debounce period and we got the falling edge.
        //log_d("at or after debounce window with falling edge - timeout ticks: %d - ticks: %d", this->wind_spd_debounce_timeout, tickstamp);
        this->wind_spd_last_state = level;
        this->wind_spd_debounce_timeout = 0;
    }
    else {
        // if we get here it was a rising edge itr outside or 
        // equal to the end of the debounce period. we should warn and reset the states.
        this->wind_spd_last_state = level;
        this->wind_spd_debounce_timeout = 0;
        log_w("rain sensor itr (ts: %d - level: %d) outside of debounce period (%d).", tickstamp, level, this->wind_spd_debounce_timeout);
    }
}

void Wesp_Sensors_Class::windSpdEventWaitTimeout() {
    this->wind_speed_kmph_instantaneous = 0.0;
    this->wind_spd_last_state = false;
    this->wind_spd_debounce_timeout = 0;
}

void Wesp_Sensors_Class::handleLightningItr(bool level, TickType_t tickstamp) {
    this->lastLightningEvent.type = (Lightning_Event_Type) lightningSensor.readInterruptReg();
    this->lastLightningEvent.distanceKm = lightningSensor.distanceToStorm();
    this->lastLightningEvent.energy = lightningSensor.lightningEnergy();
    if (this->lastLightningEvent.type == Lightning_Event_Type::LIGHTNING_E) {
        log_i("Lightning detected: %d km away with %d energy.", this->lastLightningEvent.distanceKm, this->lastLightningEvent.energy);
    }
    else if (this->lastLightningEvent.type == Lightning_Event_Type::DISTURBUER_E) {
        log_w("Lightning disturber detected.");
    }
    else {
        log_w("Lightning noise detected.");
    }

    this->newLightningEventToSend = true;
}

void Wesp_Sensors_Class::handleSolarChargeItr(bool level, TickType_t tickstamp) {
    this->solar_charge = level;
    this->setSolarStatus();
}

void Wesp_Sensors_Class::handleSolarFaultItr(bool level, TickType_t tickstamp) {
    this->solar_fault = level;
    this->setSolarStatus();
}

void Wesp_Sensors_Class::setSolarStatus() {
    if (!this->solar_charge && !this->solar_fault) {
        this->solar_status = Solar_Status::OK_NOT_CHARGING;
    } else if (!this->solar_charge && this->solar_fault) {
        this->solar_status = Solar_Status::BAD_BATTERY_FAULT;
    } else if (this->solar_charge && !this->solar_fault) {
        this->solar_status = Solar_Status::OK_CHARGING;
    } else if (this->solar_charge && this->solar_fault) {
        this->solar_status = Solar_Status::OK_CHARGING_PAUSED_TEMP;
    } else {
        this->solar_status = Solar_Status::UNKNOWN;
    }
}

float Wesp_Sensors_Class::get_wind_speed_kmph() {
    return this->wind_speed_kmph_instantaneous;
}

void Wesp_Sensors_Class::updateWeatherDataTaskLoop() {
    uint32_t curTimestamp;
    uint32_t lastSecondTimestamp = pdTICKS_TO_MS(xTaskGetTickCount());
    uint32_t last_report_timestamp = lastSecondTimestamp;
    uint32_t last_calc_wx_timestamp = lastSecondTimestamp;

    for (;;) {
        curTimestamp = pdTICKS_TO_MS(xTaskGetTickCount());
        //log_d("curTimestamp: %d last timestamp: %d", curTimestamp, lastSecondTimestamp);

        // wind and wind dir avgs get updated every WEATHER_DATA_UPDATE_PERIOD_MS 
        Wesp_Sensors.updateWindData();

        if (newLightningEventToSend) {
            this->sendLightningEvent(); 
            newLightningEventToSend = false;
        }

        // look for a second to have passed
        if (curTimestamp - lastSecondTimestamp >= 1000) {
            this->cur_sec++;
            this->cur_2m_sec++;

            if (this->cur_2m_sec > 119) {
                this->cur_2m_sec = 0;
            }

            if (this->cur_sec > 59) {
                this->cur_sec = 0;
                this->cur_min++;

                if (this->cur_min > 59) {
                    this->cur_min = 0;
                    this->cur_hour++;

                    // new hour, lets check the GPS for the current time to correct drift.
                    GPS_Timestamp_t ts = Wesp_GPS.updateAndFetchCurrentTime();
                    this->cur_sec = ts.second;
                    this->cur_min = ts.minute;
                    this->cur_hour = ts.hour;
                    this->day = ts.day;
                    this->month = ts.month;
                    this->year = ts.year;

                    if (this->cur_hour > 23) {
                        this->cur_hour = 0;
                        // reset daily counters
                        this->resetCountersForNewDay();
                    }
                }

                this->rain_accum_60m_samples[this->cur_min] = 0;
                this->wind_gust_10m_samples[this->cur_min % 10] = 0;
            }

            if (this->cur_sec % 10 == 0) {
                float batt_vol = get_batt_vin_v();
                log_d("Read batt voltage: %f", batt_vol);
            }

            if (curTimestamp - last_calc_wx_timestamp >= CALC_WX_DATA_INTERVAL_MS) {
                this->calcWeatherData();
                last_calc_wx_timestamp = curTimestamp;
            }

            if (curTimestamp - last_report_timestamp >= this->reportWeatherDataDelay) {
                this->reportWeatherData();
                last_report_timestamp = curTimestamp;
            }
            lastSecondTimestamp = curTimestamp;
        }
        vTaskDelay(pdMS_TO_TICKS(WEATHER_DATA_UPDATE_PERIOD_MS));
    }
}

void Wesp_Sensors_Class::updateWindData() {
    //log_d("updating wind data");
    this->last_wind_dir = this->getWindDir();
    // wind speed is updated via interrupts and the monitor thread
    this->wind_dir_instantaneous = this->last_wind_dir->degree;
    this->wind_speed_avg_120s_samples[this->cur_2m_sec] = this->wind_speed_kmph_instantaneous;
    this->wind_dir_avg_120s_samples[this->cur_2m_sec] = this->wind_dir_instantaneous;
    
    // check if current wind speed is a gust for the current min in the 10 min window
    if (this->wind_speed_kmph_instantaneous > this->wind_gust_10m_samples[this->cur_min % 10]) {
        this->wind_gust_10m_samples[this->cur_min % 10] = this->wind_speed_kmph_instantaneous;
        this->wind_gust_dir_10m_samples[this->cur_min % 10] = this->wind_dir_instantaneous;
    }

    // check if a gust for the day
    if (this->wind_speed_kmph_instantaneous > this->wind_gust_kmph_day) {
        this->wind_gust_kmph_day = this->wind_speed_kmph_instantaneous;
        this->wind_gust_dir_day = this->wind_dir_instantaneous;
    }
}

void Wesp_Sensors_Class::calcWeatherData() {
    uint8_t i;
    //current winddir, current windspeed, windgustmph, and windgustdir are calculated every 100ms throughout the day
    //log_d("Starting weather data crunches.");
    //TickType_t startCrunchesTickstamp = xTaskGetTickCount();
    this->wakeAtmosphericWaitForMeasurement();
    WU_TakeI2cLock(pdMS_TO_TICKS(1000));
    this->temp_c = this->atmosphericSensor->readTempC();
    this->humidity = this->atmosphericSensor->readFloatHumidity();
    this->pressure = this->atmosphericSensor->readFloatPressure();
    this->barometric_pressure_in = (this->pressure / 10) * 0.295300; // hPa -> kPa -> inHg
    this->dew_point_c = this->atmosphericSensor->dewPointC();
    log_d("barometer based altitude: %f", this->atmosphericSensor->readFloatAltitudeMeters());
    WU_ReleaseI2cLock();

	//Calc windspdmph_avg2m
	float temp = 0;
	for(i = 0 ; i < 120 ; i++)
		temp += this->wind_speed_avg_120s_samples[i];
	temp /= 120.0;
    this->wind_speed_kmph_2m_avg = temp;

    if (this->wind_speed_kmph_2m_avg > WIND_SPEED_FAST_REPORTING_THRESHOLD_KMPH) {
        this->reportWeatherDataDelay = REPORT_TIME_INTERVAL_FAST_MS;
    } else {
        this->reportWeatherDataDelay = REPORT_TIME_INTERVAL_SLOW_MS;
    }

	//Calc winddir_avg2m, Wind Direction
	//You can't just take the average. Google "mean of circular quantities" for more info
	//We will use the Mitsuta method because it doesn't require trig functions
	//And because it sounds cool.
	//Based on: http://abelian.org/vlf/bearings.html
	//Based on: http://stackoverflow.com/questions/1813483/averaging-angles-again
	double sum = this->wind_dir_avg_120s_samples[0];
	float D = this->wind_dir_avg_120s_samples[0];
	for(i = 1; i < 120; i++) {
		float delta = this->wind_dir_avg_120s_samples[i] - D;

		if(delta < -180.0)
			D += delta + 360.0;
		else if(delta > 180.0)
			D += delta - 360.0;
		else
			D += delta;

		sum += D;
	}
    this->wind_dir_2m_avg = sum / 120;
	if(this->wind_dir_2m_avg >= 360) this->wind_dir_2m_avg -= 360;
	if(this->wind_dir_2m_avg < 0) this->wind_dir_2m_avg += 360;


	//Calc windgustmph_10m
	//Calc windgustdir_10m
	//Find the largest windgust in the last 10 minutes
    this->wind_gust_kmph_10m_top = 0.0;
    this->wind_gust_dir_10m_top = 0.0;
	//Step through the 10 minutes
	for(i = 0; i < 10 ;i++) {
		if(this->wind_gust_10m_samples[i] > this->wind_gust_kmph_10m_top) {
			this->wind_gust_kmph_10m_top = this->wind_gust_10m_samples[i];
            this->wind_gust_dir_10m_top = this->wind_gust_dir_10m_samples[i];
		}
	}

	//Total rainfall for the day is calculated within the interrupt
	//Calculate amount of rainfall for the last 60 minutes
    this->rain_accum_hour = 0;
	for(i = 0 ; i < 60 ; i++)
		this->rain_accum_hour += this->rain_accum_60m_samples[i];
    /*
	//Calc light level
	light_lvl = get_light_level();
    */

   // usually takes 13ms
   //log_d("Finished weather data crunching, took: %d", pdTICKS_TO_MS(xTaskGetTickCount() - startCrunchesTickstamp));
}


void Wesp_Sensors_Class::reportWeatherData() {
    sprintf(this->timestampStr, "%d-%02d-%02d %02d:%02d:%02d", this->year, this->month, this->day, this->cur_hour, this->cur_min, this->cur_sec);

    DynamicJsonDocument jsonOutDoc(JSON_OBJECT_SIZE(24));
    jsonOutDoc["action"] = "updateraw";
    jsonOutDoc["id"] = "test-wu-id"; //WESP_WUNDERGROUND_ID;
    jsonOutDoc["password"] = "test-wu-password"; //WESP_WUNDERGROUND_PASSWORD;
    jsonOutDoc["dateutc"] = this->timestampStr; 
    jsonOutDoc["winddir"] = this->wind_dir_instantaneous;
    jsonOutDoc["windspeedkmph"] = this->wind_speed_kmph_instantaneous;
    jsonOutDoc["windgustkmph"] = this->wind_gust_kmph_day;
    jsonOutDoc["windgustdir"] = this->wind_gust_dir_day;
    jsonOutDoc["windspeedkmph_avg2m"] = this->wind_speed_kmph_2m_avg;
    jsonOutDoc["winddir_avg2m"] = this->wind_dir_2m_avg;
    jsonOutDoc["windgustkmph_10m"] = this->wind_gust_kmph_10m_top;
    jsonOutDoc["windgustdir_10m"] = this->wind_gust_dir_10m_top;
    jsonOutDoc["humidity"] = this->humidity;
    jsonOutDoc["tempc"] = this->temp_c;
    jsonOutDoc["rainmm"] = this->rain_accum_hour;
    jsonOutDoc["dailyrainmm"] = this->rain_accum_day;
    jsonOutDoc["pressure"] = this->pressure;
    jsonOutDoc["baromin"] = this->barometric_pressure_in;
    jsonOutDoc["dewpointc"] = this->dew_point_c;
    jsonOutDoc["vin"] = this->batt_vin_v;
    jsonOutDoc["solstat"] = SolarStatusToNiceStr(this->solar_status);

    char jsonStr[1024];

    serializeJson(jsonOutDoc, jsonStr, 1024);

    Wesp_MQTT.sendData(Wesp_Config.getWxTopic(), jsonStr);
}

void Wesp_Sensors_Class::sendLightningEvent() {
    if (!this->lightningSensorAvailable) {
        return;
    }

    sprintf(this->timestampStr, "%d-%02d-%02d %02d:%02d:%02d", this->year, this->month, this->day, this->cur_hour, this->cur_min, this->cur_sec);
    
    DynamicJsonDocument jsonOutDoc(JSON_OBJECT_SIZE(4));
    jsonOutDoc["dateutc"] = this->timestampStr; 
    jsonOutDoc["lt"] = this->lastLightningEvent.type;
    jsonOutDoc["le"] = this->lastLightningEvent.energy;
    jsonOutDoc["ldkm"] = this->lastLightningEvent.distanceKm;

    char jsonStr[255];

    serializeJson(jsonOutDoc, jsonStr, 255);

    Wesp_MQTT.sendData(Wesp_Config.getWxLightningTopic(), jsonStr);
}

bool Wesp_Sensors_Class::wakeAtmosphericWaitForMeasurement() {
    bool waiting_for_meassurng_failed = true;
    uint8_t i;
    WU_TakeI2cLock(pdMS_TO_TICKS(1000));
    this->atmosphericSensor->setMode(MODE_FORCED);
    for (i = 0; i < 20; i++) {
        // wait for the measuring to start
        if (this->atmosphericSensor->isMeasuring()) {
            waiting_for_meassurng_failed = false;
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(ATMOSPHERIC_SENSOR_WAIT_UNTIL_MEASSURING_START_P_MS));
    }

    if (waiting_for_meassurng_failed) {
        log_e("Waiting for the atmospheric sensor to start measuring failed. Waited %d (ms)", ATMOSPHERIC_SENSOR_WAIT_UNTIL_MEASSURING_START_P_MS * 20);
    } 

    // this might be an issue, but until its proven we will just continue on.
    waiting_for_meassurng_failed = true;

    for (i = 0; i < 10; i++) {
        // wait for measuring to end
        if (!this->atmosphericSensor->isMeasuring()) {
            waiting_for_meassurng_failed = false;
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(ATMOSPHERIC_SENSOR_WAIT_UNTIL_MEASSURING_FINISH_P_MS));
    }

    if (waiting_for_meassurng_failed) {
        log_e("Waiting for the atmospheric sensor to finishing measuring failed. Waited %d (ms)", ATMOSPHERIC_SENSOR_WAIT_UNTIL_MEASSURING_FINISH_P_MS * 10);
    }

    WU_ReleaseI2cLock();

    return waiting_for_meassurng_failed;
}

Wesp_Sensors_Class Wesp_Sensors;