#include "Tasks.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <Arduino.h>
#include "wesp_config/Wesp_Config.h"
#include "wesp_eeprom/Wesp_EEPROM.h"
#include "wesp_mdns/Wesp_MDNS.h"
#include "wesp_webserver/Wesp_Webserver.h"
#include "wesp_gps/Wesp_GPS.h"
#include "wesp_sensors/Wesp_Sensors.h"
#include "wesp_mqtt/Wesp_MQTT.h"
#include "wesp_update_manager/Wesp_Update_Manager.h"
#include <esp_timer.h>
#include <WiFi.h>

#define MQTT_TASK_CORE 0
#define GPS_UPDATE_TASK_CORE 0
#define WEATHER_TASK_CORE 1
#define WEATHER_UPDATE_DATA_CORE 1
#define UPDATE_TASK_CORE 1
#define OVERDUE_UPDATE_GRACE_PERIOD_MICRO_S 30000000
#define MQTT_UPDATE_STALLED_CYCLES 10

void restartWifiTask(void* params) {
    vTaskDelay(pdMS_TO_TICKS(5000));
    WiFi.disconnect();
    vTaskDelay(pdMS_TO_TICKS(1000));
    WiFi.begin(Wesp_EEPROM.readWifiSSID().c_str(), Wesp_EEPROM.readWifiPasscode().c_str());
    vTaskDelete(NULL);
}

void restartDeviceTask(void* params) {
    delay((uint32_t) params);
    ESP.restart();
    vTaskDelete(NULL);
}

void webOtaUpdateWatchdogTask(void* params) {
    vTaskDelay(pdMS_TO_TICKS(6000));

    for(;;) {
        uint64_t elapsedMs = esp_timer_get_time() - Wesp_Webserver.getWebOTAUpdateLastProgressMs();
        if (elapsedMs < 0) {
            elapsedMs = 0;
        }

        log_d("Update watch dog checking elapsed update interval: %d", elapsedMs);

        if (elapsedMs > OVERDUE_UPDATE_GRACE_PERIOD_MICRO_S) {
            log_e("An update attempt has stalled out. Restarting the device.");
            vTaskDelay(pdMS_TO_TICKS(1000));
            ESP.restart();
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(5000));
    }

    vTaskDelete(NULL);
}

void mqttOtaUpdateWatchdogTask(void* params) {
    vTaskDelay(pdMS_TO_TICKS(6000));
    int bytesDownloaded = 0;
    int newBytesDownloaded = 0;
    int downloadedBytesDelta = 0;
    uint8_t stalledCycles = 0;

    for(;;) {
        newBytesDownloaded = Wesp_Update_Manager.getFirmwareBytesDownloaded();
        downloadedBytesDelta = newBytesDownloaded - bytesDownloaded;
        bytesDownloaded = newBytesDownloaded;

        log_d("Mqtt update watch dog checking downloaded new firmware bytes delta: %d", downloadedBytesDelta);

        if (downloadedBytesDelta <= 0) {
            stalledCycles = stalledCycles + 1;
        } else {
            stalledCycles = 0;
        }
        
        if (stalledCycles > MQTT_UPDATE_STALLED_CYCLES) {
            log_e("An mqtt update attempt has stalled out. Restarting the device.");
            vTaskDelay(pdMS_TO_TICKS(1000));
            ESP.restart();
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(5000));
    }

    vTaskDelete(NULL);
}

void gpsUpdateFixAndCoordinatesTask(void* params) {
  log_d("GPS Update Fix and Coordiantes task is running on core: %d ...", xPortGetCoreID());
    for(;;) {
        Wesp_GPS.updateFixAndCoordinates();
        vTaskDelay(pdMS_TO_TICKS(10000));
    }
    vTaskDelete(NULL);
}

void readWeatherSensorsTask(void* params) {
    for(;;) {
        volatile Wind_Dir_t* wind_dir = Wesp_Sensors.getWindDir();
        float wind_speed = Wesp_Sensors.get_wind_speed_kmph();
        log_i("Wind dir deg: %s (%f)", WindDirNiceToStr(wind_dir->wind_dir_nice), wind_dir->degree);
        log_i("Wind speed kmph: %f", wind_speed);
        vTaskDelay(pdMS_TO_TICKS(2000)); 
    }
    vTaskDelete(NULL);
}

Tasks_Class::Tasks_Class() {
    this->runAwsMqttTask = false;
    this->initialized = false;
}

void Tasks_Class::init(bool runAwsMqttTask) {
    this->runAwsMqttTask = runAwsMqttTask;
    this->initialized = true;
}

void Tasks_Class::startAllCritical() {

    Wesp_MDNS.start(Wesp_Config.getDeviceId()); 

    xTaskCreatePinnedToCore(
        MonitorWeatherSensorInterruptsTask,
        "WxItr",
        2000,
        NULL,
        5,
        &this->HandleWeatherSensorInterruptsHandle,
        WEATHER_TASK_CORE
    );
    xTaskCreatePinnedToCore(
        UpdateWeatherDataAndReportTask,
        "WxData",
        5000,
        NULL,
        4,
        &this->WeatherUpdateDataAndReportTaskHandle,
        WEATHER_UPDATE_DATA_CORE 
    );
    Wesp_Sensors.startSensorInterrupts();
}

void Tasks_Class::startAllSecondary() {
    /* 
    if (Wesp_GPS.isPeripherialAvailable()) {
        xTaskCreatePinnedToCore(
            gpsUpdateFixAndCoordinatesTask,
            "GPS",
            2000,
            NULL,
            5,
            &this->GPSUpdateTask,
            GPS_UPDATE_TASK_CORE
        );
    }
    */

    xTaskCreatePinnedToCore(
        SendMqttDataTask, 
        "MQTT",
        3000,
        NULL,
        3,
        &this->MQQTPublishTaskHandle,
        MQTT_TASK_CORE
    );
}

void Tasks_Class::stopAllCritical() {
    Wesp_MDNS.stop();
    if (this->runAwsMqttTask) {
        vTaskDelete(this->AWSMQTTaskHandle);
    }
    vTaskDelete(this->ReadWeatherSensorsTaskHandle);
    vTaskDelete(this->HandleWeatherSensorInterruptsHandle);
    vTaskDelete(this->WeatherUpdateDataAndReportTaskHandle);
    Wesp_Sensors.stopSensorInterrupts();
}

void Tasks_Class::stopAllSecondary() {
    if (Wesp_GPS.isPeripherialAvailable()) {
        vTaskDelete(this->GPSUpdateTask);
    }

    vTaskDelete(this->MQQTPublishTaskHandle); 
}

void Tasks_Class::suspendAllTasks() {
    if (Wesp_GPS.isPeripherialAvailable()) {
        vTaskSuspend(this->GPSUpdateTask);
    }
    /*
    if (this->runAwsMqttTask) {
        vTaskSuspend(this->AWSMQTTaskHandle);
    }
    */
    vTaskSuspend(this->ReadWeatherSensorsTaskHandle);
    vTaskSuspend(this->HandleWeatherSensorInterruptsHandle);
    vTaskSuspend(this->WeatherUpdateDataAndReportTaskHandle);
    vTaskSuspend(this->MQQTPublishTaskHandle);
    Wesp_Sensors.stopSensorInterrupts();
}

void Tasks_Class::resumeAllTasks() {
    
    if (Wesp_GPS.isPeripherialAvailable()) {
        vTaskResume(this->GPSUpdateTask);
    }
    /*
    if (this->runAwsMqttTask) {
        vTaskResume(this->AWSMQTTaskHandle);
    }
    */
   vTaskResume(this->ReadWeatherSensorsTaskHandle);
   vTaskResume(this->HandleWeatherSensorInterruptsHandle);
   vTaskResume(this->WeatherUpdateDataAndReportTaskHandle);
   vTaskResume(this->MQQTPublishTaskHandle);
    Wesp_Sensors.startSensorInterrupts();
}

void Tasks_Class::startRestartTask(uint32_t delayRestartMicroSecs) {
    xTaskCreatePinnedToCore(
      restartDeviceTask,
      "restart",
      5000,
      (void*) delayRestartMicroSecs,
      3,
      &this->RestartDeviceTaskHandle,
      tskNO_AFFINITY
    );
}

void Tasks_Class::startWebOTAWatchdog() {
    log_d("Starting a web ota update watchdog task");
    xTaskCreatePinnedToCore(
        webOtaUpdateWatchdogTask,
        "updateWatchdog",
        7000,
        NULL,
        3,
        &this->WebOTAUpdateWatchdogTask,
        tskNO_AFFINITY
    );
}

void Tasks_Class::restartWifi() {
    log_d("Restarting wifi.");
    xTaskCreatePinnedToCore(
        restartWifiTask,
        "restartWifi",
        5000,
        NULL,
        3,
        &this->RestartWifiTask,
        tskNO_AFFINITY
    );
}

void Tasks_Class::stopWebOTAWatchdog() {
    vTaskDelete(this->WebOTAUpdateWatchdogTask);
}

void Tasks_Class::stopAllAndStartMqttOTAUpdateTask() {
    this->stopAllCritical();
    this->stopAllSecondary();

    xTaskCreatePinnedToCore(
        Wesp_Update_Task,
        "updateTask",
        10000,
        NULL,
        10,
        &this->UpdateTaskHandle,
        tskNO_AFFINITY
    );
}

Tasks_Class Tasks;
