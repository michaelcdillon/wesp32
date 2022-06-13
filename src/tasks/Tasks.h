#ifndef OGC_TASKS_H
#define OGC_TASKS_H

#include <Arduino.h>

class Tasks_Class {
    private:
        bool runAwsMqttTask;
        bool initialized;
        TaskHandle_t RestartDeviceTaskHandle;
        TaskHandle_t AWSMQTTaskHandle;
        TaskHandle_t GPSUpdateTask;
        TaskHandle_t WebOTAUpdateWatchdogTask;
        TaskHandle_t RestartWifiTask;
        TaskHandle_t ReadWeatherSensorsTaskHandle;
        TaskHandle_t HandleWeatherSensorInterruptsHandle;
        TaskHandle_t WeatherUpdateDataAndReportTaskHandle;
        TaskHandle_t MQQTPublishTaskHandle;
        TaskHandle_t UpdateTaskHandle;

    public:
        Tasks_Class();
        void init(bool runAwsMqttTask);
        bool isInitialized();
        void startAllCritical();
        void stopAllCritical();
        void startAllSecondary();
        void stopAllSecondary();
        void suspendAllTasks();
        void resumeAllTasks();
        void startRestartTask(uint32_t delayRestartMicroSecs);
        void startWebOTAWatchdog();
        void stopWebOTAWatchdog();
        void restartWifi();
        void stopAllAndStartMqttOTAUpdateTask();
};

extern Tasks_Class Tasks;

#endif // TASKS_H