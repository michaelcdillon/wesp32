#ifndef WESP_WEBSERVER_H
#define WESP_WEBSERVER_H
#include <Arduino.h>
#include <ESPAsyncWebServer.h>
#include <esp_timer.h>

#define DEFAULT_LISTEN_PORT 80
#define ROOT_ENDPOINT "/"
#define HEALTH_ENDPOINT "/health"
#define UPDATE_CHECK_FOR_LATEST "/update/check-for-latest"
#define UPDATE_TO_LATEST "/update/update-to-latest"
#define UPDATE_TO_SPECIFIC "/update/update-to-specific"
#define SYSTEM_RESTART "/system/restart"
#define SYSTEM_WIFI_CREDS_ENDPOINT "/system/wifi"


class Wesp_Webserver_Class {
    private:
        SemaphoreHandle_t webOTAUpdateLastProgressSemaphore;
        uint64_t webOTAUpdateLastProgressMs;
        AsyncWebServer server = AsyncWebServer(DEFAULT_LISTEN_PORT);
        void setupHandlers();

    public:
        Wesp_Webserver_Class();
        void start();
        uint64_t getWebOTAUpdateLastProgressMs();
        void setWebOTAUpdateLastProgressMs(uint64_t newValue);
};

extern Wesp_Webserver_Class Wesp_Webserver;

#endif // WESP_WEBSERVER_H