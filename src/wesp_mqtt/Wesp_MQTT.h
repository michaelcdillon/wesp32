#ifndef WESP_MQTT_H
#define WESP_MQTT_H

#include "esp_event.h"
#include "mqtt_client.h"

extern void SendMqttDataTask(void* arg);

class Wesp_MQTT_Class {
    private:
        bool initialized;
        bool connected;
        esp_mqtt_client_config_t* mqtt_cfg;
        esp_event_loop_handle_t event_loop_handle;
        esp_mqtt_client_handle_t mqtt_client;

    public:
        Wesp_MQTT_Class();
        void init(const char* mqtt_uri, const char* username, const char* password);
        void start();
        void stop();
        void sendData(const char* topic, const char* data);
        bool isConnected();
        void setConnected(bool status);
};

extern Wesp_MQTT_Class Wesp_MQTT;

#endif