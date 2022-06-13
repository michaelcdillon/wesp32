#include "Wesp_MQTT_Conf.h"

esp_mqtt_client_config_t config;

esp_mqtt_client_config_t* generate_mqtt_client_conf(
    const char* uri, const char* username, const char* password
) {
    config.uri = uri;
    config.username = username;
    config.password = password;
    config.protocol_ver = MQTT_PROTOCOL_V_3_1_1;

    return &config;
}