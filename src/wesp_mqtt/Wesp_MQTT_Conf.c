#include "Wesp_MQTT_Conf.h"
#include "esp_crt_bundle.h"

esp_mqtt_client_config_t config;

esp_mqtt_client_config_t* generate_mqtt_client_conf(
    const char* uri, const char* username, const char* password, const char* certPem, size_t certLen
) {
    config.uri = uri;
    config.username = username;
    config.password = password;
    config.protocol_ver = MQTT_PROTOCOL_V_3_1_1;
    config.cert_pem = certPem;
    config.cert_len = certLen;

    return &config;
}