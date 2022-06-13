#ifndef WESP_MQTT_CONF_H
#define WESP_MQTT_CONF_H

#include "mqtt_client.h"

#ifdef __cplusplus
extern "C" {
#endif

esp_mqtt_client_config_t* generate_mqtt_client_conf(const char* uri, const char* username, const char* password);

#ifdef __cplusplus
}
#endif

#endif // WESP_MQTT_CONF_H