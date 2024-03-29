#include "Wesp_Config.h"
#include <string.h>
#include <stdio.h>
#include <esp_system.h>
#include "ArduinoJson.h"
#include "wesp_version/Wesp_Version.h"

#define UNINITIALIZED_DEVICE_ID "FFFFFFFFFFFF"

static char announceOnMsgJson[130];
static char announceOffMsgJson[130];
static char announceStationTopic[] = WESP_STATION_ANNOUNCE_TOPIC;
static char controlTopic[] = WESP_CONTROL_TOPIC;
static char mqttUri[35];
static char mqttUsername[] = WESP_MQTT_USERNAME;
static char mqttPassword[] = WESP_MQTT_PASSWORD;

Wesp_Config_Class::Wesp_Config_Class() {
    strcpy(this->deviceId, UNINITIALIZED_DEVICE_ID);
    sprintf(mqttUri, "%s://%s:%s", WESP_MQTT_SCHEME, WESP_MQTT_HOST, WESP_MQTT_PORT);
}

void Wesp_Config_Class::setupAll() {
    // order counts, device id is needed by the others, wtx is needed by announce messages
    this->setupDeviceId();
    this->setupWxTopic();
    log_d("wxt: %s", this->wxt);
    this->setupWxLightningTopic();
    log_d("wxlt: %s", this->wxlt);
    this->setupAnnounceMessages();
    log_d("announce on msg: %s", announceOnMsgJson);
}

void Wesp_Config_Class::setupDeviceId() {
    uint8_t macBytes[6]; 
    esp_efuse_mac_get_default(macBytes);

    sprintf(this->deviceId, "%x%x%x%x%x%x", macBytes[0], macBytes[1], macBytes[2], macBytes[3], macBytes[4], macBytes[5]);
}

void Wesp_Config_Class::setupWxTopic() {
    sprintf(this->wxt, "%s%s", WESP_STATION_WX_TOPIC_PREFIX, this->deviceId);
}

void Wesp_Config_Class::setupWxLightningTopic() {
    sprintf(this->wxlt, "%s%s", WESP_STATION_WX_LIGHTNING_TOPIC_PREFIX, this->deviceId);
}

void Wesp_Config_Class::setupAnnounceMessages() {
    StaticJsonDocument<240> announceOnMsg;
    StaticJsonDocument<240> announceOffMsg;
    
    announceOnMsg["did"] = this->deviceId;
    announceOffMsg["did"] = this->deviceId;
    announceOnMsg["s"] = "on";
    announceOffMsg["s"] = "off";
    announceOnMsg["fw"] = Wesp_Version.getVersion();
    announceOffMsg["fw"] = Wesp_Version.getVersion();
    announceOnMsg["wxt"] = this->wxt;
    announceOffMsg["wxt"] = this->wxt;
    announceOnMsg["wxlt"] = this->wxlt;
    announceOffMsg["wxlt"] = this->wxlt;

    serializeJson(announceOnMsg, announceOnMsgJson, 130);
    serializeJson(announceOffMsg, announceOffMsgJson, 130);
}

const char* Wesp_Config_Class::getDeviceId() {
    if (strcmp(this->deviceId, UNINITIALIZED_DEVICE_ID) == 0) {
        this->setupDeviceId();
    }
    return this->deviceId;
}

const char* Wesp_Config_Class::getWxTopic() {
    return this->wxt;
}

const char* Wesp_Config_Class::getWxLightningTopic() {
    return this->wxlt;
}

const char* Wesp_Config_Class::getAnnounceOffMessage() {
    return announceOffMsgJson;
}

const char* Wesp_Config_Class::getAnnounceOnMessage() {
    return announceOnMsgJson;
}

const char* Wesp_Config_Class::getAnnounceStationTopic() {
    return announceStationTopic;
}

const char* Wesp_Config_Class::getControlTopic() {
    return controlTopic;
}

const char* Wesp_Config_Class::getMqttUri() {
    return mqttUri;
}

const char* Wesp_Config_Class::getMqttUsername() {
    return mqttUsername;
}
const char* Wesp_Config_Class::getMqttPassword() {
    return mqttPassword;
}

Wesp_Config_Class Wesp_Config;