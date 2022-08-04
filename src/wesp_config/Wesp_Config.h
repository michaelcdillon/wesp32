#ifndef Wesp_CONFIG_H
#define Wesp_CONFIG_H

class Wesp_Config_Class {
    private:
        char deviceId[13];
        char wxt[34];
        char wxlt[42];
        void setupDeviceId();
        void setupAnnounceMessages();
        void setupWxTopic();
        void setupWxLightningTopic();
    public:
        Wesp_Config_Class();
        void setupAll();
        const char* getDeviceId();
        const char* getWxTopic();
        const char* getWxLightningTopic();
        const char* getAnnounceOffMessage();
        const char* getAnnounceOnMessage();
        const char* getAnnounceStationTopic();
        const char* getControlTopic();
        const char* getMqttUri();
        const char* getMqttUsername();
        const char* getMqttPassword();
};

extern Wesp_Config_Class Wesp_Config;

#endif // Wesp_Config_H