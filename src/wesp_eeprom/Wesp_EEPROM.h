#ifndef WESP_EEPROM_H
#define WESP_EEPROM_H

#include <EEPROM.h>

#define SSID_MAX_LENGTH_CHARS 32
#define WPA2_PASSCODE_MAX_LENGTH_CHARS 63

#define NUMBER_OF_STORED_BYTES 100
#define BLE_PIN_CODE_ADDR 0x01
#define WIFI_SSID_ADDR 0x05
#define WIFI_PASSCODE_ADDR 0x25 

class Wesp_EEPROM_Class {
    private: 
        uint8_t readAddress(uint8_t address);
        bool writeToAddress(uint8_t address, uint8_t& value);

    public:
        Wesp_EEPROM_Class();
        void begin();
        bool setBlePinCode(uint32_t pinCode);
        uint32_t readBlePinCode();
        bool setWifiSSID(std::string wifiSSID);
        std::string readWifiSSID();
        bool setWifiPasscode(std::string wifiPasscode);
        std::string readWifiPasscode();
};

extern Wesp_EEPROM_Class Wesp_EEPROM;

#endif // WESP_EEPPROM_H