#include <wesp_eeprom/Wesp_EEPROM.h>
#include <esp32-hal-log.h>

Wesp_EEPROM_Class::Wesp_EEPROM_Class() {
}

uint8_t Wesp_EEPROM_Class::readAddress(uint8_t address) {
    return EEPROM.read(address);
}

bool Wesp_EEPROM_Class::writeToAddress(uint8_t address, uint8_t& value) {
    EEPROM.write(address, value);
    return EEPROM.commit();
}

void Wesp_EEPROM_Class::begin() {
    log_d("Setting up EEPROM with %d bytes of storage...", NUMBER_OF_STORED_BYTES);
    if (EEPROM.begin(NUMBER_OF_STORED_BYTES)) {
        log_i("EEPROM Setup.");
    }
    else {
        log_e("Failed to setup the EEPROM successfully.");
    }
}

bool Wesp_EEPROM_Class::setBlePinCode(uint32_t pinCode) {
    EEPROM.writeUInt(BLE_PIN_CODE_ADDR, pinCode);
    return EEPROM.commit();
}

uint32_t Wesp_EEPROM_Class::readBlePinCode() {
    return EEPROM.readUInt(BLE_PIN_CODE_ADDR);
}

bool Wesp_EEPROM_Class::setWifiSSID(std::string wifiSSID) {
    EEPROM.writeString(WIFI_SSID_ADDR, wifiSSID.c_str());
    return EEPROM.commit();
}

std::string Wesp_EEPROM_Class::readWifiSSID() {
    char buffer[SSID_MAX_LENGTH_CHARS];
    EEPROM.readString(WIFI_SSID_ADDR, buffer, SSID_MAX_LENGTH_CHARS);
    return buffer;
}

bool Wesp_EEPROM_Class::setWifiPasscode(std::string wifiPasscode) {
    EEPROM.writeString(WIFI_PASSCODE_ADDR, wifiPasscode.c_str());
    return EEPROM.commit();
}

std::string Wesp_EEPROM_Class::readWifiPasscode() {
    char buffer[WPA2_PASSCODE_MAX_LENGTH_CHARS];
    EEPROM.readString(WIFI_PASSCODE_ADDR, buffer, WPA2_PASSCODE_MAX_LENGTH_CHARS);
    return buffer;
}

Wesp_EEPROM_Class Wesp_EEPROM;