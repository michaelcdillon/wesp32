#include <functional>
#include <Arduino.h>
#include <Wire.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <esp32-hal-log.h>
#include <EEPROM.h>
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include "wesp_config/Wesp_Config.h"
#include "wesp_version/Wesp_Version.h"
#include "wesp_gps/Wesp_GPS.h"
#include "wesp_eeprom/Wesp_EEPROM.h"
#include "tasks/Tasks.h"
#include "wesp_webserver/Wesp_Webserver.h"
#include "wesp_sensors/Wesp_Sensors.h"
#include "wesp_mqtt/Wesp_MQTT.h"
//#include "wesp_wifi/Wesp_Wifi.h"

void setup() {
    Serial.begin(115200);

    log_i("test");
    
    // Wesp_Config.setupAll();
    // // put your setup code here, to run once:
    // log_i("WESP32 Booting...");
    // log_i("Firmware Branch: %s", Wesp_Version.getBranchName());
    // log_i("Firmware Version: %d", Wesp_Version.getVersion());
    // log_i("Wesp Device Id: %s", Wesp_Config.getDeviceId());
    // log_d("setup is running on core: %d...", xPortGetCoreID());

    // Wire.begin();
    // Wire.setClock(400000);
    // SPI.begin();
    // Wesp_EEPROM.begin();

    // log_i("Starting wifi...");
    // log_i("Attempting to connect to wifi ssid: %s", Wesp_EEPROM.readWifiSSID().c_str());
    // WiFi.mode(WIFI_STA);
    // WiFi.begin(Wesp_EEPROM.readWifiSSID().c_str(), Wesp_EEPROM.readWifiPasscode().c_str());
    
    // //wesp_initialize_wifi();

    
    // if (WiFi.waitForConnectResult() != WL_CONNECTED) {
    //     log_e("Failed to connect to to wifi");
    // }
    // else {
    //     log_i("Connected to wifi, got ip: %s", WiFi.localIP().toString().c_str());
    // }

    // log_i("Initializing mqtt...");
    // Wesp_MQTT.init(Wesp_Config.getMqttUri(), Wesp_Config.getMqttUsername(), Wesp_Config.getMqttPassword());

    // log_i("Starting mqtt client...");
    // Wesp_MQTT.start();

    // Wesp_GPS.init();
    // if (Wesp_GPS.isPeripherialAvailable()) {

    //     bool gpsHasFix = Wesp_GPS.waitForFix(120 * 1000); // wait 2 min for fix    

    //     if (!gpsHasFix) {
    //         log_e("Waited 2 min, for gps fix, failed to acquire one, so restarting.");
    //         //esp_restart();
    //     }

    //     // put GPS in powersave mode now.
    //     Wesp_GPS.enablePowerSaveMode();
    // } 
    // else {
    //     // look up a ntp server for current time
    //     log_w("GPS is unavailable, using defaults of 0:00:00 for time. Consider spending time to look up ntp and get time.");
    // }

    // log_i("Initializing weather sensors.");
    // Wesp_Sensors.init(Wesp_GPS.updateAndFetchCurrentTime());

    // Tasks.init(false);
    // Tasks.startAllCritical();
    // Tasks.startAllSecondary();
    // Wesp_Webserver.start();
    log_i("Setup finished.");
}

void loop() {
  // put your main code here, to run repeatedly:
}