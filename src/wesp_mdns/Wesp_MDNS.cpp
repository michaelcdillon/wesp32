#include "Wesp_MDNS.h"
#include <stdlib.h>
#include <stdint.h>
#include <mdns.h>
#include <esp32-hal-log.h>

Wesp_MDNS_Class::Wesp_MDNS_Class() {
    this->started = false;
}

void Wesp_MDNS_Class::start(const char* deviceId) {
    log_i("Starting up MDNS...");
    esp_err_t err = mdns_init();
    if (err) {
        log_e("Error during MDNS startup: %d", err);
        return;        
    }
    uint8_t num_service_data_entries = 1;
    mdns_txt_item_t serviceTxtData[num_service_data_entries] = {
        {"did", (char*) deviceId},
    };

    err = mdns_hostname_set(deviceId);
    if (err) {
        log_e("Error during MDNS hostname set: %d", err);
        return;        
    }
    err = mdns_instance_name_set(deviceId);
    if (err) {
        log_e("Error during MDNS instance name set: %d", err);
        return;        
    }
    err = mdns_service_add(NULL, Wesp_HTTP_MDNS_PROTOCOL, "_tcp", 80, NULL, 0);
    if (err) {
        log_e("Error during MDNS service add: %d", err);
        return;        
    }
    err = mdns_service_txt_set(Wesp_HTTP_MDNS_PROTOCOL, "_tcp", serviceTxtData, num_service_data_entries);
    if (err) {
        log_e("Error during MDNS instance name set again: %d", err);
        return;        
    }

    this->started = true;
    log_i("MDNS started");
}

void Wesp_MDNS_Class::stop() {
    mdns_free();
    this->started = false;
    log_i("MDNS stopped");
}

bool Wesp_MDNS_Class::isRunning() {
    return this->started;
}

Wesp_MDNS_Class Wesp_MDNS;