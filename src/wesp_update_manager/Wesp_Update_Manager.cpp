#include "Wesp_Update_Manager.h"
#include <string.h>
#include "esp_log.h"
#include "esp_ota_ops.h"
#include "esp_http_client.h"
#include "esp_https_ota.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "tasks/Tasks.h"
#include <esp32-hal-log.h>

void Wesp_Update_Task(void* params) {
    log_d("Wesp OTA Updater starting.");

    esp_err_t ota_finish_err = ESP_OK;
    esp_http_client_config_t http_config;
    http_config.url = Wesp_Update_Manager.getUpdateFirmwareUrl();
    http_config.use_global_ca_store = true;

    esp_https_ota_config_t ota_config = {
        .http_config = &http_config,
    };

    esp_https_ota_handle_t https_ota_handle = NULL;
    
    esp_err_t err = esp_https_ota_begin(&ota_config, &https_ota_handle);

    if (err != ESP_OK) {
        log_e("ESP OTA failed. Rebooting.");
        vTaskDelay(pdMS_TO_TICKS(1000));
        esp_restart();
    }

    esp_app_desc_t app_desc;
    err = esp_https_ota_get_img_desc(https_ota_handle, &app_desc);
    if (err != ESP_OK) {
        log_e("ESP OTA app description read failed. Rebooting");
        vTaskDelay(pdMS_TO_TICKS(1000));
        esp_restart();
    }

    int bytes_downloaded = 0;
    while(1) {
        err = esp_https_ota_perform(https_ota_handle);
        
        if (err != ESP_ERR_HTTPS_OTA_IN_PROGRESS) {
            break;
        }

        bytes_downloaded =  esp_https_ota_get_image_len_read(https_ota_handle);
        
        log_d("New firmware bytes read: %d", bytes_downloaded);
        
        Wesp_Update_Manager.setFirmwareBytesDownloaded(bytes_downloaded);
    }

    if (!esp_https_ota_is_complete_data_received(https_ota_handle)) {
        log_e("The new firmware image was not completely downloaded. Rebooting.");
    } else {
        ota_finish_err = esp_https_ota_finish(https_ota_handle);
        
        if ((err == ESP_OK) && (ota_finish_err == ESP_OK)) {
            log_i("New firmware successfully downloaded and setup to execute next, rebooting.");
        } else {
            if (ota_finish_err == ESP_ERR_OTA_VALIDATE_FAILED) {
                log_e("New firmare image validation failed, image downloaded is corrupt.");
            }
            log_e("Wesp OTA upgrade failed 0x%x", ota_finish_err);
        }
    }

    vTaskDelay(pdMS_TO_TICKS(1000));
    esp_restart();

    vTaskDelete(NULL);
}

Wesp_Update_Manager_Class::Wesp_Update_Manager_Class() {
    this->initialized = false;
    this->newFirmwareBytesDownloaded = 0;
}

void Wesp_Update_Manager_Class::init() {
}

bool Wesp_Update_Manager_Class::shouldUpdate() {
    ESP_LOGI(TAG, "Checking if update should be performed for b: %d v: %d at %s", 
        this->updateFirmwareBranch, this->updateFirmwareVersion, this->updateFirmwareUrl);
    
    if (Wesp_Version.getBranch() == this->updateFirmwareBranch) {
        // same branch, update if newer version is > current
        log_d("Same branch upgrade - b: %d v: %d to v: %d", Wesp_Version.getBranch(), 
            Wesp_Version.getVersion(), this->updateFirmwareVersion);

        return this->updateFirmwareVersion > Wesp_Version.getVersion();
    }

    if ((Wesp_Version.getBranch() == Wesp_VersionBranch::DEV || 
         Wesp_Version.getBranch() == Wesp_VersionBranch::STAGE) && 
        this->updateFirmwareBranch == Wesp_VersionBranch::PROD) {
        
        log_d("Switching branch to prod from b: %d - v: %d to v: %d", Wesp_Version.getBranch(), 
            Wesp_Version.getVersion(), this->updateFirmwareVersion);

        return true;
    }

    log_w("Not updating current b: %d v: %d to b: %d v: %d", 
            Wesp_Version.getBranch(), Wesp_Version.getVersion(), this->updateFirmwareBranch, this->updateFirmwareVersion);

    return false;
}

void Wesp_Update_Manager_Class::performUpdate(char* updateFirmwareUrl, uint8_t updateVersion, 
                                              Wesp_VersionBranch updateBranch, bool force) {

    strcpy(updateFirmwareUrl, this->updateFirmwareUrl);
    this->updateFirmwareVersion = updateVersion;
    this->updateFirmwareBranch = updateBranch;

    if (force) {
        log_w("Forcing update.");
    }

    if (force || this->shouldUpdate()) {
        log_i("Initiating update to b: %d v: %d.",
            this->updateFirmwareBranch, this->updateFirmwareVersion);

        this->newFirmwareBytesDownloaded = 0;
        Tasks.stopAllAndStartMqttOTAUpdateTask();
    } else {
        log_w("Refusing to update to b: %d v: %d",
            this->updateFirmwareBranch, this->updateFirmwareVersion);
    } 
}

bool Wesp_Update_Manager_Class::hasBeenInitialized() {
    return this->initialized;
}

char* Wesp_Update_Manager_Class::getUpdateFirmwareUrl() {
    return this->updateFirmwareUrl;
}

void Wesp_Update_Manager_Class::setFirmwareBytesDownloaded(int firmwareBytesDownloaded) {
    this->newFirmwareBytesDownloaded = firmwareBytesDownloaded;
}

int Wesp_Update_Manager_Class::getFirmwareBytesDownloaded() {
    return this->newFirmwareBytesDownloaded;
}

Wesp_Update_Manager_Class Wesp_Update_Manager;
