#include "Wesp_Webserver.h"
#include <Arduino.h>
#include <functional>
#include <ESPAsyncWebServer.h>
#include <ArduinoJson.h>
#include <AsyncJson.h>
#include <Update.h>
#include "wesp_version/Wesp_Version.h"
#include "wesp_eeprom/Wesp_EEPROM.h"
#include "tasks/Tasks.h"

bool updateInitiated = false;
float_t updateProgress = 0.0f;

void notFoundHandler(AsyncWebServerRequest* req) {
    log_i("Not Found Fired for: %s", req->url().c_str());
    req->send(404, "text/plain", "Not Found");
}

void handleRootRequest(AsyncWebServerRequest* req) {
    log_i("Get Request for Root");

    AsyncResponseStream* resp = req->beginResponseStream("text/html");
    resp->setCode(200);
    resp->print("<html><head></head></body>");
    resp->print("<h1>WESP32 - Weather Station via ESP32</h1>");
    resp->print("<ul>");
    resp->printf("<li>Branch: %s</li>", Wesp_Version.getBranchName());
    resp->printf("<li>Version: %d</li>", Wesp_Version.getVersion());
    resp->print("</ul>");
    resp->print("</body></html>");
    req->send(resp);
}

void handleHealthCheck(AsyncWebServerRequest* req) {

    if (!updateInitiated) {
        log_i("Health Check Handler.");
        req->send(200, "text/plain", "OK");
    }
    else {
        log_i("Health Check Handler with Update message.");
        req->send(200, "text/plain", "OK - UPDATING");
    }
}

void handleSetWifiCreds(AsyncWebServerRequest* req, JsonVariant& jsonBody) {
    if (req->method() != HTTP_POST) {
        log_w("Set Wifi Creds endpoint called with: %s method.", req->methodToString());

        notFoundHandler(req);
        return;
    }
    else if (!jsonBody.is<JsonObject>()) {
        log_w("Set wifi creds called without a json object.");

        req->send(400, "text/plain", "Error: JSON Object required.");
        return;
    }

    JsonObject wifiCreds = jsonBody.as<JsonObject>();

    if (!wifiCreds.containsKey("ssid")) {
        log_w("Set wifi creds called without a 'ssid' key.");

        req->send(400, "text/plain", "Error: JSON key 'ssid' required.");
        return;
    }
    else if (!wifiCreds.containsKey("password")) {
        log_w("Set wifi creds called without a 'password' key.");

        req->send(400, "text/plain", "Error: JSON key 'password' required.");
        return;
    }

    Wesp_EEPROM.setWifiPasscode(wifiCreds["password"]);
    Wesp_EEPROM.setWifiSSID(wifiCreds["ssid"]);


    log_d("Request to set wifi creds.");
    req->send(200, "text/plain", "OK");
    
    Tasks.restartWifi();
}

void handleUpdateSpecificUpdateFirmwareForm(AsyncWebServerRequest *request) {
    request->send(200, "text/html", "<form method='POST' action='/update/update-to-specific' enctype='multipart/form-data'><input type='file' name='update'><input type='submit' value='Update'></form>");
}

void handleUpdateToSpecificFirmware(AsyncWebServerRequest *request, String filename, size_t index, uint8_t *data, size_t len, bool final) {
    if(!index){
        Wesp_Webserver.setWebOTAUpdateLastProgressMs(esp_timer_get_time());
        Tasks.startWebOTAWatchdog();
        updateInitiated = true;
        log_i("Stopping all tasks for update...");
        Tasks.stopAllCritical();
        Tasks.stopAllSecondary();
        log_i("Stopped all tasks, preparing to update.");

        Serial.printf("Update Start: %s\n", filename.c_str());
        if(!Update.begin((ESP.getFreeSketchSpace() - 0x1000) & 0xFFFFF000)){
            Update.printError(Serial);
        }

        Update.onProgress([](uint32_t progress, uint32_t total) {
            updateProgress = (progress / (total / 100));
        });
    }
    if(!Update.hasError()){
        if(Update.write(data, len) != len){
            Update.printError(Serial);
        }
        Wesp_Webserver.setWebOTAUpdateLastProgressMs(esp_timer_get_time());
    }
    if(final){
        Tasks.stopWebOTAWatchdog();
        if(Update.end(true)){
            Serial.printf("Update Success: %uB\n", index+len);
        } else {
            Update.printError(Serial);
        }
    }
}

void handleSystemRestartRequest(AsyncWebServerRequest* req) {
    log_i("System Restart Request");
    log_w("Restarting...");

    Tasks.startRestartTask(5000);
    req->send(200, "text/plain", "OK");
}

Wesp_Webserver_Class::Wesp_Webserver_Class() {
    this->webOTAUpdateLastProgressMs = 0;
    this->webOTAUpdateLastProgressSemaphore = xSemaphoreCreateBinary();

    if (this->webOTAUpdateLastProgressSemaphore == NULL) {
        log_e("Failed to create the web ota last update progress semaphore.");
    } else {
        xSemaphoreGive(this->webOTAUpdateLastProgressSemaphore);
    }
    
    this->setupHandlers();
}

void Wesp_Webserver_Class::start() {
    this->server.begin();
}

void Wesp_Webserver_Class::setupHandlers() {
    this->server.onNotFound(notFoundHandler);
    this->server.on(ROOT_ENDPOINT, HTTP_GET, handleRootRequest);
    this->server.on(HEALTH_ENDPOINT, HTTP_GET, handleHealthCheck);
    this->server.addHandler(new AsyncCallbackJsonWebHandler(SYSTEM_WIFI_CREDS_ENDPOINT, handleSetWifiCreds));
    this->server.on(SYSTEM_RESTART, HTTP_POST, handleSystemRestartRequest);
    this->server.on(UPDATE_TO_SPECIFIC, HTTP_GET, handleUpdateSpecificUpdateFirmwareForm);
    this->server.on(UPDATE_TO_SPECIFIC, HTTP_POST, [](AsyncWebServerRequest *req) {
        bool shouldReboot = !Update.hasError();
        uint32_t respCode = 200; 
        if (shouldReboot) {
            log_i("Rebooting after successful firmware upload.");
            Tasks.startRestartTask(1000);
        }
        else {
            log_e("Failed to store the new firmware successfully.");
            respCode = 500;
            log_i("Restarting tasks after failed update attempt.");
            Tasks.startAllCritical();
            Tasks.startAllSecondary();
            updateInitiated = false;
        }
        AsyncWebServerResponse *response = req->beginResponse(respCode, "text/plain", shouldReboot?"OK":"FAIL");
        response->addHeader("Connection", "close");
        
        req->send(response);
    }, handleUpdateToSpecificFirmware);
}

void Wesp_Webserver_Class::setWebOTAUpdateLastProgressMs(uint64_t newValue) {
    if (xSemaphoreTake(this->webOTAUpdateLastProgressSemaphore, 100) == pdTRUE) {
        this->webOTAUpdateLastProgressMs = newValue;
        xSemaphoreGive(this->webOTAUpdateLastProgressSemaphore);
    }
    else {
        log_w("Failed to take the web ota update last progress semaphore during set.");
    }
}

uint64_t Wesp_Webserver_Class::getWebOTAUpdateLastProgressMs() {
    uint64_t returnValue = 0;
    if (xSemaphoreTake(this->webOTAUpdateLastProgressSemaphore, 50) == pdTRUE) {
        returnValue = this->webOTAUpdateLastProgressMs;
        xSemaphoreGive(this->webOTAUpdateLastProgressSemaphore);
    }
    else {
        log_w("Failed to take the web ota update last progress semaphore during fetch.");
    }

    return returnValue;
}

Wesp_Webserver_Class Wesp_Webserver;
