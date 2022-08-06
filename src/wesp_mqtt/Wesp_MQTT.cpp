#include "ArduinoJson.h"
#undef IPADDR_NONE // this has to be right after the arduino json call because arduino json depends on something which defines this, but mqtt uses lwip directly
#include "Wesp_MQTT.h"
#include "mqtt_client.h"
#include "esp_event.h"
#include "esp_tls.h"
#include "freertos/queue.h"
#include <esp32-hal-log.h>
#include "wesp_mqtt/Wesp_MQTT_Conf.h"
#include "wesp_config/Wesp_Config.h"
#include <string.h>

extern const char isrg_root_x1_pem_start[] asm("_binary_src_certs_isrg_x1_pem_start");
extern const char isrg_root_x1_pem_end[] asm("_binary_src_certs_isrg_x1_pem_end");

#define DATA_TO_BROADCAST_QUEUE_SIZE 100

typedef struct BroadcastData {
    esp_mqtt_client* mqtt_client;
    const char* topic;
    const char* data;
} BroadcastData_t;

static xQueueHandle dataToBroadcastQueue = NULL;


void SendMqttDataTask(void* arg) {
    BroadcastData broadcastData;
    log_d("in brodcast mqtt data queue loop.");
    for(;;) {
        if (xQueueReceive(dataToBroadcastQueue, &broadcastData, portMAX_DELAY)) {
            log_i("publishing message: %s", broadcastData.data);
            esp_mqtt_client_publish(broadcastData.mqtt_client, broadcastData.topic, broadcastData.data, 0, 0, 0);
        }
    }

    vTaskDelete(NULL);
}

static void handle_mqtt_message(esp_mqtt_client_handle_t client, int topic_len, char* topic, int payload_len, char* payload) {
    char topic_str[topic_len + 1];
    char payload_str[payload_len + 1];
    strncpy(topic_str, topic, topic_len);
    topic_str[sizeof(topic_str)-1]='\0';
    strncpy(payload_str, payload, payload_len);
    payload_str[sizeof(payload_str)-1]='\0';
    log_d("Handling mqtt msg on topic: %s - %s", topic_str, payload_str);

    if (strcmp(topic_str, Wesp_Config.getControlTopic()) == 0) {
        // control topic, this is json
        DynamicJsonDocument control_msg(512);
        DeserializationError err = deserializeJson(control_msg, payload_str);

        if (err != DeserializationError::Ok) {
            log_e("Failed to deserialize the control message due to: %d", err);
            return;
        }

        if (strcmp(control_msg["c"], "ChkIn") == 0) {
            log_i("Got the ChkIn command, checking in now...");
            esp_mqtt_client_publish(client, Wesp_Config.getAnnounceStationTopic(), Wesp_Config.getAnnounceOnMessage(), strlen(Wesp_Config.getAnnounceOnMessage()), 0, 0);
        }
    }
    else {
        log_w("Not handling message from unknown topic: %s - %s", topic_str, payload_str);
    }
}

static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data) {
    log_i("Event dispatched from event loop base=%s, event_id=%d", base, event_id);
    esp_mqtt_event_handle_t event = (esp_mqtt_event_handle_t) event_data;
    esp_mqtt_client_handle_t client = event->client;
    int msg_id;
    switch ((esp_mqtt_event_id_t)event_id) {
    case MQTT_EVENT_CONNECTED:
        log_i("MQTT_EVENT_CONNECTED");
        msg_id = esp_mqtt_client_subscribe(client, Wesp_Config.getControlTopic(), 0);
        log_i("sent subscribe successful, msg_id=%d", msg_id);

        esp_mqtt_client_publish(client, Wesp_Config.getAnnounceStationTopic(), Wesp_Config.getAnnounceOnMessage(), strlen(Wesp_Config.getAnnounceOnMessage()), 0, 0);
        Wesp_MQTT.setConnected(true);
        break;
    case MQTT_EVENT_DISCONNECTED:
        log_i("MQTT_EVENT_DISCONNECTED");
        Wesp_MQTT.setConnected(false);
        break;

    case MQTT_EVENT_SUBSCRIBED:
        log_i("MQTT_EVENT_SUBSCRIBED, msg_id=%d", event->msg_id);
        //msg_id = esp_mqtt_client_publish(client, "/topic/qos0", "data", 0, 0, 0);
        //log_i("sent publish successful, msg_id=%d", msg_id);
        break;
    case MQTT_EVENT_UNSUBSCRIBED:
        log_i("MQTT_EVENT_UNSUBSCRIBED, msg_id=%d", event->msg_id);
        break;
    case MQTT_EVENT_PUBLISHED:
        log_i("MQTT_EVENT_PUBLISHED, msg_id=%d", event->msg_id);
        break;
    case MQTT_EVENT_DATA:
        log_d("MQTT_EVENT_DATA");
        handle_mqtt_message(event->client, event->topic_len, event->topic, event->data_len, event->data);
        break;
    case MQTT_EVENT_ERROR:
        log_i("MQTT_EVENT_ERROR");
        if (event->error_handle->error_type == MQTT_ERROR_TYPE_TCP_TRANSPORT) {
            log_i("Last error code reported from esp-tls: 0x%x", event->error_handle->esp_tls_last_esp_err);
            log_i("Last tls stack error number: 0x%x", event->error_handle->esp_tls_stack_err);
            log_i("Last captured errno : %d (%s)",  event->error_handle->esp_transport_sock_errno,
                     strerror(event->error_handle->esp_transport_sock_errno));
        } else if (event->error_handle->error_type == MQTT_ERROR_TYPE_CONNECTION_REFUSED) {
            log_i("Connection refused error: 0x%x", event->error_handle->connect_return_code);
        } else {
            log_w("Unknown error type: 0x%x", event->error_handle->error_type);
        }
        break;
    default:
        log_i("Other event id:%d", event->event_id);
        break;
    }
}

Wesp_MQTT_Class::Wesp_MQTT_Class() {
    /*
    uint32_t i = 0;
    uint32_t root_len =  (uint32_t) strlen((const char*) dillon_local_root_ca_start);
    uint32_t machine_len = (uint32_t) strlen((const char*) dillon_local_machine_ca_start);
    char ca_certs[root_len + machine_len];

    log_i("root ca len: %d - machine ca len: %d", root_len, machine_len); 

    for (i = 0; i < root_len; i++)  {
        ca_certs[i] = dillon_local_root_ca_start[i];
    }

    for (i = 0; i < machine_len; i++) {
        ca_certs[i + root_len] = dillon_local_machine_ca_start[i];
    }

    log_i("ca certs: \n%s", ca_certs); 
    
    ESP_ERROR_CHECK(esp_tls_init_global_ca_store());
    ESP_ERROR_CHECK(esp_tls_set_global_ca_store((const unsigned char*) ca_certs, root_len + machine_len));
    */

   this->initialized = false;
   this->connected = false;
}

void Wesp_MQTT_Class::init(const char* mqtt_uri, const char* username, const char* password) {
    dataToBroadcastQueue = xQueueCreate(DATA_TO_BROADCAST_QUEUE_SIZE, sizeof(BroadcastData_t));

    //ESP_ERROR_CHECK(esp_event_loop_create_default()); // likely not needed any more

    this->mqtt_cfg = generate_mqtt_client_conf(mqtt_uri, username, password, isrg_root_x1_pem_start, 1923);

    this->mqtt_client = esp_mqtt_client_init(this->mqtt_cfg);

    if (this->mqtt_client == NULL) {
        log_e("Failed to initialize the mqtt client, the mqtt cfg is likely setup incorrectly.");
        return;
    }

    ESP_ERROR_CHECK(esp_mqtt_client_register_event(this->mqtt_client, MQTT_EVENT_ANY, mqtt_event_handler, this->mqtt_client));

    this->initialized = true;
    log_i("mqtt client initialized.");
}

void Wesp_MQTT_Class::start() {
    if (!this->initialized) {
        return;
    }
    ESP_ERROR_CHECK(esp_mqtt_client_start(this->mqtt_client));

    log_i("mqtt client started.");
}

void Wesp_MQTT_Class::stop() {
    if (!this->initialized) {
        return;
    }
    ESP_ERROR_CHECK(esp_mqtt_client_stop(this->mqtt_client));
}

void Wesp_MQTT_Class::sendData(const char* topic, const char* data) {
    if (!this->initialized) {
        return;
    }
    BroadcastData broadcastData = {
        this->mqtt_client,
        topic,
        data
    };

    xQueueSendToBack(dataToBroadcastQueue, &broadcastData, 5);
}

void Wesp_MQTT_Class::setConnected(bool status) {
    this->connected = status;
}

bool Wesp_MQTT_Class::isConnected() {
    return this->connected;
}

Wesp_MQTT_Class Wesp_MQTT;