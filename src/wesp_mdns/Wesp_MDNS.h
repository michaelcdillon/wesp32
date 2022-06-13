#ifndef Wesp_MDNS_H
#define Wesp_MDNS_H

#include <stdint.h>
#include <stdlib.h>

#define Wesp_HTTP_MDNS_PROTOCOL "_wesp-http"

class Wesp_MDNS_Class {
    private:
        bool started;
        char* hostname;
        char* instanceName;
    
    public:
        Wesp_MDNS_Class();
        void start(const char* deviceId);
        void stop();
        bool isRunning();
};

extern Wesp_MDNS_Class Wesp_MDNS;

#endif // Wesp_MDNS_H