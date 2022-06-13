#ifndef WESP_VERSION_H
#define WESP_VERSION_H

#include "FreeRTOS.h"

enum Wesp_VersionBranch {
    UNKOWN = 0,
    DEV = 1,
    STAGE = 2,
    PROD = 3
};


class Wesp_VersionClass {
    private:
        uint32_t version = 0;
        Wesp_VersionBranch branch = Wesp_VersionBranch::UNKOWN;

    public:
        Wesp_VersionClass();
        Wesp_VersionBranch getBranch();
        const char* getBranchName();
        uint32_t getVersion();    
};

extern Wesp_VersionClass Wesp_Version;

#endif // WESP_VERSION_H