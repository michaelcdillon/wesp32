#include "Wesp_Version.h"

Wesp_VersionClass::Wesp_VersionClass() {
    this->branch = static_cast<Wesp_VersionBranch>(VERSION_BRANCH); // supplied by platformio build flag
    this->version = VERSION;                                       // supplied by platformio build flag
}

Wesp_VersionBranch Wesp_VersionClass::getBranch() {
    return this->branch;
}

const char* Wesp_VersionClass::getBranchName() {
    switch (this->branch) {
        case DEV:
            return "DEV";
        case STAGE:
            return "STAGE";
        case PROD:
            return "PROD";
        default:
            return "UNKOWN"; 
    }
}

uint32_t Wesp_VersionClass::getVersion() {
    return this->version;
}

Wesp_VersionClass Wesp_Version;