#ifndef WESP_UPDATE_MANAGER_H
#define WESP_UPDATE_MANAGER_H

#include "wesp_version/Wesp_Version.h"

extern void Wesp_Update_Task(void* params);

class Wesp_Update_Manager_Class {
    private:
    bool initialized;
    char updateFirmwareUrl[256];
    uint8_t  updateFirmwareVersion;
    Wesp_VersionBranch updateFirmwareBranch;
    int newFirmwareBytesDownloaded;
    bool shouldUpdate();

    public:
    Wesp_Update_Manager_Class();
    void init();
    void performUpdate(char* updateFirmwareUrl, uint8_t updateVersion, Wesp_VersionBranch updateBranch, bool force);
    bool hasBeenInitialized();
    char* getUpdateFirmwareUrl();
    int getFirmwareBytesDownloaded();
    void setFirmwareBytesDownloaded(int bytesDownloaded);
};

extern Wesp_Update_Manager_Class Wesp_Update_Manager;

#endif // WESP_UPDATE_MANAGER_H