#ifndef WESP_UTILS_H
#define WESP_UTILS_H
#include "freertos/FreeRTOS.h"

extern bool WU_TakeI2cLock(TickType_t ticksToWait);
extern bool WU_ReleaseI2cLock();

#endif // WESP_UTILS_H