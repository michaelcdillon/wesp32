#ifndef WESP_UTILS_H
#define WESP_UTILS_H
#include "freertos/FreeRTOS.h"

#define pdTICKS_TO_MS(xTicks)    (((TickType_t) (xTicks) * 1000u) / configTICK_RATE_HZ)

extern bool WU_TakeI2cLock(TickType_t ticksToWait);
extern bool WU_ReleaseI2cLock();

#endif // WESP_UTILS_H