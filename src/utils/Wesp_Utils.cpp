#include "Wesp_Utils.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"


static SemaphoreHandle_t I2C_LOCK_HANDLE = NULL;

bool WU_TakeI2cLock(TickType_t ticksToWait) {
    if (I2C_LOCK_HANDLE == NULL) {
        I2C_LOCK_HANDLE = xSemaphoreCreateMutex();
    }

    return xSemaphoreTake(I2C_LOCK_HANDLE, ticksToWait);
}

bool WU_ReleaseI2cLock() {
    if (I2C_LOCK_HANDLE == NULL) {
        I2C_LOCK_HANDLE = xSemaphoreCreateMutex();
    }

    return xSemaphoreGive(I2C_LOCK_HANDLE);
}