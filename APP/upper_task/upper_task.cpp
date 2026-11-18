#include "upper_task.hpp"

osThreadId_t VPTaskHandle;

#define kupperTaskPeriod 1.0f

volatile float VPleftMotorSumPos_rad = 0.0f;
volatile float VPrightMotorSumPos_rad = 0.0f;

void VPTask(void *argument) {
    TickType_t currentTime = xTaskGetTickCount();
    for (;;) {
        vTaskDelayUntil(&currentTime, 1);
    }
}
