#include "DDPanTilt_solution.hpp"
#include "upper_task.hpp"
#include "robot_devices.hpp"

osThreadId_t VPTaskHandle;

#define kupperTaskPeriod 1.0f

volatile float VPleftMotorSumPos_rad = 0.0f;
volatile float VPrightMotorSumPos_rad = 0.0f;

static TypedTopicSubscriber<pub_VP_cmd> vp_sub("VP_cmd", 8);
static DD_PanTilt dd_pantilt(VPLeft, VPRight);

void VPTask(void *argument) {
    pub_VP_cmd cmd{};
    TickType_t tick = xTaskGetTickCount();
    for (;;) {
        vp_sub.TryGet(&cmd);          
        dd_pantilt.run(cmd);
        vTaskDelayUntil(&tick, kupperTaskPeriod);
    }
}
