#include "upper_solutuion.hpp"
#include "upper_task.hpp"
#include "robot_devices.hpp"

osThreadId_t VPupperTaskHandle;

volatile float VPleftMotorSumPos_rad = 0.0f;
volatile float VPrightMotorSumPos_rad = 0.0f;

static TypedTopicSubscriber<pub_VPupper_cmd> vp_sub("VPupper_cmd", 8);
static UpperVP vp(VPLeft, VPRight);

void VPupperTask(void *argument) {
    pub_VPupper_cmd cmd{};
    TickType_t tick = xTaskGetTickCount();
    for (;;) {
        VPleftMotorSumPos_rad = DJI2006_motor_VPleft_controller.getMotorSumPos_rad();
        VPrightMotorSumPos_rad = DJI2006_motor_VPright_controller.getMotorSumPos_rad();
        vp_sub.TryGet(&cmd);          
        vp.run(cmd);
        vTaskDelayUntil(&tick, 1);
    }
}
