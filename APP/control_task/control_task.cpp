/**
 * @file control_task.cpp
 * @author 大帅将军 / lxlx
 * @brief
 * @version 0.3
 * @date 2026-05-28
 *
 * @copyright Copyright (c) 2026
 *
 * @attention
 * @note 
 */

#include "bsp_dwt.h"
#include "chassis_solution.hpp"
#include "main.h"
#include "control_task.h"
#include "pid_controller.hpp"
#include "chassis_task.h"
#include "rm_pocket.hpp"
#include "stm32h723xx.h"
#include "tool.hpp"
#include "topic_pool.h"
#include "topics.hpp"
#include "bsp_usart.h"
#include "tracking.h"
#include <cmath>
#include <cstdint>
#include <math.h>
#include "Motor.hpp"
#include "MotorControl.hpp"
#include "Axis.hpp"
#include "Encoder.hpp"
#include "robot_devices.hpp"
#include <algorithm>

osThreadId_t ControlTaskHandle;

// ===== 发布者 =====
TypedTopicSubscriber<pub_RM_Data> control_rm_sub("rm", 8);
pub_RM_Data control_rm_cmd{};
pub_RM_Data control_rm_cmd_last{};
//debug begin
float_t debug_dt;
uint32_t debug_DWT_CNT;
float watch[10];
//debug end

void controlInit() {
    if (!control_rm_sub.IsValid()) {
        return;
    }

}

void controlTask(void *argument) {
    TickType_t currentTime = xTaskGetTickCount();
    TickType_t rmUpdateTime = currentTime;
    constexpr TickType_t kRemoteInputTimeout = pdMS_TO_TICKS(500);

    control_rm_cmd.joyLHori = kJoyCenter;
    control_rm_cmd.joyLVert = kJoyCenter;
    control_rm_cmd.joyRHori = kJoyCenter;
    control_rm_cmd.joyRVert = kJoyCenter;
    
    controlInit();

    for (;;) {

        const TickType_t now = xTaskGetTickCount();
        if (control_rm_sub.TryGet(&control_rm_cmd)) {
            rmUpdateTime = now;
        } else if (now - rmUpdateTime > kRemoteInputTimeout) {
            control_rm_cmd.joyLHori = kJoyCenter;
            control_rm_cmd.joyLVert = kJoyCenter;
            control_rm_cmd.joyRHori = kJoyCenter;
            control_rm_cmd.joyRVert = kJoyCenter;
        }
        control_rm_cmd_last = control_rm_cmd;

        {  
            /////////////////debug begin///////////////////////
            
            /////////////////debug end///////////////////////
        }

        vTaskDelayUntil(&currentTime, 1);
    }
}
