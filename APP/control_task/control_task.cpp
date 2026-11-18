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
TypedTopicPublisher<pub_chassis_cmd> chassis_data_pub("chassis_cmd");
pub_chassis_cmd rm_cmd{};

TypedTopicSubscriber<pub_RM_Data> control_rm_sub("rm", 8);
pub_RM_Data control_rm_cmd{};
pub_RM_Data control_rm_cmd_last{};

pub_chassis_cmd robot_v_aim_cmd{};
pub_chassis_cmd state_now_cmd{};

//debug begin
float_t debug_dt;
uint32_t debug_DWT_CNT;
float watch[10];
//debug end

void controlInit() {
    if (!chassis_data_pub.IsValid()) {
        return;
    }

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
            rm_cmd.linear_x_ = JoyToVelocity(control_rm_cmd.joyLHori, kJoyDeadZoneLeft, MAX_VELOCITY_LINEAR);
            rm_cmd.linear_y_ = JoyToVelocity(control_rm_cmd.joyLVert, kJoyDeadZoneLeft, MAX_VELOCITY_LINEAR);
            rm_cmd.omega_    = -JoyToVelocity(control_rm_cmd.joyRHori, kJoyDeadZoneRight, MAX_VELOCITY_ANGULAR);

            float rm_angle_deg = atan2(rm_cmd.linear_y_, rm_cmd.linear_x_) / kDegToRad;
            float v_aim = sqrt(rm_cmd.linear_x_ * rm_cmd.linear_x_ + rm_cmd.linear_y_ * rm_cmd.linear_y_);
            robot_v_aim_cmd.linear_x_ = v_aim * cos((rm_angle_deg - state_now_cmd.omega_) * kDegToRad);
            robot_v_aim_cmd.linear_y_ = v_aim * sin((rm_angle_deg - state_now_cmd.omega_) * kDegToRad);
            robot_v_aim_cmd.omega_ = rm_cmd.omega_;    
        }

        {   
            /////////////////debug begin///////////////////////
            
            /////////////////debug end///////////////////////
        }

        chassis_data_pub.Publish(robot_v_aim_cmd);

        vTaskDelayUntil(&currentTime, 1);
    }
}
