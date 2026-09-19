/**
 * @file control_task.cpp
 * @author 大帅将军 / lxlx
 * @brief 控制任务 —— Xbox按键 → 底盘 + 上身控制指令决策层
 * @version 0.3
 * @date 2026-05-28
 *
 * @copyright Copyright (c) 2026
 *
 * @attention btnSelect 切换模式：0=队友模式(现有逻辑), 1=调试模式(上身按键映射)
 * @note 持续型按键每帧累加步进量；切换型按键仅上升沿触发
 * @versioninfo v0.3: 模块化拆分 —— 流程函数移至 control_process，动作函数移至 control_action
 * @versioninfo v0.2: 新增上身控制指令 + btnSelect模式切换
 */

#include "main.h"
#include "control_task.h"
#include "pid_controller.h"
#include "chassis_task.h"
#include "rm_pocket.hpp"
#include "topic_pool.h"
#include "topics.hpp"
#include "bsp_usart.h"
#include "tracking.h"
#include <cmath>
#include <cstdint>
#include "control_Traject.hpp"

osThreadId_t ControlTaskHandle;

// ===== 发布者 =====
TypedTopicPublisher<pub_chassis_cmd> chassis_data_pub("chassis_cmd");
pub_chassis_cmd rm_cmd{};

// static TypedTopicPublisher<QR_code_cmd_t> qr_code_cmd_pub("qr_code_cmd");   
// static QR_code_cmd_t qr_code_cmd{};

TypedTopicSubscriber<pub_RM_Data> control_rm_sub("rm", 8);
pub_RM_Data control_rm_cmd{};
pub_RM_Data control_rm_cmd_last{};

TypedTopicSubscriber<pub_Position_Data> control_position_sub("position", 8);
pub_Position_Data control_position_msg{};
pub_Position_Data control_position{};

// TypedTopicSubscriber<pub_ir_data> control_ir_sub("ir_data", 8);
// pub_ir_data control_ir_msg{};

// TypedTopicSubscriber<QR_code_data_t> qr_code_data_sub("qr_code_data", 8);
// QR_code_data_t control_qr_code_data{};

pub_chassis_cmd robot_v_aim_cmd{};
pub_chassis_cmd state_now_cmd{};

void controlInit() {
    if (!chassis_data_pub.IsValid()) {
        return;
    }

    if (!control_rm_sub.IsValid()) {
        return;
    }

    if (!control_position_sub.IsValid()) {
        return;
    }
    // if (!control_ir_sub.IsValid()) {
    //     return;
    // }
    // if(!qr_code_cmd_pub.IsValid()) {
    //     return;
    // }
    // if(!qr_code_data_sub.IsValid()) {
    //     return;
    // }
}


float rm_angle_deg;
float v_aim;

void controlTask(void *argument) {
    TickType_t currentTime = xTaskGetTickCount();
    TickType_t rcUpdateTime = currentTime;
    constexpr TickType_t kRemoteInputTimeout = pdMS_TO_TICKS(500);


    controlInit();
    // uint32_t last_time = HAL_GetTick();

    for (;;) {
        // static float controlTask_dt = 0.0f;
        // static uint32_t controlTask_DWT_CNT = 0;
        // controlTask_dt = DWT_GetDeltaT(&controlTask_DWT_CNT);
        //test begin
        const TickType_t now = xTaskGetTickCount();
        if (control_rm_sub.TryGet(&control_rm_cmd)) {
            rcUpdateTime = now;
        } else if (now - rcUpdateTime > kRemoteInputTimeout) {
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

            rm_angle_deg = atan2(rm_cmd.linear_y_, rm_cmd.linear_x_) / kDegToRad;
            v_aim = sqrt(rm_cmd.linear_x_ * rm_cmd.linear_x_ + rm_cmd.linear_y_ * rm_cmd.linear_y_);
            robot_v_aim_cmd.linear_x_ = v_aim * cos((rm_angle_deg - state_now_cmd.omega_) * kDegToRad);
            robot_v_aim_cmd.linear_y_ = v_aim * sin((rm_angle_deg - state_now_cmd.omega_) * kDegToRad);
            robot_v_aim_cmd.omega_ = rm_cmd.omega_;    
        }

        chassis_data_pub.Publish(robot_v_aim_cmd);

        vTaskDelayUntil(&currentTime, 5);
    }
}
