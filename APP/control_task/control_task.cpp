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
#include "robot_params.hpp"
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
pub_chassis_cmd chassis_pub_cmd{};
pub_chassis_cmd chassis_raw_cmd{};

TypedTopicPublisher<pub_chassis_lift_cmd> chassis_lift_data_pub("chassis_lift_cmd");
pub_chassis_lift_cmd chassis_lift_pub_cmd{};
pub_chassis_lift_cmd chassis_lift_raw_cmd{};

TypedTopicSubscriber<pub_RM_Data> control_rm_sub("rm", 8);
pub_RM_Data control_rm_cmd{};
pub_RM_Data control_rm_last_cmd{};

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

        control_rm_last_cmd = control_rm_cmd;
        if (control_rm_sub.TryGet(&control_rm_cmd)) {
            rmUpdateTime = now;
        } else if (now - rmUpdateTime > kRemoteInputTimeout) {
            control_rm_cmd.joyLHori = kJoyCenter;
            control_rm_cmd.joyLVert = kJoyCenter;
            control_rm_cmd.joyRHori = kJoyCenter;
            control_rm_cmd.joyRVert = kJoyCenter;
        }

        {
            if(control_rm_cmd.swD == RM_2_POS_SW_State_t::UP){
                chassis_raw_cmd.linear_x_ = JoyToVelocity(control_rm_cmd.joyLHori, kJoyDeadZoneLeft, MAX_VELOCITY_LINEAR);
                chassis_raw_cmd.linear_y_ = JoyToVelocity(control_rm_cmd.joyLVert, kJoyDeadZoneLeft, MAX_VELOCITY_LINEAR);
                chassis_raw_cmd.omega_    = -JoyToVelocity(control_rm_cmd.joyRHori, kJoyDeadZoneRight, MAX_VELOCITY_ANGULAR);

                float rm_angle_deg = atan2(chassis_raw_cmd.linear_y_, chassis_raw_cmd.linear_x_) / kDegToRad;
                float v_aim = sqrt(chassis_raw_cmd.linear_x_ * chassis_raw_cmd.linear_x_ + chassis_raw_cmd.linear_y_ * chassis_raw_cmd.linear_y_);
                chassis_pub_cmd.linear_x_ = v_aim * cos((rm_angle_deg - state_now_cmd.omega_) * kDegToRad);
                chassis_pub_cmd.linear_y_ = v_aim * sin((rm_angle_deg - state_now_cmd.omega_) * kDegToRad);
                chassis_pub_cmd.omega_ = chassis_raw_cmd.omega_;    
            }else if(control_rm_cmd.swD == RM_2_POS_SW_State_t::DOWN){
                chassis_pub_cmd.linear_x_ = 0.0f;
                chassis_pub_cmd.linear_y_ = JoyToVelocity(control_rm_cmd.joyLVert, kJoyDeadZoneLeft, MAX_VELOCITY_LINEAR);
                chassis_pub_cmd.omega_ = 0.0f;    
            }
        }

        {
            chassis_lift_pub_cmd.speed_x_ = chassis_pub_cmd.linear_y_;
            if(control_rm_cmd.swD == RM_2_POS_SW_State_t::UP){
                chassis_lift_pub_cmd.height_front_ = chassis_lift_UP;
                chassis_lift_pub_cmd.height_hind_ = chassis_lift_UP;                
            }else if(control_rm_cmd.swD == RM_2_POS_SW_State_t::DOWN){
                if(control_rm_last_cmd.swD == RM_2_POS_SW_State_t::UP){
                    chassis_lift_pub_cmd.height_front_ = chassis_lift_MIDDLE;
                    chassis_lift_pub_cmd.height_hind_ = chassis_lift_MIDDLE;
                }else {
                    if(control_rm_cmd.swC != control_rm_last_cmd.swC
                    && control_rm_last_cmd.swC == RM_3_POS_SW_State_t::MIDDLE){
                        if(control_rm_cmd.swC == RM_3_POS_SW_State_t::UP){
                            chassis_lift_pub_cmd.height_front_ = chassis_lift_MIDDLE;
                            chassis_lift_pub_cmd.height_hind_ = chassis_lift_MIDDLE;
                        }else if(control_rm_cmd.swC == RM_3_POS_SW_State_t::DOWN){
                            chassis_lift_pub_cmd.height_front_ = chassis_lift_DOWN;
                            chassis_lift_pub_cmd.height_hind_ = chassis_lift_DOWN;
                        }
                    }else if(control_rm_cmd.swB != control_rm_last_cmd.swB 
                            && control_rm_last_cmd.swB == RM_3_POS_SW_State_t::MIDDLE){
                        if(control_rm_cmd.swB == RM_3_POS_SW_State_t::UP){
                            if(chassis_lift_pub_cmd.height_front_ == chassis_lift_MIDDLE)chassis_lift_pub_cmd.height_front_ = chassis_lift_DOWN;
                            else if(chassis_lift_pub_cmd.height_front_ == chassis_lift_DOWN)chassis_lift_pub_cmd.height_front_ = chassis_lift_MIDDLE;
                        }else if(control_rm_cmd.swB == RM_3_POS_SW_State_t::DOWN){
                            if(chassis_lift_pub_cmd.height_hind_ == chassis_lift_MIDDLE)chassis_lift_pub_cmd.height_hind_ = chassis_lift_DOWN;
                            else if(chassis_lift_pub_cmd.height_hind_ == chassis_lift_DOWN)chassis_lift_pub_cmd.height_hind_ = chassis_lift_MIDDLE;
                        }
                    }    
                }
                
            }
            
        }

        {   
            /////////////////debug begin///////////////////////
            // chassis_lift_controller3.MotorMitCtrl({M_PI,0.0f,0.0f,0.0f,1.0f});
            /////////////////debug end///////////////////////
        }

        chassis_lift_data_pub.Publish(chassis_lift_pub_cmd);
        chassis_data_pub.Publish(chassis_pub_cmd);

        vTaskDelayUntil(&currentTime, 1);
    }
}
