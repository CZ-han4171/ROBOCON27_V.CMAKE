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

#include "ControlOptimize.hpp"
#include "bsp_dwt.h"
#include "chassis_lift_solution.hpp"
#include "chassis_solution.hpp"
#include "main.h"
#include "control_task.h"
#include "robot_params.hpp"
#include "pid_controller.hpp"
#include "chassis_task.h"
#include "rm_pocket.hpp"
#include "stm32h723xx.h"
#include "stm32h7xx_hal_gpio.h"
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

TypedTopicSubscriber<pub_imu_data> imu_data_sub("imu_data",8);
pub_imu_data imu_sub_msg{};

extern ChassisLift ChassisLift_solve;

TypedTopicSubscriber<pub_RM_Data> control_rm_sub("rm", 8);
pub_RM_Data control_rm_cmd{};
pub_RM_Data control_rm_last_cmd{};

pub_chassis_cmd state_now_cmd{};

//debug begin
ControlOptimizer<1> chassis_omega_optimizer;
float imu_zero = 0.0f;
bool imu_init = false;
PidParam track_omega{5.30f,0.3f,0.55f,MAX_VELOCITY_ANGULAR*0.75,0.5f*kDegToRad,Integral_Limit};
Chassis_data chassis_data;
LiftState state_tmp;
bool GPIOF9 = 0;
bool GPIOD15 = 0;
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

    chassis_omega_optimizer.init(Control_Improvement::SPEEDPLAN, 
                                 {M_PI,M_PI,M_PI},
                                 track_omega);

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

        if(imu_data_sub.TryGet(&imu_sub_msg)){
            if(!imu_init){
                imu_zero = imu_sub_msg.yaw_rad;
                imu_init = true;
            }
            chassis_data.yaw = imu_sub_msg.yaw_rad - imu_zero;
        }

        {
            GPIOF9 = HAL_GPIO_ReadPin(GPIOF, GPIO_PIN_9) == GPIO_PIN_SET;
            GPIOD15 = HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_15) == GPIO_PIN_SET;     
        }

        {
            if(control_rm_cmd.swD == RM_2_POS_SW_State_t::UP){
                chassis_raw_cmd.linear_x_ = JoyToVelocity(control_rm_cmd.joyLHori, kJoyDeadZoneLeft, MAX_VELOCITY_LINEAR);
                chassis_raw_cmd.linear_y_ = JoyToVelocity(control_rm_cmd.joyLVert, kJoyDeadZoneLeft, MAX_VELOCITY_LINEAR);
                chassis_raw_cmd.omega_    = -JoyToVelocity(control_rm_cmd.joyRHori, kJoyDeadZoneRight, MAX_VELOCITY_ANGULAR);

                float rm_angle_rad = atan2(chassis_raw_cmd.linear_y_, chassis_raw_cmd.linear_x_);
                float v_aim = sqrt(chassis_raw_cmd.linear_x_ * chassis_raw_cmd.linear_x_ + chassis_raw_cmd.linear_y_ * chassis_raw_cmd.linear_y_);
                chassis_pub_cmd.linear_x_ = v_aim * cos(rm_angle_rad - chassis_data.yaw);
                chassis_pub_cmd.linear_y_ = v_aim * sin(rm_angle_rad - chassis_data.yaw);
                chassis_pub_cmd.omega_ = chassis_raw_cmd.omega_;    
            }else if(control_rm_cmd.swD == RM_2_POS_SW_State_t::DOWN){
                chassis_pub_cmd.linear_x_ = 0.0f;
                chassis_pub_cmd.linear_y_ = JoyToVelocity(control_rm_cmd.joyLVert, kJoyDeadZoneLeft, MAX_VELOCITY_STAIR);
                chassis_omega_optimizer.Run({0.0f}, {chassis_data.yaw});
                chassis_pub_cmd.omega_ = chassis_omega_optimizer.GetSpeedOutput()[0];
                
                // float yaw_in = chassis_data.yaw - 
                // chassis_pub_cmd.omega_ = 0.0f;    
            }
        }

        {
            state_tmp =  ChassisLift_solve.getLiftState();
            if(control_rm_cmd.swD == RM_2_POS_SW_State_t::UP){
                chassis_lift_pub_cmd.speed_x_ = 0.0f;
                chassis_lift_pub_cmd.state_ = LiftState::Collapse;
                chassis_lift_pub_cmd.T = 1.0f;
            }else{
                chassis_lift_pub_cmd.speed_x_ = chassis_pub_cmd.linear_y_;
                if(control_rm_last_cmd.swD == RM_2_POS_SW_State_t::UP){
                    chassis_lift_pub_cmd.state_ = LiftState::Crouh;
                        chassis_lift_pub_cmd.T = 0.5f;
                }else if(control_rm_cmd.swC != control_rm_last_cmd.swC){
                    if(control_rm_last_cmd.swC == RM_3_POS_SW_State_t::MIDDLE){
                        if(control_rm_cmd.swC == RM_3_POS_SW_State_t::UP){
                            chassis_lift_pub_cmd.state_ = LiftState::Crouh;
                            chassis_lift_pub_cmd.T = 1.0f;
                        }else if(control_rm_cmd.swC == RM_3_POS_SW_State_t::DOWN){
                            chassis_lift_pub_cmd.state_ = LiftState::HoldUp;
                            chassis_lift_pub_cmd.T = 1.0f;
                        }
                    }
                }else if(state_tmp == LiftState::HoldUp){
                    if(!GPIOF9){
                        chassis_lift_pub_cmd.state_ = LiftState::HindHold;
                        chassis_lift_pub_cmd.T = 0.35f;
                    }
                }else if(state_tmp == LiftState::HindHold){
                    if(!GPIOD15){
                        chassis_lift_pub_cmd.state_ = LiftState::Crouh;
                        chassis_lift_pub_cmd.T = 0.35f;
                    }
                }
            }
        }

        {
            
            // if(control_rm_cmd.swD == RM_2_POS_SW_State_t::UP){
            //     chassis_lift_pub_cmd.speed_x_ = 0.0f;
            //     chassis_lift_pub_cmd.height_front_ = chassis_lift_front_UP;
            //     chassis_lift_pub_cmd.height_hind_ = chassis_lift_hind_UP;                
            // }else if(control_rm_cmd.swD == RM_2_POS_SW_State_t::DOWN){
            //     chassis_lift_pub_cmd.speed_x_ = chassis_pub_cmd.linear_y_;
            //     if(control_rm_last_cmd.swD == RM_2_POS_SW_State_t::UP){
            //         chassis_lift_pub_cmd.height_front_ = chassis_lift_MIDDLE;
            //         chassis_lift_pub_cmd.height_hind_ = chassis_lift_MIDDLE;
            //     }else {
            //         if(control_rm_cmd.swC != control_rm_last_cmd.swC
            //         && control_rm_last_cmd.swC == RM_3_POS_SW_State_t::MIDDLE){
            //             if(control_rm_cmd.swC == RM_3_POS_SW_State_t::UP){
            //                 chassis_lift_pub_cmd.height_front_ = chassis_lift_MIDDLE;
            //                 chassis_lift_pub_cmd.height_hind_ = chassis_lift_MIDDLE;
            //             }else if(control_rm_cmd.swC == RM_3_POS_SW_State_t::DOWN){
            //                 chassis_lift_pub_cmd.height_front_ = chassis_lift_DOWN;
            //                 chassis_lift_pub_cmd.height_hind_ = chassis_lift_DOWN;
            //             }
            //         }else if(control_rm_cmd.swB != control_rm_last_cmd.swB 
            //                 && control_rm_last_cmd.swB == RM_3_POS_SW_State_t::MIDDLE){
            //             if(control_rm_cmd.swB == RM_3_POS_SW_State_t::UP){
            //                 if(chassis_lift_pub_cmd.height_front_ == chassis_lift_MIDDLE)chassis_lift_pub_cmd.height_front_ = chassis_lift_DOWN;
            //                 else if(chassis_lift_pub_cmd.height_front_ == chassis_lift_DOWN)chassis_lift_pub_cmd.height_front_ = chassis_lift_MIDDLE;
            //             }else if(control_rm_cmd.swB == RM_3_POS_SW_State_t::DOWN){
            //                 if(chassis_lift_pub_cmd.height_hind_ == chassis_lift_MIDDLE)chassis_lift_pub_cmd.height_hind_ = chassis_lift_DOWN;
            //                 else if(chassis_lift_pub_cmd.height_hind_ == chassis_lift_DOWN)chassis_lift_pub_cmd.height_hind_ = chassis_lift_MIDDLE;
            //             }
            //         }    
            //     }
            // }
            
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
