/**
 * @file chassis_task.cpp
 * @author 大帅将军
 * @brief 底盘任务实现
 * @version 0.1
 * @date 2026-04-21
 *
 * @copyright Copyright (c) 2026
 *
 * @attention :
 * @note :
 * @versioninfo :
 */

#include "chassis_task.h"
#include "Axis.hpp"
#include "Encoder.hpp"
#include "Motor.hpp"
#include "chassis_solution.hpp"
#include "chassis_lift_solution.hpp"
#include "com_config.hpp"
#include "robot_devices.hpp"
#include "pid_controller.hpp"
#include "topic_pool.hpp"
#include "topics.hpp"
#include <array>

// 任务句柄
osThreadId_t ChassisTaskHandle;
osThreadId_t ChassisDataTaskHandle;

// 底盘控制命令订阅
static TypedTopicSubscriber<pub_chassis_cmd> chassis_cmd_sub("chassis_cmd", 8);
pub_chassis_cmd chassis_sub_cmd{};
pub_chassis_cmd chassis_body_cmd{};

static TypedTopicSubscriber<pub_chassis_lift_cmd> chassis_lift_cmd_sub("chassis_lift_cmd", 8);
pub_chassis_lift_cmd chassis_lift_sub_cmd{};
// static TypedTopicSubscriber<pub_chassis_lift_cmd> chassis_lift_cmd_sub("chassis_lift_cmd", 8);
// pub_chassis_lift_cmd chassis_lift_sub_cmd{};

TypedTopicSubscriber<pub_imu_data> chassis_imu_data_sub("imu_data",8);
pub_imu_data chassis_imu_sub_msg{};

TypedTopicSubscriber<pub_RM_Data> chassis_rm_sub("rm", 8);
pub_RM_Data chassis_rm_cmd{};
pub_RM_Data chassis_rm_last_cmd{};

TypedTopicPublisher<Chassis_data> chassis_data_pub("chassis_data");
Chassis_data chassis_data{};

std::array<OmniChassis<4>::Geometry, 4> OmniChassisParam{
    {{0.129f,0.35f,0.35f, M_PI*3.0f/4.0f},
    {0.129f, 0.35f, -0.35f, M_PI/2.0f},
    {0.129f,-0.35f, 0.35f,-M_PI/2.0f},
    {0.129f,-0.35f,-0.35f,-M_PI*3.0f/4.0f}}
};

bool imu_init = false;
float chassis_yaw_rad{0};
float imu_zero{0};
OmniChassis OmniChassis_solve{Omni_Chassis,chassis_yaw_rad,OmniChassisParam};
ChassisLift ChassisLift_solve{chassis_Lift,LiftDrive,chassis_yaw_rad};
LiftState Lift_state_traget{};
LiftState Lift_state{};
LiftState lift_state_last{};

static inline void chassisInit() {
    if (!chassis_cmd_sub.IsValid()) {
        return;
    }
}

void chassisTask(void *argument) {
    TickType_t currentTime = xTaskGetTickCount();

    chassisInit();

    for (;;) {

        if (chassis_cmd_sub.TryGet(&chassis_sub_cmd)) {}
        if (chassis_lift_cmd_sub.TryGet(&chassis_lift_sub_cmd)) {}
        if(chassis_imu_data_sub.TryGet(&chassis_imu_sub_msg)){
            if(!imu_init){
                imu_zero = chassis_imu_sub_msg.yaw_rad;
                imu_init = true;
            }
            chassis_yaw_rad = chassis_imu_sub_msg.yaw_rad - imu_zero;
        }

        // 无论有没有新命令，都持续执行解算
        OmniChassis_solve.run(chassis_sub_cmd);
        ChassisLift_solve.run(chassis_lift_sub_cmd);
        vTaskDelayUntil(&currentTime, 1);
    }
}

void ChassisDataTask(void *argument) {
    TickType_t currentTime = xTaskGetTickCount();

    for(;;){
        OmniChassis_solve.SendChassisData(chassis_data);
        ChassisLift_solve.SendChassisData(chassis_data);

        if(Lift_state != LiftState::Running)lift_state_last = Lift_state;
        Lift_state = ChassisLift_solve.getLiftState();
        Lift_state_traget = ChassisLift_solve.getAimLiftState();

        if(Lift_state == LiftState::Collapse || Lift_state == LiftState::Crouh){
            chassis_data = OmniChassis_solve.getChassisData();
        }else if(Lift_state == LiftState::FrontHold || Lift_state == LiftState::HindHold || Lift_state == LiftState::HoldUp){
            chassis_data = ChassisLift_solve.getChassisData();
        }else if(Lift_state == LiftState::Running){
            if(lift_state_last == LiftState::Crouh){
                if(Lift_state_traget == LiftState::HoldUp){
                    chassis_data = ChassisLift_solve.getChassisData();
                }else {
                    chassis_data = OmniChassis_solve.getChassisData();
                }
            }else if(lift_state_last == LiftState::FrontHold || lift_state_last == LiftState::HindHold){
                if(Lift_state_traget == LiftState::Crouh){
                    chassis_data = OmniChassis_solve.getChassisData();
                }else if(Lift_state_traget == LiftState::HoldUp){
                    chassis_data = ChassisLift_solve.getChassisData();
                }
            }else if(lift_state_last == LiftState::HoldUp){
                chassis_data = ChassisLift_solve.getChassisData();
            }else if(lift_state_last == LiftState::Collapse){
                chassis_data = OmniChassis_solve.getChassisData();
            }
        }

        chassis_data_pub.Publish(chassis_data);

        vTaskDelayUntil(&currentTime, 1);
    }
}
