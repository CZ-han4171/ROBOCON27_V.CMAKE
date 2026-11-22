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
#include "topic_pool.h"
#include "topics.hpp"
#include <array>

// 任务句柄
osThreadId_t ChassisTaskHandle;

// 底盘控制命令订阅
static TypedTopicSubscriber<pub_chassis_cmd> chassis_cmd_sub("chassis_cmd", 8);
pub_chassis_cmd chassis_sub_cmd{};

static TypedTopicSubscriber<pub_chassis_lift_cmd> chassis_lift_cmd_sub("chassis_lift_cmd", 8);
pub_chassis_lift_cmd chassis_lift_sub_cmd{};

OmniChassis OmniChassis_solve{Omni_Chassis};
ChassisLift ChassisLift_solve{chassis_Lift,LiftDrive};

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
        // 无论有没有新命令，都持续执行解算
        OmniChassis_solve.run(chassis_sub_cmd);
        ChassisLift_solve.run(chassis_lift_sub_cmd);
        vTaskDelayUntil(&currentTime, 1);
    }
}
