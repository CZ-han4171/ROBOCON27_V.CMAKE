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
#include "com_config.hpp"
#include "pid_controller.hpp"
#include "topic_pool.h"
#include "topics.hpp"
#include <array>

// 任务句柄
osThreadId_t ChassisTaskHandle;

// 底盘电机实例声明

// 底盘控制命令订阅
static TypedTopicSubscriber<pub_chassis_cmd> chassis_cmd_sub("chassis_cmd", 8);
pub_chassis_cmd chassis_chassis_cmd{};

static inline void chassisInit() {
    if (!chassis_cmd_sub.IsValid()) {
        return;
    }
}

void chassisTask(void *argument) {
    TickType_t currentTime = xTaskGetTickCount();

    chassisInit();

    for (;;) {

        if (chassis_cmd_sub.TryGet(&chassis_chassis_cmd)) {
        }
        // 无论有没有新命令，都持续执行解算
        vTaskDelayUntil(&currentTime, 1);
    }
}
