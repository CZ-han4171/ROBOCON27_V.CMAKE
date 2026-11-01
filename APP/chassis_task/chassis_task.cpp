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
extern C620Motor chassis_motor1, chassis_motor2, chassis_motor3, chassis_motor4;
extern RotaryAxis SwerveDrive1, SwerveDrive2, SwerveDrive3, SwerveDrive4;
extern RotaryAxis SwerveSteer1, SwerveSteer2, SwerveSteer3, SwerveSteer4;
extern BRT38B Encoder_1, Encoder_2, Encoder_3, Encoder_4;

// 底盘控制命令订阅
static TypedTopicSubscriber<pub_chassis_cmd> chassis_cmd_sub("chassis_cmd", 8);
pub_chassis_cmd chassis_chassis_cmd{};

SwerveChassis4 SwerveChassis4_solver(SwerveDrive1, SwerveDrive2, SwerveDrive3, SwerveDrive4,
                                     SwerveSteer1, SwerveSteer2, SwerveSteer3, SwerveSteer4,
                                     Encoder_1, Encoder_2, Encoder_3, Encoder_4);
namespace {//舵轮底盘参数传入
    const std::array<SwerveChassis4::Geometry, SwerveChassis4::kWheelCount>
        kSwerveChassis4Geometries = {
            SwerveChassis4::Geometry(0.08f,{ 0.1915f , 0.208f}),
            SwerveChassis4::Geometry(0.08f,{ 0.1915f , -0.208f}),
            SwerveChassis4::Geometry(0.08f,{ -0.1915f, -0.208f}),
            SwerveChassis4::Geometry(0.08f,{ -0.1915f, 0.208f}),
        };
}

static inline void chassisInit() {
    SwerveChassis4_solver.configureGeometries(kSwerveChassis4Geometries);
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
        SwerveChassis4_solver.run(chassis_chassis_cmd);
        vTaskDelayUntil(&currentTime, 1);
    }
}
