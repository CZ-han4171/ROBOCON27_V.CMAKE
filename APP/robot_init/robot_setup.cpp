/**
 * @file robot_setup.cpp
 * @author CZhan
 * @brief
 * @version 1.0
 * @date 2026-10-26
 *
 * @copyright
 *
 * @attention :
 * @note :
 * @versioninfo :
 */
 #include "chassis_task.h"
#include "Axis.hpp"
#include "Motor.hpp"
#include "chassis_solution.hpp"
#include "com_config.hpp"
#include "pid_controller.hpp"
#include "tool.hpp"
#include "topic_pool.h"
#include "topics.hpp"
#include "robot_devices.hpp"
#include <array>

PidParam M2006_P38_SpeedPid(3000.0f, 16600.0f, 0.0f, 10000.0f, 0.0f, IMCREATEMENT_OF_OUT);
PidParam M2006_P38_PosPid(3.0f,0.4f,0.01f,5*M_PI,0.5f*kDegToRad,NONE);
PidParam M3508_P19_SpeedPid(4000.0f, 8600.0f, 0.0f, 16000.0f, 0.0f, IMCREATEMENT_OF_OUT);
PidParam M3508_P19_PosPid(3.0f,0.4f,0.01f,2*M_PI,0.5f*kDegToRad,NONE);
PidParam U8Lite_SpeedPid(1000.0f,3000.0f,0.0f,30000.0f,0.0f,IMCREATEMENT_OF_OUT);

MotorControllerBase::PlanParam SwerveSteer_M2006_PlanParam(100.0f*M_PI,100.0f*M_PI,M_PI/18.0f,0.0f);
PidParam SwerveSteer_M2006_SpeedPid(3000.0f, 16600.0f, 0.0f, 10000.0f, 0.0f, IMCREATEMENT_OF_OUT);
PidParam SwerveSteer_M2006_PosPid(3.0f,0.8f,0.0f,25.0f*M_PI,0.05f*kDegToRad,NONE);

void RobotDevices_Init() {
    // can 总线初始化
    fdcan1_bus.init();
    fdcan2_bus.init();
    fdcan3_bus.init();

    chassis_motor1.init();
    chassis_motor2.init();
    chassis_motor3.init();
    chassis_motor4.init();

    U8_motor1.init(1.0f, 21);
    U8_motor2.init(1.0f, 21);
    U8_motor3.init(1.0f, 21);
    U8_motor4.init(1.0f, 21);
    Encoder_1.init(4096,16);
    Encoder_2.init(4096,16);
    Encoder_3.init(4096,16);
    Encoder_4.init(4096,16);
    DJI2006_motor1.init();
    DJI2006_motor2.init();
    DJI2006_motor3.init();
    DJI2006_motor4.init();

    // arm3508_motor.init();
    arm4310_motor.init();

    U8_controller1.ConfigSpeedPid(U8Lite_SpeedPid);
    U8_controller2.ConfigSpeedPid(U8Lite_SpeedPid);
    U8_controller3.ConfigSpeedPid(U8Lite_SpeedPid);
    U8_controller4.ConfigSpeedPid(U8Lite_SpeedPid);

    DJI2006_controller1.init(SwerveSteer_M2006_PlanParam, SwerveSteer_M2006_SpeedPid, SwerveSteer_M2006_PosPid, MotorControllerBase::SPEEDPLAN);
    DJI2006_controller2.init(SwerveSteer_M2006_PlanParam, SwerveSteer_M2006_SpeedPid, SwerveSteer_M2006_PosPid, MotorControllerBase::SPEEDPLAN);
    DJI2006_controller3.init(SwerveSteer_M2006_PlanParam, SwerveSteer_M2006_SpeedPid, SwerveSteer_M2006_PosPid, MotorControllerBase::SPEEDPLAN);
    DJI2006_controller4.init(SwerveSteer_M2006_PlanParam, SwerveSteer_M2006_SpeedPid, SwerveSteer_M2006_PosPid, MotorControllerBase::SPEEDPLAN);

}