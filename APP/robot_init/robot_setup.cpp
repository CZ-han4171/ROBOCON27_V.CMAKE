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
#include "ControlOptimize.hpp"
#include "MotorControl.hpp"
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
#include <math.h>
#include <sys/types.h>

PidParam M2006_P38_SpeedPid(3000.0f, 16600.0f, 0.0f, 10000.0f, 0.0f, IMCREATEMENT_OF_OUT);
PidParam M2006_P38_PosPid(3.0f,0.4f,0.01f,5*M_PI,0.5f*kDegToRad,NONE);
PlanParam M2006_P38_PlanParam(M_PI,M_PI,10*M_PI);

PidParam M3508_P19_SpeedPid(4000.0f, 8600.0f, 0.0f, 16000.0f, 0.0f, IMCREATEMENT_OF_OUT);
PidParam M3508_P19_PosPid(3.0f,0.4f,0.01f,2*M_PI,0.5f*kDegToRad,NONE);
PlanParam M3508_P19_PlanParam(4.0f*M_PI,4.0f*M_PI,10*M_PI);

PidParam U8Lite_SpeedPid(100.0f,200.0f,0.0f,30000.0f,0.0f,IMCREATEMENT_OF_OUT);
PidParam U8Lite_PosPid(3.0f,0.4f,0.01f,2*M_PI,0.5f*kDegToRad,NONE);
PlanParam U8Lite_PlanParam(4.0f*M_PI,4.0f*M_PI,10*M_PI);

PidParam SwerveSteer_M2006_SpeedPid(3000.0f, 16600.0f, 0.0f, 10000.0f, 0.0f, IMCREATEMENT_OF_OUT);
PidParam SwerveSteer_M2006_PosPid(3.0f,0.8f,0.0f,25.0f*M_PI,0.05f*kDegToRad,NONE);
PlanParam SwerveSteer_M2006_PlanParam(100.0f*M_PI,100.0f*M_PI, 10.0f*M_PI);

MitParam AutoHomeParam_Positive(M_PI,0.0f,0.0f,0.0f,1.0f);
MitParam AutoHomeParam_Negative(-M_PI,0.0f,0.0f,0.0f,1.0f);

void RobotDevices_Init() {
    // can 总线初始化
    fdcan1_bus.init();
    fdcan2_bus.init();
    fdcan3_bus.init();

    chassis_motor1.init();
    chassis_motor2.init();
    chassis_motor3.init();
    chassis_motor4.init();
    chassis_controller1.init(M3508_P19_SpeedPid,M3508_P19_PosPid);
    chassis_controller2.init(M3508_P19_SpeedPid,M3508_P19_PosPid);
    chassis_controller3.init(M3508_P19_SpeedPid,M3508_P19_PosPid);
    chassis_controller4.init(M3508_P19_SpeedPid,M3508_P19_PosPid);
    Omni_Chassis.init(Control_Improvement::NONE,M3508_P19_PlanParam, M3508_P19_PosPid);

    chassis_lift_motor1.init();
    chassis_lift_motor2.init();
    chassis_lift_motor3.init();
    chassis_lift_motor4.init();
    chassis_lift_controller1.init(M3508_P19_SpeedPid,M3508_P19_PosPid);
    chassis_lift_controller2.init(M3508_P19_SpeedPid,M3508_P19_PosPid);
    chassis_lift_controller3.init(M3508_P19_SpeedPid,M3508_P19_PosPid);
    chassis_lift_controller4.init(M3508_P19_SpeedPid,M3508_P19_PosPid);
    chassis_Lift.init(Control_Improvement::COSPLAN,M3508_P19_PlanParam, M3508_P19_PosPid,
                      {0,1},
                      {{0,0.33f},{0,0.33f}});

    // U8_motor1.init(1.0f, 21);
    // U8_motor2.init(1.0f, 21);
    // U8_motor3.init(1.0f, 21);
    // U8_motor4.init(1.0f, 21);
    // Encoder_1.init(4096,16);
    // Encoder_2.init(4096,16);
    // Encoder_3.init(4096,16);
    // Encoder_4.init(4096,16);
    // DJI2006_motor1.init();
    // DJI2006_motor2.init();
    // DJI2006_motor3.init();
    // DJI2006_motor4.init();

    //天空块与球的机械臂初始化
    DJI2006_motor_VPleft.init();
    DJI2006_motor_VPright.init();
    DJI3508_motor_DLleft.init();
    DJI3508_motor_DLright.init();
    DJI2006_motor_MLleft.init();
    DJI2006_motor_MLright.init();
    DJI2006_motor_EClaw.init();
    DJI2006_motor_EVP.init();
    DJI2006_motor_Claw.init();

    arm4310_motor.init();

    // U8_controller1.init(U8Lite_SpeedPid,U8Lite_PosPid);
    // U8_controller2.init(U8Lite_SpeedPid,U8Lite_PosPid);
    // U8_controller3.init(U8Lite_SpeedPid,U8Lite_PosPid);
    // U8_controller4.init(U8Lite_SpeedPid,U8Lite_PosPid);

    // DJI2006_controller1.init(SwerveSteer_M2006_SpeedPid,SwerveSteer_M2006_PosPid);
    // DJI2006_controller2.init(SwerveSteer_M2006_SpeedPid,SwerveSteer_M2006_PosPid);
    // DJI2006_controller3.init(SwerveSteer_M2006_SpeedPid,SwerveSteer_M2006_PosPid);
    // DJI2006_controller4.init(SwerveSteer_M2006_SpeedPid,SwerveSteer_M2006_PosPid);

    // SwerveDrive1.init(Control_Improvement::NONE, U8Lite_PlanParam,U8Lite_PosPid);
    // SwerveDrive2.init(Control_Improvement::NONE, U8Lite_PlanParam,U8Lite_PosPid);
    // SwerveDrive3.init(Control_Improvement::NONE, U8Lite_PlanParam,U8Lite_PosPid);
    // SwerveDrive4.init(Control_Improvement::NONE, U8Lite_PlanParam,U8Lite_PosPid);
    // SwerveSteer1.init(Control_Improvement::SPEEDPLAN, SwerveSteer_M2006_PlanParam, SwerveSteer_M2006_PosPid);
    // SwerveSteer2.init(Control_Improvement::SPEEDPLAN, SwerveSteer_M2006_PlanParam, SwerveSteer_M2006_PosPid);
    // SwerveSteer3.init(Control_Improvement::SPEEDPLAN, SwerveSteer_M2006_PlanParam, SwerveSteer_M2006_PosPid);
    // SwerveSteer4.init(Control_Improvement::SPEEDPLAN, SwerveSteer_M2006_PlanParam, SwerveSteer_M2006_PosPid);

    DJI2006_motor_VPleft_controller.init(M2006_P38_PlanParam, M2006_P38_SpeedPid, M2006_P38_PosPid,ControlOptimizer<1>::Control_Improvement::COSPLAN);
    DJI2006_motor_VPright_controller.init(M2006_P38_PlanParam, M2006_P38_SpeedPid, M2006_P38_PosPid,ControlOptimizer<1>::Control_Improvement::COSPLAN);
    DJI3508_motor_DLleft_controller.init(M3508_P19_PlanParam, M3508_P19_SpeedPid, M3508_P19_PosPid, ControlOptimizer<1>::Control_Improvement::COSPLAN);
    DJI3508_motor_DLright_controller.init(M3508_P19_PlanParam, M3508_P19_SpeedPid, M3508_P19_PosPid, ControlOptimizer<1>::Control_Improvement::COSPLAN);
    DJI2006_motor_MLleft_controller.init(M2006_P38_PlanParam, M2006_P38_SpeedPid, M2006_P38_PosPid,ControlOptimizer<1>::Control_Improvement::COSPLAN);
    DJI2006_motor_MLright_controller.init(M2006_P38_PlanParam, M2006_P38_SpeedPid, M2006_P38_PosPid,ControlOptimizer<1>::Control_Improvement::COSPLAN);
    DJI2006_motor_EVP_controller.init(M2006_P38_PlanParam, M2006_P38_SpeedPid, M2006_P38_PosPid,ControlOptimizer<1>::Control_Improvement::COSPLAN);
    DJI2006_motor_EClaw_controller.init(M2006_P38_PlanParam, M2006_P38_SpeedPid, M2006_P38_PosPid,ControlOptimizer<1>::Control_Improvement::COSPLAN);
    DJI2006_motor_Claw_controller.init(M2006_P38_PlanParam, M2006_P38_SpeedPid, M2006_P38_PosPid,ControlOptimizer<1>::Control_Improvement::COSPLAN);
}