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
#include "robot_params.hpp"
#include <array>
#include <math.h>
#include <sys/types.h>

PidParam M2006_P38_SpeedPid(3000.0f, 8000.0f, 0.0f, 10000.0f, 0.0f, IMCREATEMENT_OF_OUT);
PidParam M2006_P38_PosPid(3.0f,0.4f,0.01f,5*M_PI,0.5f*kDegToRad,NONE);
PlanParam M2006_P38_PlanParam(M_PI,M_PI,10*M_PI);

PidParam M3508_P19_SpeedPid(3300.0f, 5600.0f, 0.0f, 16000.0f, 0.0f, IMCREATEMENT_OF_OUT);
PidParam M3508_P19_PosPid(3.0f,0.4f,0.01f,2*M_PI,0.5f*kDegToRad,NONE);
PlanParam M3508_P19_PlanParam(4.0f*M_PI,4.0f*M_PI,10*M_PI);

PidParam U8Lite_SpeedPid(100.0f,200.0f,0.0f,30000.0f,0.0f,IMCREATEMENT_OF_OUT);
PidParam U8Lite_PosPid(3.0f,0.4f,0.01f,2*M_PI,0.5f*kDegToRad,NONE);
PlanParam U8Lite_PlanParam(4.0f*M_PI,4.0f*M_PI,10*M_PI);

PidParam SwerveSteer_M2006_SpeedPid(3000.0f, 16600.0f, 0.0f, 10000.0f, 0.0f, IMCREATEMENT_OF_OUT);
PidParam SwerveSteer_M2006_PosPid(3.0f,0.8f,0.0f,25.0f*M_PI,0.05f*kDegToRad,NONE);
PlanParam SwerveSteer_M2006_PlanParam(100.0f*M_PI,100.0f*M_PI, 10.0f*M_PI);

MitParam AutoHomeParam_meter(-0.25f,0.0f,0.0f,0.0f,200.0f);
MitParam AutoHomeParam_rad(-M_PI,0.0f,0.0f,0.0f,1.0f);

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
    chassis_Lift.init(Control_Improvement::POLYNOMIAL,
                    {0.5f,0.6f,1.0f},
                    M3508_P19_PosPid);
    chassis_Lift.ConfigAutoHome({0,1},
                                {{AutoHomeParam_meter,-chassis_lift_max_front_height},{AutoHomeParam_meter,-chassis_lift_max_hind_height}});
    chassis_Lift.ConfigLimit({0,1},
                             {{chassis_lift_limit_min,chassis_lift_limit_max},{chassis_lift_limit_min,chassis_lift_limit_min}});

    lift_Drive_motor1.init();
    lift_Drive_motor2.init();
    lift_Drive_controller1.init(M2006_P38_SpeedPid,M2006_P38_PosPid);
    lift_Drive_controller2.init(M2006_P38_SpeedPid,M2006_P38_PosPid);
    LiftDrive.init(Control_Improvement::NONE,M2006_P38_PlanParam, M2006_P38_PosPid);
 }