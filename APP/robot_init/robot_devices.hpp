/**
 * @file robot_devices.hpp
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
#include "main.h"
#include "Motor.hpp"
#include "MotorControl.hpp"
#include "Axis.hpp"
#include "Encoder.hpp"


extern C620Motor chassis_motor1; 
extern C620Motor chassis_motor2;
extern C620Motor chassis_motor3;
extern C620Motor chassis_motor4;

extern DJIMotorController chassis_controller1;
extern DJIMotorController chassis_controller2;
extern DJIMotorController chassis_controller3;
extern DJIMotorController chassis_controller4;

extern GeneralAxis<4,4> Omni_Chassis;


extern C620Motor chassis_lift_motor1; 
extern C620Motor chassis_lift_motor2;
extern C620Motor chassis_lift_motor3;
extern C620Motor chassis_lift_motor4;
extern DJIMotorController chassis_lift_controller1;
extern DJIMotorController chassis_lift_controller2;
extern DJIMotorController chassis_lift_controller3;
extern DJIMotorController chassis_lift_controller4;

extern GeneralAxis<2,4> chassis_Lift;


extern C610Motor lift_Drive_motor1;
extern C610Motor lift_Drive_motor2;
extern DJIMotorController lift_Drive_controller1;
extern DJIMotorController lift_Drive_controller2;

extern GeneralAxis<1,2> LiftDrive;
