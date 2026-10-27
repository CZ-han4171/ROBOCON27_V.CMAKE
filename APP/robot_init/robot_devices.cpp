/**
 * @file robot_devices.cpp
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
#include "tool.hpp"
#include "topic_pool.h"
#include "topics.hpp"
#include <cmath>
#include <cstdint>
#include <math.h>
#include "Motor.hpp"
#include "MotorControl.hpp"
#include "Axis.hpp"
#include "Encoder.hpp"
#include "com_config.hpp"

//舵轮底盘设备
VESCMotor U8_motor1(&fdcan1_bus, 0x965, true, 101, true);
VESCMotor U8_motor2(&fdcan1_bus, 0x966, true, 102, true);
VESCMotor U8_motor3(&fdcan1_bus, 0x967, true, 103, true);
VESCMotor U8_motor4(&fdcan1_bus, 0x968, true, 104, true);
BRT38B Encoder_1(&fdcan2_bus, 0x01, false, 0x01, false);
BRT38B Encoder_2(&fdcan2_bus, 0x02, false, 0x02, false);
BRT38B Encoder_3(&fdcan2_bus, 0x03, false, 0x03, false);
BRT38B Encoder_4(&fdcan2_bus, 0x04, false, 0x04, false);
C610Motor DJI2006_motor1(&fdcan2_bus, 0x205, false, 0x1FF, false);
C610Motor DJI2006_motor2(&fdcan2_bus, 0x206, false, 0x1FF, false);
C610Motor DJI2006_motor3(&fdcan2_bus, 0x207, false, 0x1FF, false);
C610Motor DJI2006_motor4(&fdcan2_bus, 0x208, false, 0x1FF, false);

VESCMotorController U8_controller1(U8_motor1);
VESCMotorController U8_controller2(U8_motor2);
VESCMotorController U8_controller3(U8_motor3);
VESCMotorController U8_controller4(U8_motor4);
DJIMotorController DJI2006_controller1(DJI2006_motor1);
DJIMotorController DJI2006_controller2(DJI2006_motor2);
DJIMotorController DJI2006_controller3(DJI2006_motor3);
DJIMotorController DJI2006_controller4(DJI2006_motor4);

RotaryAxis SwerveDrive1(U8_controller1,4.0f);
RotaryAxis SwerveDrive2(U8_controller2,4.0f);
RotaryAxis SwerveDrive3(U8_controller3,4.0f);
RotaryAxis SwerveDrive4(U8_controller4,4.0f);
RotaryAxis SwerveSteer1(DJI2006_controller1,-112.0f/19.0f,4.0f);
RotaryAxis SwerveSteer2(DJI2006_controller2,-112.0f/19.0f,4.0f);
RotaryAxis SwerveSteer3(DJI2006_controller3,-112.0f/19.0f,4.0f);
RotaryAxis SwerveSteer4(DJI2006_controller4,-112.0f/19.0f,4.0f);

// 全向轮底盘设备
C620Motor chassis_motor1(&fdcan3_bus, 0x201, 0, 0x200, 0);
C620Motor chassis_motor2(&fdcan3_bus, 0x202, 0, 0x200, 0);
C620Motor chassis_motor3(&fdcan3_bus, 0x203, 0, 0x200, 0);
C620Motor chassis_motor4(&fdcan3_bus, 0x204, 0, 0x200, 0);

//上层电机
DM4310Motor arm4310_motor(&fdcan2_bus, 0x301, 0, 0x01, 0,DM4310Motor::PosWithSpeed);

// C620Motor arm3508_motor(&fdcan3_bus, 0x206, 0, 0x1FF, 0);
// DJIMotorController arm3508_controller(arm3508_motor);
// RotaryAxis arm3508_Axis(arm3508_controller, 1.0f);

