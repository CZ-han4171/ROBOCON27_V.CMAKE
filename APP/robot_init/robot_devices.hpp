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

//舵轮底盘设备
extern VESCMotor U8_motor1;
extern VESCMotor U8_motor2;
extern VESCMotor U8_motor3;
extern VESCMotor U8_motor4;
extern BRT38B Encoder_1;
extern BRT38B Encoder_2;
extern BRT38B Encoder_3;
extern BRT38B Encoder_4;
extern C610Motor DJI2006_motor1;
extern C610Motor DJI2006_motor2;
extern C610Motor DJI2006_motor3;
extern C610Motor DJI2006_motor4;

extern VESCMotorController U8_controller1;
extern VESCMotorController U8_controller2;
extern VESCMotorController U8_controller3;
extern VESCMotorController U8_controller4;
extern DJIMotorController DJI2006_controller1;
extern DJIMotorController DJI2006_controller2;
extern DJIMotorController DJI2006_controller3;
extern DJIMotorController DJI2006_controller4;

extern RotaryAxis SwerveDrive1;
extern RotaryAxis SwerveDrive2;
extern RotaryAxis SwerveDrive3;
extern RotaryAxis SwerveDrive4;
extern RotaryAxis SwerveSteer1;
extern RotaryAxis SwerveSteer2;
extern RotaryAxis SwerveSteer3;
extern RotaryAxis SwerveSteer4;

// 全向轮底盘设备
extern C620Motor chassis_motor1;
extern C620Motor chassis_motor2;
extern C620Motor chassis_motor3;
extern C620Motor chassis_motor4;

//上层电机
extern DM4310Motor arm4310_motor;

extern C610Motor DJI2006_motor_VPleft;
extern C610Motor DJI2006_motor_VPright;
extern DJIMotorController DJI2006_motor_VPleft_controller;
extern DJIMotorController DJI2006_motor_VPright_controller;
extern RotaryAxis VPLeft;
extern RotaryAxis VPRight;

extern C620Motor DJI3508_motor_DLleft;
extern C620Motor DJI3508_motor_DLright;
extern DJIMotorController DJI3508_motor_DLleft_controller;
extern DJIMotorController DJI3508_motor_DLright_controller;
extern LinearAxis DLLeft;
extern LinearAxis DLRight;

extern C610Motor DJI2006_motor_MLleft;
extern C610Motor DJI2006_motor_MLright;
extern DJIMotorController DJI2006_motor_MLleft_controller;
extern DJIMotorController DJI2006_motor_MLright_controller;
extern LinearAxis MLLeft;
extern LinearAxis MLRight;

extern C610Motor DJI2006_motor_EClaw;
extern C610Motor DJI2006_motor_EVP;
extern DJIMotorController DJI2006_motor_EClaw_controller;
extern DJIMotorController DJI2006_motor_EVP_controller;
extern RotaryAxis EClaw;
extern RotaryAxis EVP;

extern C610Motor DJI2006_motor_Claw;
extern DJIMotorController DJI2006_controller_Claw;
extern RotaryAxis Claw;
