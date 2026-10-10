/**
 * @file Axis.hpp
 * @author CZhan
 * @brief
 * @version 1.0
 * @date 2026-10-10
 *
 * @copyright
 *
 * @attention :
 * @note :
 * @versioninfo :
 */
#pragma once

#include "main.h"
#include <cmath>
#include <cstdint>
#include <stdatomic.h>
#include <stdint.h>
#include "MotorControl.hpp"
#include "tool.hpp"

class RotaryAxis{
public:
    explicit RotaryAxis(MotorControllerBase& MotorController, float motor_rad_per_output_rad)
        : MotorController_(MotorController), motor_rad_per_output_rad_(motor_rad_per_output_rad){}

    enum class ForbiddenZoneMode {
        None,     // 不启用禁区
        Enabled   // 启用禁区
    };

    void setAxisPos_rad(float Axis_single_pos){
        float delat = 0.0f;
        if(isEnableForbidden_ == ForbiddenZoneMode::Enabled){
            forbiddenPoint_ = Warp_ToRange(forbiddenPoint_, 0, 2*M_PI);
            float Axis_traget_single_pos_forbidden = Warp_ToRange(Axis_single_pos - forbiddenPoint_, 0, 2*M_PI);
            float Axis_single_pos_forbidden = Warp_ToRange(Axis_single_pos_ - forbiddenPoint_, 0, 2*M_PI);
            delat = Axis_traget_single_pos_forbidden - Axis_single_pos_forbidden;
        }else if(isEnableForbidden_ == ForbiddenZoneMode::None){
            delat = Warp_ToRange(Axis_single_pos - Axis_single_pos_, -M_PI, M_PI);
        }
        Axis_single_pos_ = Axis_single_pos;
        Axis_sum_pos_ = Axis_sum_pos_ + delat ;
        float Motor_sum_pos = Axis_sum_pos_*motor_rad_per_output_rad_;
        MotorController_.MotorSumPosCtrl_rad(Motor_sum_pos);
    }
    void setAxisPos_rad(float Axis_traget_single_pos, float Axis_actual_sum_pos){
        float delat = 0.0f;
        if(isEnableForbidden_ == ForbiddenZoneMode::Enabled){
            forbiddenPoint_ = Warp_ToRange(forbiddenPoint_, 0, 2*M_PI);
            float Axis_traget_single_pos_forbidden = Warp_ToRange(Axis_traget_single_pos - forbiddenPoint_, 0, 2*M_PI);
            float Axis_single_pos_forbidden = Warp_ToRange(Axis_single_pos_ - forbiddenPoint_, 0, 2*M_PI);
            delat = Axis_traget_single_pos_forbidden - Axis_single_pos_forbidden;
        }else if(isEnableForbidden_ == ForbiddenZoneMode::None){
            delat = Warp_ToRange(Axis_traget_single_pos - Axis_single_pos_, -M_PI, M_PI);
        }
        Axis_single_pos_ = Axis_traget_single_pos ;
        Axis_sum_pos_ = Axis_sum_pos_ + delat;
        float Motor_traget_sum_pos = Axis_sum_pos_*motor_rad_per_output_rad_;
        float Motor_actual_sum_pos = Axis_actual_sum_pos*motor_rad_per_output_rad_;
        MotorController_.MotorSumPosCtrl_rad(Motor_traget_sum_pos,Motor_actual_sum_pos);
    }
    void setAxisSpeed_rad_s(float Axis_speed){
        float Motor_speed = Axis_speed*motor_rad_per_output_rad_;
        MotorController_.MotorSpeedCtrl_rad_s(Motor_speed);
    }

    void setAxisforbidden(float forbiddenPoint){
        forbiddenPoint_ = forbiddenPoint;
        isEnableForbidden_ = ForbiddenZoneMode::Enabled;
    }

private:
    MotorControllerBase& MotorController_;
    float motor_rad_per_output_rad_;
    float Axis_single_pos_ = 0.0f;
    float Axis_sum_pos_ = 0.0f; 
    float forbiddenPoint_ = 0.0f;
    ForbiddenZoneMode isEnableForbidden_ = ForbiddenZoneMode::None;
};

class LinearAxis{
public:
    explicit LinearAxis(MotorControllerBase& MotorController, float motor_rad_per_output_meter)
        : MotorController_(MotorController), motor_rad_per_output_meter_(motor_rad_per_output_meter){}

    void setAxisPos_meter(float Axis_pos){
        float Motor_pos = Axis_pos*motor_rad_per_output_meter_;
        MotorController_.MotorSumPosCtrl_rad(Motor_pos);
    }
    void setAxisPos_meter(float Axis_traget_pos, float Axis_actual_pos){
        float Motor_traget_pos = Axis_traget_pos*motor_rad_per_output_meter_;
        float Motor_actual_pos = Axis_actual_pos*motor_rad_per_output_meter_;
        MotorController_.MotorSumPosCtrl_rad( Motor_traget_pos, Motor_actual_pos );
    }
    void setAxisSpeed_meter_s(float Axis_speed){
        float Motor_speed = Axis_speed*motor_rad_per_output_meter_;
        MotorController_.MotorSpeedCtrl_rad_s(Motor_speed);
    }

private:
    MotorControllerBase& MotorController_;
    float motor_rad_per_output_meter_;
};


