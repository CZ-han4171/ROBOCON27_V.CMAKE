/**
 * @file chassis_lift_solution.hpp
 * @author CZhan
 * @brief 
 * @version 1.0
 * @date 2026-11-19
 *
 * @copyright Copyright (c) 2026
 *
 * @attention : 
 * @note :
 * @versioninfo :
 */
#pragma once

#include "Axis.hpp"
#include "robot_params.hpp"
#include "ControlOptimize.hpp"
#include "Encoder.hpp"
#include "Motor.hpp"
#include "MotorControl.hpp"
#include "pid_controller.hpp"
#include "topic_pool.h"
#include "tool.hpp"

#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <math.h>

class ChassisLift {
public:
    struct Geometry {
        float wheel_diameter_m; // 轮径
        Geometry(float wheel_diameter = 0.04f)
            : wheel_diameter_m(wheel_diameter){}
    };

    enum ControlMode : size_t {
        PosCtrl,
        SpeedCtrl,
        MitCtrl
    };

    ChassisLift( GeneralAxis<2,4> &Lift_legs , 
                 GeneralAxis<1,2> &Lift_Drive,
                 const Geometry &geometry = Geometry())
                :Lift_Legs_{Lift_legs},
                 Lift_Drive_{Lift_Drive},
                 geometry_{geometry}{
    }

    // void run(const pub_chassis_lift_cmd &cmd){
    //     front_leg_traget_height_m = cmd.height_front_;
    //     hind_leg_traget_height_m = cmd.height_hind_;
    //     drive_traget_speed_m_s = 2.0f*cmd.speed_x_/geometry_.wheel_diameter_m;
    //     Lift_Legs_.setAxisPos({front_leg_traget_height_m,hind_leg_traget_height_m},1.0f);
    //     Lift_Drive_.setAxisSpeed({drive_traget_speed_m_s});
    //     front_leg_actual_height_m = Lift_Legs_.getAxisPos()[0];
    //     hind_leg_actual_height_m = Lift_Legs_.getAxisPos()[1];

    void run(const pub_chassis_lift_cmd &cmd){
        drive_traget_speed_m_s = 2.0f*cmd.speed_x_/geometry_.wheel_diameter_m;    
        lift_traget_state_ = cmd.state_;
        complete_time = cmd.T;
        // Lift_Legs_.setAxisPos({Lift_state_to_height[lift_traget_state_][0],Lift_state_to_height[lift_traget_state_][1]},complete_time);
        Lift_Legs_.setAxisPos(Lift_state_to_height[lift_traget_state_],complete_time);
        // Lift_Legs_.setAxisSpeed({0.0f,0.0f});
        Lift_Drive_.setAxisSpeed({drive_traget_speed_m_s});
        front_leg_actual_height_m = Lift_Legs_.getAxisPos()[0];
        hind_leg_actual_height_m = Lift_Legs_.getAxisPos()[1];
        if(Lift_Legs_.getAxisfinish()){
            lift_actual_state_ = lift_traget_state_;
        }
    }


    LiftState getLiftState(){ return lift_actual_state_; }
    
    GeneralAxis<2,4>& Lift_Legs_;
    GeneralAxis<1,2>& Lift_Drive_;
    Geometry geometry_{};

    float front_leg_traget_height_m = 0.0f;
    float hind_leg_traget_height_m = 0.0f;
    float front_leg_actual_height_m = 0.0f;
    float hind_leg_actual_height_m = 0.0f;
    
    MitParam mit_front_leg_{};
    MitParam mit_hind_leg_{};
    float drive_traget_speed_m_s = 0.0f;
    float drive_actual_speed_m_s = 0.0f;

    std::optional<float> complete_time = std::nullopt;

    LiftState lift_traget_state_;
    LiftState lift_actual_state_;

    LiftState lift_traget_state_last_;
    LiftState lift_actual_state_last_;

    ControlMode LiftControlMode_ = PosCtrl;
    ControlMode DriveControlMode_ = SpeedCtrl;

    bool is_lift_preparing_ = false;

    const std::array<std::array<float,2>,5> Lift_state_to_height = {{
        {chassis_lift_front_UP,chassis_lift_hind_UP},
        {chassis_lift_MIDDLE,chassis_lift_MIDDLE},
        {chassis_lift_DOWN,chassis_lift_DOWN},
        {chassis_lift_DOWN,chassis_lift_MIDDLE},
        {chassis_lift_MIDDLE,chassis_lift_DOWN}
    }};

};
