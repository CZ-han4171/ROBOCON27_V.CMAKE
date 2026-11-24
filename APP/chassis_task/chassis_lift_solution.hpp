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

    enum LiftLegState : size_t{
        UP,
        MIDDLE,
        DOWN,
        RUNNING
    };

    enum LiftState : size_t{
        Collapse,
        Crouh,
        HoldUp,
        FrontHold,
        HindHold
    };

    ChassisLift( GeneralAxis<2,4> &Lift_legs , 
                 GeneralAxis<1,2> &Lift_Drive,
                 const Geometry &geometry = Geometry())
                :Lift_Legs_{Lift_legs},
                 Lift_Drive_{Lift_Drive},
                 geometry_{geometry}{
    }

    void set_Lift_perparing(bool is_preparing){
        if(is_preparing){
            if(!is_lift_preparing_){
                front_leg_traget_state_ = MIDDLE;
                hind_leg_traget_state_ = MIDDLE;
            }
        }else{
            if(is_lift_preparing_){
                front_leg_traget_state_ = UP;
                hind_leg_traget_state_ = UP;
            }
        }
        is_lift_preparing_ = is_preparing;
    }

    void Lift_front_leg_ctrl(LiftLegState pos){
        // if(front_leg_traget_state_ == UP || front_leg_actual_state_ == UP)return;
        if(!is_lift_preparing_)return;
        front_leg_traget_state_ = pos;
        Lift_Legs_.optimizer_.ConfigImprovement(Control_Improvement::SPEEDPLAN);
        LiftControlMode_ = ControlMode::PosCtrl;
    }

    void Lift_hind_leg_ctrl(LiftLegState pos){
        if(!is_lift_preparing_)return;
        hind_leg_traget_state_ = pos;
        Lift_Legs_.optimizer_.ConfigImprovement(Control_Improvement::SPEEDPLAN);
        LiftControlMode_ = ControlMode::PosCtrl;
    }

    void Lift_ctrl(LiftLegState pos){
        if(!is_lift_preparing_)return;
        if(front_leg_traget_state_ != hind_leg_traget_state_)return;
        // if(front_leg_traget_state_ == UP && pos == DOWN)pos = MIDDLE;
        // if(front_leg_traget_state_ == DOWN && pos == UP)pos = MIDDLE;
        front_leg_traget_state_ = pos;
        hind_leg_traget_state_ = pos;
        Lift_Legs_.optimizer_.ConfigImprovement(Control_Improvement::COSPLAN);
    }

    void run(const pub_chassis_lift_cmd &cmd){
        front_leg_traget_height_m = cmd.height_front_;
        hind_leg_traget_height_m = cmd.height_hind_;
        drive_traget_speed_m_s = 2.0f*cmd.speed_x_/geometry_.wheel_diameter_m;
        Lift_Legs_.setAxisPos({front_leg_traget_height_m,hind_leg_traget_height_m},1.0f);
        Lift_Drive_.setAxisSpeed({drive_traget_speed_m_s});
        front_leg_actual_height_m = Lift_Legs_.getAxisPos()[0];
        hind_leg_actual_height_m = Lift_Legs_.getAxisPos()[1];

        // if(front_leg_traget_state_ == UP && hind_leg_traget_state_ == UP){
        //     LiftControlMode_ = ControlMode::MitCtrl;
        //     Lift_Legs_.setAxisMit({{{-0.25f,0.0f,0.0f,0.0f,100.0f},{-0.25f,0.0f,0.0f,100.0f}}});
        // }else {
        //     switch (front_leg_traget_state_){
        //         case LiftLegState::UP:{
        //         }
        //         case LiftLegState::MIDDLE:{
        //             front_leg_traget_height_m = chassis_lift_MIDDLE;
        //             break;
        //         }
        //         case LiftLegState::DOWN:{
        //             front_leg_traget_height_m = chassis_lift_DOWN;
        //             break;
        //         }
        //         case LiftLegState::RUNNING:{
        //         }
        //         default: break;
        //     }
        //     switch (hind_leg_traget_state_){
        //         case LiftLegState::UP:{
        //         }
        //         case LiftLegState::MIDDLE:{
        //             hind_leg_traget_height_m = chassis_lift_MIDDLE;
        //             break;
        //         }
        //         case LiftLegState::DOWN:{
        //             hind_leg_traget_height_m = chassis_lift_DOWN;
        //             break;
        //         }
        //         case LiftLegState::RUNNING:{
        //         }
        //         default: break;
        //     }
        //     Lift_Legs_.setAxisPos({front_leg_traget_height_m,hind_leg_traget_height_m});
        // }

        // // Lift_Legs_.setAxisSpeed({0.0f,0.0f});
        // // Lift_Drive_.setAxisSpeed({0.0f});

        

        // front_leg_actual_state_ = RUNNING;
        // if(fabsf(front_leg_actual_height_m - chassis_lift_MIDDLE) >= 0.02f)front_leg_actual_state_ = MIDDLE;
        // else if(fabsf(front_leg_actual_height_m - chassis_lift_DOWN) >= 0.02f)front_leg_actual_state_ = DOWN;
        // else if(fabsf(front_leg_actual_height_m - chassis_lift_max_front_height) >= 0.02f)front_leg_actual_state_ = UP;

        // hind_leg_actual_state_ = RUNNING;
        // if(fabsf(hind_leg_actual_state_ - chassis_lift_MIDDLE) >= 0.02f)hind_leg_actual_state_ = MIDDLE;
        // else if(fabsf(hind_leg_actual_state_ - chassis_lift_DOWN) >= 0.02f)hind_leg_actual_state_ = DOWN;
        // else if(fabsf(hind_leg_actual_state_ - chassis_lift_max_hind_height) >= 0.02f)hind_leg_actual_state_ = UP;
    }

private:
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

    LiftLegState front_leg_traget_state_ = UP;
    LiftLegState hind_leg_traget_state_ = UP;
    LiftLegState front_leg_actual_state_ = UP;
    LiftLegState hind_leg_actual_state_ = UP;

    ControlMode LiftControlMode_ = PosCtrl;
    ControlMode DriveControlMode_ = SpeedCtrl;

    bool is_lift_preparing_ = false;

};
