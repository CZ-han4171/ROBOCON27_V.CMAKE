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
#include "Encoder.hpp"
#include "Motor.hpp"
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

    ChassisLift( GeneralAxis<2,4> &Lift_legs , 
                 GeneralAxis<1,2> &Lift_Drive,
                 const Geometry &geometry = Geometry())
                :Lift_Legs_{Lift_legs},
                 Lift_Drive_{Lift_Drive},
                 geometry_{geometry}{
    }

    void run(const pub_chassis_lift_cmd &cmd){
        height_front_leg_m = cmd.height_front_;
        height_hind_leg_m = cmd.height_hind_;
        speed_drive_m_s = 2.0f*cmd.speed_x_/geometry_.wheel_diameter_m;

        Lift_Legs_.setAxisPos({height_front_leg_m,height_hind_leg_m});
        Lift_Drive_.setAxisSpeed({speed_drive_m_s});
    }

private:
    GeneralAxis<2,4>& Lift_Legs_;
    GeneralAxis<1,2>& Lift_Drive_;
    Geometry geometry_{};

    float height_front_leg_m = 0.0f;
    float height_hind_leg_m = 0.0f;
    float speed_drive_m_s = 0.0f;
};
