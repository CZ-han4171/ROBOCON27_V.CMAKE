/**
 * @file chassis_lift_solution.hpp
 * @author CZhan
 * @brief 
 * @version 0.1
 * @date 2026-11-15
 *
 * @copyright 
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
#include "ControlOptimize.hpp"
#include "optional"

#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <math.h>

class ChassisLift {
    enum LegIndex : size_t {
        kLeftUp = 0,
        kRightUp = 1,
        kLeftDown = 2,
        kRightDown = 3,
        kLegCount = 4,
    };
    enum WheelIndex : size_t {
        kLeft = 0,
        kRight = 1,
        kWheelCount = 2,
    };
    struct Leg_Geometry {
        float_t wheel_diameter_m; // 轮径

        Wheel_Geometry(float wheel_diameter = 0.8f)
            : wheel_diameter_m(wheel_diameter){}
    };
    struct Wheel_Geometry {
        float_t max_ground_diameter_m; // 最大离地高度
        float_t max_stroke_m; // 最大行程

        Wheel_Geometry(float_t max_ground_diameter = 0.024f,
                 float_t max_stroke = 0.33f)
            : max_ground_diameter_m(max_ground_diameter),
              max_stroke_m(max_stroke) {}
    };
    enum class PositionControlMode {
        kJointSpace,
        kCartesianSpace
    };

    ChassisLift(LinearAxis &Leg_lu, LinearAxis &Leg_ru, LinearAxis &Leg_ld, LinearAxis &Leg_ld,
                RotaryAxis &Wheel_left, RotaryAxis &Wheel_right,
            )
}

