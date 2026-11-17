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

class chassis_lift {
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
    struct Geometry {
        float_t wheel_diameter_m; // 轮径

        Geometry(float wheel_diameter = 0.8f)
            : wheel_diameter_m(wheel_diameter){}
    };
    enum class PositionControlMode {
        kJointSpace,
        kCartesianSpace
    };
}

