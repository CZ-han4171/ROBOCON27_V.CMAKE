/**
 * @file robot_devices.hpp
 * @author CZhan
 * @brief
 * @version 1.0
 * @date 2026-11-20
 *
 * @copyright
 *
 * @attention :
 * @note :
 * @versioninfo :
 */
#include "main.h"
#define chassis_lift_DOWN_COMPENSATE 0.004f
#define chassis_lift_max_front_height 0.0169f
#define chassis_lift_max_hind_height 0.0063f
#define chassis_lift_MIDDLE 0.0f
#define chassis_lift_DOWN (0.2f + chassis_lift_DOWN_COMPENSATE)
// #define chassis_lift_DOWN 0.25f
#define chassis_lift_front_UP -0.015f
#define chassis_lift_hind_UP -0.005f
#define chassis_lift_limit_min -0.02f
#define chassis_lift_limit_max 0.3f

