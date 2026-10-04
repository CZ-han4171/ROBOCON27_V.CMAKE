/**
 * @file tool.cpp
 * @author CZ
 * @brief
 * @version 0.0
 * @date 2026-09-18
 *
 * @copyright Copyright (c) 2026
 *
 * @attention :
 * @note :
 * @versioninfo :
 */
#include <cstdint>
#include <cstring>

#pragma once

inline constexpr float kDegToRad = M_PI / 180.0f;
inline constexpr float kRadToDeg = 180.0f / M_PI;

typedef struct {
    float_t x;
    float_t y;
} Vec2_t;

typedef struct {
    float_t x;
    float_t y;
    float_t z;
} Vec3_t;

typedef struct {
    float radius;   // r
    float theta;    // θ
}Polar_t;

typedef struct {
    float radius;   // r
    float theta;    // θ
    float z;        // z
}Cylinder_t;

inline float Warp_ToRange(float value,float min,float max){
    float range = max - min;
    while(value > max){
        value = value - range;
    }
    while(value < min){
        value = value + range;
    }
    return value;
}

inline int8_t sign(double value) {
    if (value > 0) return 1;
    else if (value < 0) return -1;
    else return 0;
}