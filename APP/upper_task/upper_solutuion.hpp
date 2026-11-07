#pragma once

#include <algorithm>
#include <array>
#include <cstddef>

#include "Axis.hpp"
#include "topic_pool.h"
#include "tool.hpp"


#define kPitchMax 180.0f
#define kPitchMin 0.0f
#define kYawMax 360.0f
#define kYawMin 0.0f
#define kYawGearRatio 2.0f  // yaw 输出端的额外减速比：电机反向各转2圈，机构yaw转1圈

class UpperVP{
public:
    enum MotorIndex : size_t { kLeft = 0, kRight = 1, kMotorCount = 2 };
    enum VPIndex : size_t { kPitch = 0, kYaw = 1, kVPCount = 2 };

    UpperVP(RotaryAxis &VPmotor_Left, RotaryAxis &VPmotor_Right)
        : motors_{&VPmotor_Left, &VPmotor_Right} {}

    void getVP_Pos(){
        for (size_t i = 0; i < motors_.size(); ++i) {
            current_motor_rad_[i] = motors_[i]->getAxisSumPos_rad();
        }
        VP_rad_[kPitch] = (current_motor_rad_[kLeft] + current_motor_rad_[kRight]) / 2.0f;   //pitch
        VP_rad_[kYaw] = current_motor_rad_[kLeft] - current_motor_rad_[kRight] / 2.0f / kYawGearRatio;  //yaw
    }

    // 纯逆解：输入末端角度（度），返回左右轴累计角度（rad）
    void solveMotorTargets(const pub_VPupper_cmd &cmd) {
        const float pitch = std::clamp(cmd.pitch, kPitchMin, kPitchMax);
        const float yaw = std::clamp(cmd.yaw, kYawMin, kYawMax);

        target_motor_rad_[kLeft] = (pitch + (yaw * kYawGearRatio)) * kDegToRad;
        target_motor_rad_[kRight] = (pitch - (yaw * kYawGearRatio)) * kDegToRad;
    }

    // 周期控制入口：限位、逆解、下发；目标同时保留供调试或规划使用。
    void run(const pub_VPupper_cmd &cmd) {
        solveMotorTargets(cmd);
        for (size_t i = 0; i < motors_.size(); ++i) {
            motors_[i]->setAxisSumPos_rad(target_motor_rad_[i]);
        }
    }

private:
    std::array<RotaryAxis *, kMotorCount> motors_{};
    std::array<float, kMotorCount> target_motor_rad_{};
    std::array<float, kMotorCount> current_motor_rad_{};
    std::array<float, kVPCount> VP_rad_{};
};
