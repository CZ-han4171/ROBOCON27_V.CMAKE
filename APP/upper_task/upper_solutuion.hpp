#pragma once

#include <algorithm>
#include <array>
#include <cstddef>
#include <type_traits>

#include "Axis.hpp"
#include "ControlOptimize.hpp"
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
        : motors_{&VPmotor_Left, &VPmotor_Right} {
        for (auto &optimizer : cos_optimizers_) {
            optimizer.ConfigImprovement(ControlOptimizer::Control_Improvement::COSPLAN);
        }
    }

    void configurePosPid(const std::array<PidParam, kVPCount> &params) {
        for (size_t i = 0; i < cos_optimizers_.size(); ++i) {
            applyPidParam(cos_optimizers_[i].pos_pid_, params[i]);
        }
    }

    void getVP_Pos(){
        for (size_t i = 0; i < motors_.size(); ++i) {
            motors_[i]->UpdateAxisPos();
            current_motor_rad_[i] = motors_[i]->getAxisSumPos_rad();

            current_motor_speed_rad_s_[i] = motors_[i]->getAxisSpeed_rad();
        }
        actual_VP_rad_[kPitch] = (current_motor_rad_[kLeft] + current_motor_rad_[kRight]) / 2.0f;   //pitch
        actual_VP_rad_[kYaw] = (current_motor_rad_[kLeft] - current_motor_rad_[kRight]) / 2.0f / kYawGearRatio;  //yaw
    }

    void solveTargetSpeed(float pitch, float yaw) {
        target_VP_rad_[kPitch] = std::clamp(pitch, kPitchMin, kPitchMax) * kDegToRad;
        target_VP_rad_[kYaw] = std::clamp(yaw, kYawMin, kYawMax) * kDegToRad;

        for (size_t i = 0; i < kVPCount; ++i) {
            error_VP_rad_[i] = target_VP_rad_[i] - actual_VP_rad_[i];
            output_VP_speed_rad_[i] = PID_Calculate(&cos_optimizers_[i].pos_pid_, actual_VP_rad_[i], target_VP_rad_[i]);
        }

        output_motor_speed_rad_s_[kLeft] = (output_VP_speed_rad_[kPitch] + (output_VP_speed_rad_[kYaw] * kYawGearRatio));
        output_motor_speed_rad_s_[kRight] = (output_VP_speed_rad_[kPitch] - (output_VP_speed_rad_[kYaw] * kYawGearRatio));
    }

    void run(const pub_VPupper_cmd &cmd,float action_dt_s) {
        if (!std::isfinite(action_dt_s) || action_dt_s <= 0.0f) return;
        getVP_Pos();
        setTrajectory(cmd.pitch, cmd.yaw, action_dt_s);
        PlanSpeedSolution_COS();
        for (size_t i = 0; i < motors_.size(); ++i) {
            motors_[i]->setAxisSpeed_rad_s(output_motor_speed_rad_s_[i]);
        }
    }

    void setTrajectory(float pitch, float yaw,float common_duration_s = 0.01f)
    {
        const float pitch_rad = std::clamp(pitch, kPitchMin, kPitchMax) * kDegToRad;
        const float yaw_rad = std::clamp(yaw, kYawMin, kYawMax) * kDegToRad;

        target_VP_rad_[kPitch] = pitch_rad;
        target_VP_rad_[kYaw] = yaw_rad;

        getVP_Pos();
        common_T_s_ = common_duration_s;
        for (size_t i = 0; i < kVPCount; ++i) {
                cos_optimizers_[i].Set(target_VP_rad_[i], actual_VP_rad_[i], common_T_s_);
        }
    }

    void PlanSpeedSolution_COS()
    {
        for(size_t i = 0;i < kVPCount; ++i)
        {
            if (std::fabs(cos_optimizers_[i].traj_param_.delat_traj()) < 1e-6f || cos_optimizers_[i].traj_param_.T <= 0.0f) 
            {
                cos_optimizers_[i].traj_param_.ref = cos_optimizers_[i].traj_param_.traget;
                output_VP_speed_rad_[i] = cos_optimizers_[i].PosPidCalculate(cos_optimizers_[i].traj_param_.traget - actual_VP_rad_[i]);
                ismoved_[i] = false;
            }
            else {
                ismoved_[i] = true;
            }
        } 
        

        for (size_t i = 0; i < kVPCount; ++i) {
            if(ismoved_[i])
            {
                cos_optimizers_[i].Run(actual_VP_rad_[i]);
                output_VP_speed_rad_[i] = cos_optimizers_[i].GetPosOutput();
            }
        }
        output_motor_speed_rad_s_[kLeft] = output_VP_speed_rad_[kPitch]
            + kYawGearRatio * output_VP_speed_rad_[kYaw];
        output_motor_speed_rad_s_[kRight] = output_VP_speed_rad_[kPitch]
            - kYawGearRatio * output_VP_speed_rad_[kYaw];
    }


private:
    static void applyPidParam(PID_t &pid, const PidParam &param) {
        pid = {};
        pid.Kp = param.kp;
        pid.Ki = param.ki;
        pid.Kd = param.kd;
        pid.MaxOut = param.max_out;
        pid.DeadBand = param.deadband;
        pid.Improve = param.improve;
        PID_Init(&pid);
    }

    std::array<RotaryAxis *, kMotorCount> motors_{};

    std::array<ControlOptimizer, kVPCount> cos_optimizers_{};


    std::array<float, kMotorCount> target_motor_rad_{};
    std::array<float, kMotorCount> current_motor_rad_{};

    std::array<float, kMotorCount> target_motor_speed_rad_s_{};
    std::array<float, kMotorCount> current_motor_speed_rad_s_{};
    std::array<float, kMotorCount> output_motor_speed_rad_s_{};

    std::array<float, kMotorCount> output_ff_{};
    std::array<float, kMotorCount> output_pid_{};

    std::array<float, kVPCount> actual_VP_rad_{};
    std::array<float, kVPCount> target_VP_rad_{};
    std::array<float, kVPCount> error_VP_rad_{};
    std::array<bool, kVPCount> ismoved_{};

    std::array<float, kVPCount> output_VP_speed_rad_{};

    //规划
    bool isFinished_ = true;
    bool isFetched_  = false;
    float dt_s_;  // 规划周期
    float common_T_s_ = 0.01f;
};



