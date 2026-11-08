#pragma once

#include <algorithm>
#include <array>
#include <cstddef>

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
        : motors_{&VPmotor_Left, &VPmotor_Right} {}

    // Parameters are ordered as [pitch, yaw]; MaxOut: rad/s, DeadBand: rad.
    void configurePosPid(const std::array<PidParam, kVPCount> &params) {
        for (size_t i = 0; i < VP_pid_.size(); ++i) {
            applyPidParam(VP_pid_[i], params[i]);
        }
    }

    // void configureSingleAxisPosPid(VPIndex axis, const PidParam &param) {
    //     applyPidParam(VP_pid_[axis], param);
    // }

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
            output_VP_speed_rad_[i] = PID_Calculate(&VP_pid_[i], actual_VP_rad_[i], target_VP_rad_[i]);
        }

        output_motor_speed_rad_s_[kLeft] = (output_VP_speed_rad_[kPitch] + (output_VP_speed_rad_[kYaw] * kYawGearRatio));
        output_motor_speed_rad_s_[kRight] = (output_VP_speed_rad_[kPitch] - (output_VP_speed_rad_[kYaw] * kYawGearRatio));
    }

    void run(const pub_VPupper_cmd &cmd,float dt_ms) {
        dt_s_ = dt_ms * 0.001f;
        getVP_Pos();
        setTrajectory(cmd.pitch, cmd.yaw);
        PlanSpeedSolution();
        for (size_t i = 0; i < motors_.size(); ++i) {
            motors_[i]->setAxisSpeed_rad_s(output_motor_speed_rad_s_[i]);
        }
    }

    void setTrajectory(float pitch, float yaw, float T = 0.0f) {
        if (!std::isfinite(pitch) ||
            !std::isfinite(yaw) ||
            !std::isfinite(T) || T < 0.0f) {
            return;
        }

        for (size_t i = 0; i < kVPCount; ++i) {
            if (!std::isfinite(VP_PlanParam_.Acc[i]) ||
                !std::isfinite(VP_PlanParam_.Dec[i]) ||
                !std::isfinite(VP_PlanParam_.w_Max[i]) ||
                VP_PlanParam_.Acc[i] <= 0.0f ||
                VP_PlanParam_.Dec[i] <= 0.0f ||
                VP_PlanParam_.w_Max[i] <= 0.0f) {
                return;
            }
        }

        const float pitch_rad =
            std::clamp(pitch, kPitchMin, kPitchMax) * kDegToRad;
        const float yaw_rad =
            std::clamp(yaw, kYawMin, kYawMax) * kDegToRad;

        
        if(fabsf(pitch_rad - VP_TrajectParam_.target[kPitch]) <= 1e-6f &&
            fabsf(yaw_rad - VP_TrajectParam_.target[kYaw]) <= 1e-6f)
        {   

            return;
        }
        

        target_VP_rad_[kPitch] = pitch_rad;
        target_VP_rad_[kYaw] = yaw_rad;

        getVP_Pos();

        bool moving = false;
        for (size_t i = 0; i < kVPCount; ++i) {
            VP_TrajectParam_.start[i] = actual_VP_rad_[i];
            VP_TrajectParam_.now[i] = actual_VP_rad_[i];
            VP_TrajectParam_.ref[i] = actual_VP_rad_[i];
            VP_TrajectParam_.target[i] = target_VP_rad_[i];

            moving |= fabsf(VP_TrajectParam_.delat_traj(i)) > 1e-6f;

            output_pid_[i] = 0.0f;
        }

        VP_TrajectParam_.T = T;
        VP_TrajectParam_.t = 0.0f;
        VP_TrajectParam_.auto_calculate_T = (T == 0.0f);

        VP_TrajectParam_.u = moving ? 0.0f : 1.0f;
        VP_TrajectParam_.u_speed = 0.0f;

        isFinished_ = !moving;
    }

    void PlanSpeedSolution() {
        if (!std::isfinite(dt_s_) || dt_s_ <= 0.0f) {
            return;
        }

        auto &traj = VP_TrajectParam_;

        // 实际位置由getVP_Pos()更新。
        for (size_t i = 0; i < kVPCount; ++i) {
            traj.now[i] = actual_VP_rad_[i];
        }

        if (!isFinished_) {
            float Acc = INFINITY;
            float Dec = INFINITY;
            float w_Max = INFINITY;

            for (size_t i = 0; i < kVPCount; ++i) {
                const float distance = fabsf(traj.delat_traj(i));

                if (distance <= 1e-6f)
                    continue;

                Acc = std::min(Acc, VP_PlanParam_.Acc[i] / distance);
                Dec = std::min(Dec, VP_PlanParam_.Dec[i] / distance);
                w_Max = std::min(
                    w_Max, VP_PlanParam_.w_Max[i] / distance);
            }

            // 计算理想连续梯形的最短时间。
            const float peak = std::min(
                w_Max,
                sqrtf(2.0f / (1.0f / Acc + 1.0f / Dec)));

            const float ta = peak / Acc;
            const float td = peak / Dec;
            const float tc = std::max(
                0.0f, (1.0f - 0.5f * peak * (ta + td)) / peak);

            const float minimum_T = ta + tc + td;

            // 指定较长T时，等比例放慢。
            // 指定时间过短时，采用满足限制的最短时间。
            const float scale = traj.auto_calculate_T
                ? 1.0f
                : std::max(1.0f, traj.T / minimum_T);

            traj.T = minimum_T * scale;

            Acc /= scale * scale;
            Dec /= scale * scale;
            w_Max /= scale;

            const float previous_speed = traj.u_speed;

            const float w_Acc =
                previous_speed + Acc * dt_s_;

            const float w_Dec = sqrtf(
                2.0f * Dec * fmaxf(0.0f, 1.0f - traj.u));

            traj.u_speed =
                std::min(w_Max, std::min(w_Acc, w_Dec));

            traj.u +=
                0.5f * (previous_speed + traj.u_speed) * dt_s_;

            traj.t += dt_s_;

            if (traj.u >= 1.0f) {
                traj.u = 1.0f;
                traj.u_speed = 0.0f;
                isFinished_ = true;
            }
        }

        for (size_t i = 0; i < kVPCount; ++i) {
            const float delta = traj.delat_traj(i);

            traj.ref[i] = traj.start[i] + delta * traj.u;
            
            output_ff_[i] = delta * traj.u_speed;

            error_VP_rad_[i] = traj.ref[i] - traj.now[i];

            output_pid_[i] = PID_Calculate(
                &VP_pid_[i], traj.now[i], traj.ref[i]);

            output_VP_speed_rad_[i] =
                output_ff_[i] + output_pid_[i];
        }

        // 两轴角速度 → 左右驱动轴角速度。
        output_motor_speed_rad_s_[kLeft] =
            output_VP_speed_rad_[kPitch]
            + kYawGearRatio * output_VP_speed_rad_[kYaw];

        output_motor_speed_rad_s_[kRight] =
            output_VP_speed_rad_[kPitch]
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

    struct PlanParam {
        float Acc[kVPCount]{};
        float Dec[kVPCount]{};
        float w_Max[kVPCount]{};

        PlanParam(float Acc_in = M_PI,
                float Dec_in = M_PI,
                float w_Max_in = 10.0f * M_PI) {
            for (size_t i = 0; i < kVPCount; ++i) {
                Acc[i] = Acc_in;
                Dec[i] = Dec_in;
                w_Max[i] = w_Max_in;
            }
        }
    };
    struct TrajectParam {
        float start[kVPCount]{};
        float target[kVPCount]{};
        float now[kVPCount]{};
        float ref[kVPCount]{};

        float T{};
        float t{};        
        bool auto_calculate_T{true};

        float u{};
        float u_speed{};

        float delat_traj(size_t axis) const {
            return target[axis] - start[axis];
        }

        float delat_now(size_t axis) const {
            return target[axis] - now[axis];
        }

        float delat_ref(size_t axis) const {
            return target[axis] - ref[axis];
        }
    };

    std::array<RotaryAxis *, kMotorCount> motors_{};

    std::array<PID_t, kVPCount> VP_pid_{};

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

    std::array<float, kVPCount> output_VP_speed_rad_{};

    //规划
    bool isFinished_ = true;
    bool isFetched_  = false;
    TrajectParam VP_TrajectParam_;
    PlanParam VP_PlanParam_;
    float dt_s_ = 0.0f;   // 规划周期
};


// class VP_Trajectory{
//     public:
//         struct TrajectoryParam {
//             float start;
//             float target;
//             float now;
//             float T;
//             float t;
//             bool auto_calculate_T = true;

//             float delat_traj(){ return target - start; }
//             float delat_now(){ return target - now; }
//         };

//         void setTrajectoryParam(const float &start_speed, const float &target_speed, const float &T){
//             traj_param_.start = start_speed;
//             traj_param_.target = target_speed;
//             traj_param_.now = upperVP_.;
//             traj_param_.T = T;
//             traj_param_.t = 0.0f;
//         }
        
//         float T_Planner()
//         {
//             upperVP_.getVP_Pos();
//             upperVP_.solveTargetSpeed(VP_target_pos_rad_[0], VP_target_pos_rad_[1]);

//         }

//         void runTrajectory(const float &now_speed, const float &dt){
//             if(isFinished_ || dt <= 0.0f) {
//                 return;
//             }
//             if(isFetched_)
//             {
//                 output_speed_ = PlanSpeedSolution(now_speed, output_pid_, traj_param_);
//             }
//             else 
//             {
//                 output_speed_ = PlanSpeedSolution(now_speed, output_pid_, traj_param_);
//             }
//         }

//     private:
//         TrajectoryParam traj_param_;
//         UpperVP upperVP_;
//         bool isFinished_ = false;
//         bool isFetched_  = false;

//         std::array<float, 2> VP_target_pos_rad_{};
//         std::array<float, 2> VP_current_pos_rad_{};
        
//         std::array<float, 2> VP_target_speed_rad_s_{};
//         std::array<float, 2> VP_current_speed_rad_s_{};


// };
