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

    // Parameters are ordered as [pitch, yaw]; MaxOut: rad/s, DeadBand: rad.
    void configurePosPid(const std::array<PidParam, kVPCount> &params) {
        for (size_t i = 0; i < cos_optimizers_.size(); ++i) {
            applyPidParam(cos_optimizers_[i].pos_pid_, params[i]);
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
            output_VP_speed_rad_[i] = PID_Calculate(&cos_optimizers_[i].pos_pid_, actual_VP_rad_[i], target_VP_rad_[i]);
        }

        output_motor_speed_rad_s_[kLeft] = (output_VP_speed_rad_[kPitch] + (output_VP_speed_rad_[kYaw] * kYawGearRatio));
        output_motor_speed_rad_s_[kRight] = (output_VP_speed_rad_[kPitch] - (output_VP_speed_rad_[kYaw] * kYawGearRatio));
    }

    void run(const pub_VPupper_cmd &cmd,float dt_ms) {
        if (!std::isfinite(dt_ms) || dt_ms <= 0.0f) return;
        dt_s_ = dt_ms * 0.001f;
        getVP_Pos();
        setTrajectory(cmd.pitch, cmd.yaw);
        PlanSpeedSolution_COS();
        for (size_t i = 0; i < motors_.size(); ++i) {
            motors_[i]->setAxisSpeed_rad_s(output_motor_speed_rad_s_[i]);
        }
    }

    void setTrajectory(float pitch, float yaw,float T_s = 0.0f)
    {
        if(!std::isfinite(pitch) || !std::isfinite(yaw) || !std::isfinite(T_s) || T_s < 0.0f)
        {
            return;
        }

        for(size_t i = 0; i < kVPCount; ++i)
        {
            if(!std::isfinite(VP_PlanParam_.Acc[i]) || !std::isfinite(VP_PlanParam_.Dec[i]) || !std::isfinite(VP_PlanParam_.w_Max[i]) ||
                VP_PlanParam_.Acc[i] <= 0.0f || VP_PlanParam_.Dec[i] <= 0.0f || VP_PlanParam_.w_Max[i] <= 0.0f)
            {
                return;
            }
        }

        const float pitch_rad = std::clamp(pitch, kPitchMin, kPitchMax) * kDegToRad;
        const float yaw_rad = std::clamp(yaw, kYawMin, kYawMax) * kDegToRad;

        auto &traj = VP_TrajectParam_;
        if (isFetched_ && fabsf(pitch_rad - traj.target[kPitch]) <= 1e-6f &&
            fabsf(yaw_rad - traj.target[kYaw]) <= 1e-6f) return;

        getVP_Pos();
        const float targets[kVPCount]{pitch_rad, yaw_rad};
        TrajectParam candidate{};
        float duration = T_s;
        bool moving = false;
        for (size_t i = 0; i < kVPCount; ++i) {
            if (!std::isfinite(actual_VP_rad_[i])) return;
            candidate.start[i] = actual_VP_rad_[i];
            candidate.now[i] = actual_VP_rad_[i];
            candidate.ref[i] = actual_VP_rad_[i];
            candidate.target[i] = targets[i];
            const float distance = fabsf(candidate.delat_traj(i));
            moving |= distance > 1e-6f;
            duration = std::max(duration,
                distance * static_cast<float>(M_PI) / (2.0f * VP_PlanParam_.w_Max[i]));
            duration = std::max(duration, sqrtf(distance * static_cast<float>(M_PI * M_PI) /
                (2.0f * std::min(VP_PlanParam_.Acc[i], VP_PlanParam_.Dec[i]))));
        }
        if (!std::isfinite(duration)) return;
        candidate.T_s = moving ? duration : 0.0f;
        candidate.auto_calculate_T = (T_s == 0.0f);
        traj = candidate;
        target_VP_rad_[kPitch] = pitch_rad;
        target_VP_rad_[kYaw] = yaw_rad;
        output_ff_.fill(0.0f);
        isFinished_ = !moving;
        isFetched_ = true;
        for (size_t i = 0; i < kVPCount; ++i) {
            auto &optimizer = cos_optimizers_[i];
            optimizer.ConfigPlanParam(ControlOptimizer::PlanParam(
                VP_PlanParam_.Acc[i], VP_PlanParam_.Dec[i], VP_PlanParam_.w_Max[i]));
            optimizer.traj_param_last_.traget = targets[i] + 1.0f;
            optimizer.Set(targets[i], actual_VP_rad_[i], traj.T_s);
            optimizer.output_ff_ = 0.0f;
            optimizer.output_pid_ = 0.0f;
            optimizer.pos_output_ = 0.0f;
            DWT_GetDeltaT(&optimizer.DWT_CNT);
        }
    }

    void PlanSpeedSolution_COS()
    {
        if (!isFetched_ || !std::isfinite(dt_s_) || dt_s_ <= 0.0f) return;
        auto &traj = VP_TrajectParam_;
        float elapsed = traj.T_s;
        for (size_t i = 0; i < kVPCount; ++i) {
            traj.now[i] = actual_VP_rad_[i];
            auto &optimizer = cos_optimizers_[i];
            const bool hold = isFinished_ || fabsf(traj.delat_traj(i)) <= 1e-6f ||
                optimizer.traj_param_.t >= traj.T_s;

            optimizer.ConfigImprovement(hold ? ControlOptimizer::Control_Improvement::NONE
                : ControlOptimizer::Control_Improvement::COSPLAN);
            optimizer.Set(traj.target[i], traj.now[i], traj.T_s);
            optimizer.Run(traj.now[i]);
            
            output_VP_speed_rad_[i] = optimizer.GetPosOutput();
            traj.ref[i] = hold ? traj.target[i] : optimizer.GetPosRef();
            output_ff_[i] = hold ? 0.0f : optimizer.output_ff_;
            output_pid_[i] = optimizer.output_pid_;
            error_VP_rad_[i] = traj.ref[i] - traj.now[i];
            elapsed = std::min(elapsed, optimizer.traj_param_.t);
        }
        traj.t_s = elapsed;
        isFinished_ = elapsed >= traj.T_s;
        output_motor_speed_rad_s_[kLeft] = output_VP_speed_rad_[kPitch]
            + kYawGearRatio * output_VP_speed_rad_[kYaw];
        output_motor_speed_rad_s_[kRight] = output_VP_speed_rad_[kPitch]
            - kYawGearRatio * output_VP_speed_rad_[kYaw];
    }
    /********硬同时，会有一点冲击问题**********/
    // void setTrajectory(float pitch, float yaw, float T = 0.0f) {
    //     if (!std::isfinite(pitch) ||
    //         !std::isfinite(yaw) ||
    //         !std::isfinite(T) || T < 0.0f) {
    //         return;
    //     }

    //     for (size_t i = 0; i < kVPCount; ++i) {
    //         if (!std::isfinite(VP_PlanParam_.Acc[i]) ||
    //             !std::isfinite(VP_PlanParam_.Dec[i]) ||
    //             !std::isfinite(VP_PlanParam_.w_Max[i]) ||
    //             VP_PlanParam_.Acc[i] <= 0.0f ||
    //             VP_PlanParam_.Dec[i] <= 0.0f ||
    //             VP_PlanParam_.w_Max[i] <= 0.0f) {
    //             return;
    //         }
    //     }

    //     const float pitch_rad =
    //         std::clamp(pitch, kPitchMin, kPitchMax) * kDegToRad;
    //     const float yaw_rad =
    //         std::clamp(yaw, kYawMin, kYawMax) * kDegToRad;

        
    //     if(fabsf(pitch_rad - VP_TrajectParam_.target[kPitch]) <= 1e-6f &&
    //         fabsf(yaw_rad - VP_TrajectParam_.target[kYaw]) <= 1e-6f)
    //     {   

    //         return;
    //     }
        

    //     target_VP_rad_[kPitch] = pitch_rad;
    //     target_VP_rad_[kYaw] = yaw_rad;

    //     getVP_Pos();

    //     bool moving = false;
    //     for (size_t i = 0; i < kVPCount; ++i) {
    //         VP_TrajectParam_.start[i] = actual_VP_rad_[i];
    //         VP_TrajectParam_.now[i] = actual_VP_rad_[i];
    //         VP_TrajectParam_.ref[i] = actual_VP_rad_[i];
    //         VP_TrajectParam_.target[i] = target_VP_rad_[i];

    //         moving |= fabsf(VP_TrajectParam_.delat_traj(i)) > 1e-6f;

    //         output_pid_[i] = 0.0f;
    //     }

    //     VP_TrajectParam_.T = T;
    //     VP_TrajectParam_.t = 0.0f;
    //     VP_TrajectParam_.auto_calculate_T = (T == 0.0f);

    //     VP_TrajectParam_.u = moving ? 0.0f : 1.0f;
    //     VP_TrajectParam_.u_speed = 0.0f;

    //     isFinished_ = !moving;
    // }

    // void PlanSpeedSolution() {
    //     if (!std::isfinite(dt_s_) || dt_s_ <= 0.0f) {
    //         return;
    //     }

    //     auto &traj = VP_TrajectParam_;

    //     // 实际位置由getVP_Pos()更新。
    //     for (size_t i = 0; i < kVPCount; ++i) {
    //         traj.now[i] = actual_VP_rad_[i];
    //     }

    //     if (!isFinished_) {
    //         float Acc = INFINITY;
    //         float Dec = INFINITY;
    //         float w_Max = INFINITY;

    //         for (size_t i = 0; i < kVPCount; ++i) {
    //             const float distance = fabsf(traj.delat_traj(i));

    //             if (distance <= 1e-6f)
    //                 continue;

    //             Acc = std::min(Acc, VP_PlanParam_.Acc[i] / distance);
    //             Dec = std::min(Dec, VP_PlanParam_.Dec[i] / distance);
    //             w_Max = std::min(
    //                 w_Max, VP_PlanParam_.w_Max[i] / distance);
    //         }

    //         // 计算理想连续梯形的最短时间。
    //         const float peak = std::min(
    //             w_Max,
    //             sqrtf(2.0f / (1.0f / Acc + 1.0f / Dec)));

    //         const float ta = peak / Acc;
    //         const float td = peak / Dec;
    //         const float tc = std::max(
    //             0.0f, (1.0f - 0.5f * peak * (ta + td)) / peak);

    //         const float minimum_T = ta + tc + td;

    //         const float scale = traj.auto_calculate_T
    //             ? 1.0f
    //             : std::max(1.0f, traj.T / minimum_T);

    //         traj.T = minimum_T * scale;

    //         Acc /= scale * scale;
    //         Dec /= scale * scale;
    //         w_Max /= scale;

    //         const float previous_speed = traj.u_speed;

    //         const float w_Acc =
    //             previous_speed + Acc * dt_s_;

    //         const float w_Dec = sqrtf(
    //             2.0f * Dec * fmaxf(0.0f, 1.0f - traj.u));

    //         traj.u_speed =
    //             std::min(w_Max, std::min(w_Acc, w_Dec));

    //         traj.u +=
    //             0.5f * (previous_speed + traj.u_speed) * dt_s_;

    //         traj.t += dt_s_;

    //         if (traj.u >= 1.0f) {
    //             traj.u = 1.0f;
    //             traj.u_speed = 0.0f;
    //             isFinished_ = true;
    //         }
    //     }

    //     for (size_t i = 0; i < kVPCount; ++i) {
    //         const float delta = traj.delat_traj(i);

    //         traj.ref[i] = traj.start[i] + delta * traj.u;
            
    //         output_ff_[i] = delta * traj.u_speed;

    //         error_VP_rad_[i] = traj.ref[i] - traj.now[i];

    //         output_pid_[i] = PID_Calculate(
    //             &VP_pid_[i], traj.now[i], traj.ref[i]);

    //         output_VP_speed_rad_[i] =
    //             output_ff_[i] + output_pid_[i];
    //     }

    //     // 两轴角速度 → 左右驱动轴角速度。
    //     output_motor_speed_rad_s_[kLeft] =
    //         output_VP_speed_rad_[kPitch]
    //         + kYawGearRatio * output_VP_speed_rad_[kYaw];

    //     output_motor_speed_rad_s_[kRight] =
    //         output_VP_speed_rad_[kPitch]
    //         - kYawGearRatio * output_VP_speed_rad_[kYaw];
    // }

    /*********软同时T，三角区可能有问题***********/
    // void setTrajectory(float pitch, float yaw, float T_s = 0.0f) {
    //     if (!std::isfinite(pitch) || !std::isfinite(yaw) ||
    //         !std::isfinite(T_s) || T_s < 0.0f) return;

    //     const float targets[kVPCount]{
    //         std::clamp(pitch, kPitchMin, kPitchMax) * kDegToRad,
    //         std::clamp(yaw, kYawMin, kYawMax) * kDegToRad};
    //     auto &traj = VP_TrajectParam_;
    //     if (fabsf(targets[kPitch] - traj.target[kPitch]) <= 1e-6f &&
    //         fabsf(targets[kYaw] - traj.target[kYaw]) <= 1e-6f) return;

    //     TrajectParam candidate{};
    //     float common_T_s = std::max(T_s, 0.01f);
    //     for (size_t i = 0; i < kVPCount; ++i) {
    //         const float a = VP_PlanParam_.Acc[i];
    //         const float d = VP_PlanParam_.Dec[i];
    //         const float vmax = VP_PlanParam_.w_Max[i];
    //         if (!std::isfinite(a) || !std::isfinite(d) ||
    //             !std::isfinite(vmax) || a <= 0.0f || d <= 0.0f || vmax <= 0.0f ||
    //             !std::isfinite(traj.ref[i]) || !std::isfinite(output_ff_[i]) ||
    //             fabsf(output_ff_[i]) > vmax) return;

    //         candidate.start[i] = traj.ref[i];
    //         candidate.ref[i] = traj.ref[i];
    //         candidate.now[i] = actual_VP_rad_[i];
    //         candidate.target[i] = targets[i];
    //         candidate.start_speed[i] = output_ff_[i];
    //         common_T_s = std::max(common_T_s, fabsf(targets[i] - traj.ref[i]) / vmax);
    //     }

    //     const float pitch_T_s = calculatemintime(candidate, kPitch);
    //     const float yaw_T_s = calculatemintime(candidate, kYaw);
    //     if (pitch_T_s < 0.0f || yaw_T_s < 0.0f) return;
    //     common_T_s = std::max(common_T_s, std::max(pitch_T_s, yaw_T_s));

    //     // Plan once; a failed request leaves the active trajectory untouched.
    //     for (size_t i = 0; i < kVPCount; ++i) {
    //         if (!planAxisForDuration(candidate, i, common_T_s)) return;
    //     }
    //     candidate.T_s = common_T_s;
    //     candidate.t_s = 0.0f;
    //     candidate.auto_calculate_T = (T_s == 0.0f);
    //     traj = candidate;
    //     for (size_t i = 0; i < kVPCount; ++i) {
    //         target_VP_rad_[i] = targets[i];
    //     }
    //     isFinished_ = false;
    // }

    // void PlanSpeedSolution() {
    //     if (!std::isfinite(dt_s_) || dt_s_ <= 0.0f) return;
    //     auto &traj = VP_TrajectParam_;
    //     if (!isFinished_) {
    //         traj.t_s = std::min(traj.t_s + dt_s_, traj.T_s);
    //         for (size_t i = 0; i < kVPCount; ++i) {
    //             if (traj.t_s >= traj.T_s) {
    //                 traj.ref[i] = traj.target[i];
    //                 output_ff_[i] = 0.0f;
    //                 continue;
    //             }
    //             const float v0 = traj.direction[i] * traj.start_speed[i];
    //             float positionfromstart = 0.0f;
    //             float speed;
    //             if (traj.t_s < traj.ta[i]) {
    //                 speed = v0 + traj.acc[i] * traj.t_s;
    //                 positionfromstart = v0 * traj.t_s + 0.5f * traj.acc[i] * traj.t_s * traj.t_s;
    //             } else if (traj.t_s < traj.ta[i] + traj.tc[i]) {
    //                 speed = traj.peak[i];
    //                 positionfromstart = 0.5f * (v0 + traj.peak[i]) * traj.ta[i]
    //                                   + traj.peak[i] * (traj.t_s - traj.ta[i]);
    //             } else {
    //                 const float remaining = traj.T_s - traj.t_s;
    //                 speed = traj.dec[i] * remaining;
    //                 positionfromstart = traj.direction[i] * (traj.target[i] - traj.start[i])
    //                                    - 0.5f * traj.dec[i] * remaining * remaining;
    //             }
    //             traj.ref[i] = traj.start[i] + traj.direction[i] * positionfromstart;
    //             output_ff_[i] = traj.direction[i] * speed;
    //         }
    //         isFinished_ = traj.t_s >= traj.T_s;
    //     }

    //     for (size_t i = 0; i < kVPCount; ++i) {
    //         traj.now[i] = actual_VP_rad_[i];
    //         error_VP_rad_[i] = traj.ref[i] - traj.now[i];
    //         output_pid_[i] = PID_Calculate(
    //             &VP_pid_[i], traj.now[i], traj.ref[i]);
    //         output_VP_speed_rad_[i] = output_ff_[i] + output_pid_[i];
    //     }
    //     output_motor_speed_rad_s_[kLeft] = output_VP_speed_rad_[kPitch]
    //         + kYawGearRatio * output_VP_speed_rad_[kYaw];
    //     output_motor_speed_rad_s_[kRight] = output_VP_speed_rad_[kPitch]
    //         - kYawGearRatio * output_VP_speed_rad_[kYaw];
    // }

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

        float T_s{};
        float t_s{};        
        bool auto_calculate_T{true};

        float u{};
        float u_speed{};

        float start_speed[kVPCount]{};
        float direction[kVPCount]{};
        float peak[kVPCount]{};
        float acc[kVPCount]{};
        float dec[kVPCount]{};
        float ta[kVPCount]{};
        float tc[kVPCount]{};
        float td[kVPCount]{};

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

    // Calculate the minimum duration with the current initial speed.
    // float calculatemintime(const TrajectParam &candidate, size_t i) const {
    //     float minimum_T = -1.0f;
        
    //     const float start = candidate.start[i];
    //     const float target = candidate.target[i];
    //     const float initial_speed = candidate.start_speed[i];
    //     const float L_limit = (i == kPitch ? kPitchMin : kYawMin) * kDegToRad;
    //     const float H_limit = (i == kPitch ? kPitchMax : kYawMax) * kDegToRad;
    //     if (start < L_limit || start > H_limit || target < L_limit || target > H_limit)
    //     {
    //         return minimum_T;
    //     }

        
    //     if (fabsf(target - start) <= 1e-8f && fabsf(initial_speed) <= 1e-8f)
    //     {
    //         minimum_T = 0.0f;
    //         return minimum_T;
    //     }

    //     const float direction = target >= start ? 1.0f : -1.0f;
    //     const float distance = direction * (target - start);
    //     const float v0 = direction * initial_speed;
    //     const float Acc = v0 < 0.0f ? std::min(VP_PlanParam_.Acc[i], VP_PlanParam_.Dec[i]) : VP_PlanParam_.Acc[i];
    //     const float Dec = VP_PlanParam_.Dec[i];
    //     const float vmax = VP_PlanParam_.w_Max[i];
    //     if (v0 < 0.0f) {
    //         const float native_distance = start - direction * v0 * v0 / (2.0f * Acc);
    //         if (native_distance < L_limit || native_distance > H_limit) return minimum_T;
    //     }

    //     const float peak_squared =
    //         (distance + v0 * v0 / (2.0f * Acc)) /
    //         (0.5f * (1.0f / Acc + 1.0f / Dec));
    //     if (peak_squared < 0.0f) return minimum_T;
    //     const float unconstrained_peak = sqrtf(peak_squared);
    //     const float peak = std::min(unconstrained_peak, vmax);
    //     if (peak < v0) return minimum_T;
    //     const float ta = (peak - v0) / Acc;
    //     const float td = peak / Dec;
    //     float tc = 0.0f;
    //     if (unconstrained_peak > vmax) {
    //         const float ramp_distance =
    //             (peak * peak - v0 * v0) / (2.0f * Acc) +
    //             peak * peak / (2.0f * Dec);
    //         tc = std::max(0.0f, (distance - ramp_distance) / peak);
    //     }
    //     const float duration = ta + tc + td;
    //     if (std::isfinite(duration) && (minimum_T < 0.0f || duration < minimum_T))
    //     {   
    //         minimum_T = duration;
    //     }
    //     return minimum_T;
    // }

    // bool planAxisForDuration(TrajectParam &candidate, size_t i, float common_T_s) const {
    //     const float start = candidate.start[i];
    //     const float target = candidate.target[i];
    //     const float initial_speed = candidate.start_speed[i];
    //     const float L_limit = (i == kPitch ? kPitchMin : kYawMin) * kDegToRad;
    //     const float H_limit = (i == kPitch ? kPitchMax : kYawMax) * kDegToRad;
    //     if (start < L_limit || start > H_limit || target < L_limit || target > H_limit)
    //         return false;
    //     if (fabsf(target - start) <= 1e-8f && fabsf(initial_speed) <= 1e-8f) {
    //         candidate.direction[i] = 1.0f;
    //         candidate.peak[i] = candidate.acc[i] = candidate.dec[i] = 0.0f;
    //         candidate.ta[i] = candidate.td[i] = 0.0f;
    //         candidate.tc[i] = common_T_s;
    //         return true;
    //     }
    //     const float preferred = target >= start ? 1.0f : -1.0f;
    //     for (unsigned attempt = 0; attempt < 2; ++attempt) {
    //         const float direction = attempt == 0 ? preferred : -preferred;
    //         const float distance = direction * (target - start);
    //         const float v0 = direction * initial_speed;
    //         const float forward_acc = v0 < 0.0f
    //             ? std::min(VP_PlanParam_.Acc[i], VP_PlanParam_.Dec[i])
    //             : VP_PlanParam_.Acc[i];
    //         const float Dec = VP_PlanParam_.Dec[i];
            
    //         for (unsigned ramp = 0; ramp < 2; ++ramp) {
    //             const float Acc = ramp == 0 ? forward_acc : -Dec;
    //             const float A = 0.5f * (1.0f / Acc + 1.0f / Dec);
    //             const float B = common_T_s + v0 / Acc;
    //             const float C = distance + v0 * v0 / (2.0f * Acc);
    //             if (A == 0.0f && B == 0.0f && fabsf(C) > 1e-7f) continue;
    //             const float discriminant = B * B - 4.0f * A * C;
    //             if (discriminant < -1e-6f * (B * B + fabsf(4.0f * A * C))) continue; // 超出浮点误差范围，判定无解
    //             const float root = sqrtf(std::max(0.0f, discriminant)); //微小负误差按0处理，避免开方得到NaN
    //             const float denominator = B + root;
    //             float peak = A == 0.0f
    //                 ? (B != 0.0f ? C / B : 0.0f)
    //                 : (denominator != 0.0f
    //                     ? 2.0f * C / denominator : (B - root) / (2.0f * A));
    //             if (!std::isfinite(peak) || peak < 0.0f ||
    //                 peak > VP_PlanParam_.w_Max[i] * (1.0f + 1e-6f)) continue;   //速度上限允许微小浮点误差
    //             peak = std::min(peak, VP_PlanParam_.w_Max[i]);  //消除误差造成的微小超速
    //             float ta = (peak - v0) / Acc;
    //             const float td = peak / Dec;
    //             const float time_tolerance = 1e-6f * common_T_s;  //时间容差，单位s
    //             if (ta < -time_tolerance) continue;     //加速时间明显为负，当前方案不可行
    //             ta = std::max(0.0f, ta);    // 微小负时间按0处理
    //             const float tc = common_T_s - ta - td;
    //             if (tc < -time_tolerance) continue;     //匀速时间明显为负，共同时间不足
    //             if (v0 < 0.0f) {
    //                 const float native_distance = start - direction * v0 * v0 / (2.0f * Acc);
    //                 if (native_distance < L_limit || native_distance > H_limit) continue;
    //             }
    //             candidate.direction[i] = direction;
    //             candidate.peak[i] = peak;
    //             candidate.acc[i] = Acc;
    //             candidate.dec[i] = Dec;
    //             candidate.ta[i] = ta;
    //             candidate.tc[i] = std::max(0.0f, tc); // 消除浮点误差，保证匀速时间非负
    //             candidate.td[i] = td;
    //             return true;
    //         }
    //     }
    //     return false;
    // }

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
