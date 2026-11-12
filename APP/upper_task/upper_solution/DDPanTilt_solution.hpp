#pragma once

#include <algorithm>
#include <array>
#include <cstddef>
#include <type_traits>

#include "Axis.hpp"
#include "ControlOptimize.hpp"
#include "stm32h723xx.h"
#include "topic_pool.h"
#include "tool.hpp"


#define kPitchMax 180.0f
#define kPitchMin 0.0f
#define kYawMax 360.0f
#define kYawMin 0.0f
#define kYawGearRatio 2.0f  // yaw 输出端的额外减速比：电机反向各转2圈，机构yaw转1圈


class DD_PanTilt{
public:
    enum MotorIndex : size_t { kLeft = 0, kRight = 1, kMotorCount = 2 };
    enum VPIndex : size_t { kPitch = 0, kYaw = 1, kVPCount = 2 };
    enum ContorlMode : size_t { INTERPOLATION = 0, SPEEDPLAN = 1};

    DD_PanTilt(RotaryAxis &VPmotor_Left, RotaryAxis &VPmotor_Right, ContorlMode controlmode = SPEEDPLAN, ControlOptimizer::Control_Improvement control_improvement = ControlOptimizer::Control_Improvement::COSPLAN)
        : motors_{&VPmotor_Left, &VPmotor_Right} {
        for (auto &optimizer : optimizers_) {
            optimizer.ConfigImprovement(control_improvement);
            optimizer.traj_param_.auto_calculate_T = false;
        }
        controlmode_ = controlmode;
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

    void run(const pub_VP_cmd &cmd) {
        setTrajectory(cmd.pitch, cmd.yaw, cmd.common_T_s);
        PlanCommand();
    }

    void setTrajectory(float pitch, float yaw, std::optional<float> common_duration_s = std::nullopt)
    {
        const float pitch_rad = std::clamp(pitch, kPitchMin, kPitchMax) * kDegToRad;
        const float yaw_rad = std::clamp(yaw, kYawMin, kYawMax) * kDegToRad;

        target_VP_rad_[kPitch] = pitch_rad;
        target_VP_rad_[kYaw] = yaw_rad;

        getVP_Pos();
        // caculationcommonmintimes();
        // if(common_duration_s.has_value() && *common_duration_s > common_T_s_){ }
        if(common_duration_s.has_value()){
            for (size_t i = 0; i < kVPCount; ++i) {
                optimizers_[i].Set(target_VP_rad_[i], actual_VP_rad_[i], *common_duration_s);
            }
        }    
        else{
            for (size_t i = 0; i < kVPCount; ++i) {
                optimizers_[i].Set(target_VP_rad_[i], actual_VP_rad_[i], common_T_s_);
            }
        }   
    }

    void PlanSpeedSolution()
    {
        for (size_t i = 0; i < kVPCount; ++i) {
            
            optimizers_[i].Run(actual_VP_rad_[i]);
            output_VP_speed_rad_s_[i] = optimizers_[i].GetPosOutput();
            
        }
        output_motor_speed_rad_s_[kLeft] = output_VP_speed_rad_s_[kPitch]
            + kYawGearRatio * output_VP_speed_rad_s_[kYaw];
        output_motor_speed_rad_s_[kRight] = output_VP_speed_rad_s_[kPitch]
            - kYawGearRatio * output_VP_speed_rad_s_[kYaw];
    }

    void PlanInterpolationSolution() {
        for (size_t i = 0; i < kVPCount; ++i) {
            optimizers_[i].Run(actual_VP_rad_[i]);
            output_VP_pos_rad_[i] = optimizers_[i].GetPosRef();           
        }
        output_motor_pos_rad_[kLeft] = output_VP_pos_rad_[kPitch]
            + kYawGearRatio * output_VP_pos_rad_[kYaw];
        output_motor_pos_rad_[kRight] = output_VP_pos_rad_[kPitch]
            - kYawGearRatio * output_VP_pos_rad_[kYaw];
    }

    void stop()
    {
        getVP_Pos();
        output_motor_pos_rad_[kLeft] = actual_VP_rad_[kPitch]
            + kYawGearRatio * actual_VP_rad_[kYaw];
        output_motor_pos_rad_[kRight] = actual_VP_rad_[kPitch]
            - kYawGearRatio * actual_VP_rad_[kYaw];
        for (size_t i = 0; i < motors_.size(); ++i) {
                    motors_[i]->setAxisPos_rad(output_motor_pos_rad_[i]);
        }
    }

    void PlanCommand()
    {
        switch (controlmode_) {
            case SPEEDPLAN:
                PlanSpeedSolution();
                for (size_t i = 0; i < motors_.size(); ++i) {
                    motors_[i]->setAxisSpeed_rad_s(output_motor_speed_rad_s_[i]);
                }
                break;
            case INTERPOLATION:
                PlanInterpolationSolution();
                for (size_t i = 0; i < motors_.size(); ++i) {
                    motors_[i]->setAxisPos_rad(output_motor_pos_rad_[i]);
                }
                break;
            default :
                getVP_Pos();
                stop();
                break;
        }
    }
    
    void ConfigControlPlan(ContorlMode controlmode = SPEEDPLAN, ControlOptimizer::Control_Improvement control_improvement = ControlOptimizer::Control_Improvement::COSPLAN)
    {
        controlmode_ = controlmode;
        for (size_t i = 0; i < kVPCount; ++i) {
            if(optimizers_[i].Improvement_ != control_improvement) {
                optimizers_[i].ConfigImprovement(control_improvement); 
                optimizers_[i].traj_param_last_.traget += 1.0f; //重置 
            }
        }
    }

    // void caculationcommonmintimes()
    // {
    //     switch(optimizers_[kPitch].Improvement_)
    //     {
    //         case ControlOptimizer::Control_Improvement::COSPLAN:
    //             common_T_s_ = std::max(fabs((target_VP_rad_[kPitch] - actual_VP_rad_[kPitch])*M_PI*sinf(M_PI*0.5f)/(2.0f*optimizers_[kPitch].plan_param_.w_Max)), 
    //                                     fabs((target_VP_rad_[kYaw] - actual_VP_rad_[kYaw])*M_PI*sinf(M_PI*0.5f)/(2.0f*optimizers_[kYaw].plan_param_.w_Max)));
    //             break;
    //         case ControlOptimizer::Control_Improvement::POLYNOMIAL:
    //             common_T_s_ = std::max(fabs((target_VP_rad_[kPitch] - actual_VP_rad_[kPitch])*( 30.0f*0.5f*0.5f - 60.0f*0.5f*0.5f*0.5f + 30.0f*0.5f*0.5f*0.5f*0.5f )/optimizers_[kPitch].plan_param_.w_Max), 
    //                                 fabs((target_VP_rad_[kYaw] - actual_VP_rad_[kYaw])*( 30.0f*0.5f*0.5f - 60.0f*0.5f*0.5f*0.5f + 30.0f*0.5f*0.5f*0.5f*0.5f )/optimizers_[kYaw].plan_param_.w_Max));
    //             break;
    //         default:
    //             common_T_s_ = 0.01f;
    //             break;
    //     }
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
    std::array<RotaryAxis *, kMotorCount> motors_{};

    std::array<ControlOptimizer, kVPCount> optimizers_{};

    std::array<float, kMotorCount> current_motor_speed_rad_s_{};
    std::array<float, kMotorCount> output_motor_speed_rad_s_{};
    std::array<float, kMotorCount> current_motor_rad_{};
    std::array<float, kMotorCount> output_motor_pos_rad_{};

    std::array<float, kVPCount> actual_VP_rad_{};
    std::array<float, kVPCount> target_VP_rad_{};


    std::array<float, kVPCount> output_VP_pos_rad_{};
    std::array<float, kVPCount> output_VP_speed_rad_s_{};

    //规划
    bool isFetched_  = false;
    float dt_s_;  // 规划周期
    float common_T_s_ = 0.01f;
    ContorlMode controlmode_ = SPEEDPLAN;
};
