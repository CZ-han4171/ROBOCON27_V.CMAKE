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

class SLM
{
public:
    enum MotorIndex : size_t {kL = 0,kR = 1, kCount = 2};
    enum ControlMode : size_t {SPEEDPLAN = 0, INTERPOLATION = 1};
    struct LiftLimit
    {
        float min;
        float max;
    };
    
    SLM(LinearAxis &L_L, LinearAxis &L_R,ControlMode controlmode = SPEEDPLAN,ControlOptimizer::Control_Improvement control_improvement = ControlOptimizer::Control_Improvement::COSPLAN)
    : motors_{&L_L, &L_R}{
        for (auto &optimizer : optimizers_) {
            optimizer.ConfigImprovement(control_improvement);
            optimizer.traj_param_.auto_calculate_T = false;
        }
        controlmode_ = controlmode;
    };

    void getPos_meter(){
        for (size_t i = 0; i < motors_.size(); ++i) {
            current_BLM_meter_[i] = motors_[i]->getAxisPos_meter();
            current_BLM_Speed_meter_s_[i] = motors_[i]->getAxisSpeed_meter();
        }
    }

    void setTrajectory(float lift_meter, std::optional<float> common_duration_s = std::nullopt)
    {
        lift_meter = std::clamp(lift_meter, liftlimit_.min, liftlimit_.max);

        for(size_t i = 0;i < kCount; ++i)
        {
            target_BLM_meter_[i] = lift_meter;
        }
        getPos_meter();
        if(common_duration_s.has_value()){
            for (size_t i = 0; i < kCount; ++i) {
                optimizers_[i].Set(target_BLM_meter_[i], current_BLM_meter_[i], *common_duration_s);
            }
        }    
        else{
            for (size_t i = 0; i < kCount; ++i) {
                optimizers_[i].Set(target_BLM_meter_[i], current_BLM_meter_[i], common_T_s_);
            }
        }   
    }

    void PlanSpeedSolution()
    {
        getPos_meter();
        for (size_t i = 0; i < kCount; ++i) {            
            optimizers_[i].Run(current_BLM_meter_[i]);
            output_BLM_speed_meter_s_[i] = optimizers_[i].GetPosOutput();
        } 
    }

    void PlanInterpolationSolution() 
    {
        getPos_meter();
        for (size_t i = 0; i < kCount; ++i) {
            optimizers_[i].Run(current_BLM_meter_[i]);
            output_BLM_pos_meter_[i] = optimizers_[i].GetPosRef();           
        }
    }

    void stop()
    {
        getPos_meter();
        for (size_t i = 0; i < motors_.size(); ++i) {
            output_BLM_pos_meter_[i] = current_BLM_meter_[i];
        }
        for (size_t i = 0; i < motors_.size(); ++i) {
                    motors_[i]->setAxisPos_meter(output_BLM_pos_meter_[i]);
        }
    }

    void PlanCommand()
    {
        switch (controlmode_) {
            case SPEEDPLAN:
                PlanSpeedSolution();
                for (size_t i = 0; i < motors_.size(); ++i) {
                    motors_[i]->setAxisSpeed_meter_s(output_BLM_speed_meter_s_[i]);
                }
                break;
            case INTERPOLATION:
                PlanInterpolationSolution();
                for (size_t i = 0; i < motors_.size(); ++i) {
                    motors_[i]->setAxisPos_meter(output_BLM_pos_meter_[i]);
                }
                break;
            default :
                stop();
                break;
        }
    }
    
    void ConfigControlPlan(ControlMode controlmode = SPEEDPLAN, ControlOptimizer::Control_Improvement control_improvement = ControlOptimizer::Control_Improvement::COSPLAN)
    {
        controlmode_ = controlmode;
        for (size_t i = 0; i < kCount; ++i) {
            if(optimizers_[i].Improvement_ != control_improvement) {
                optimizers_[i].ConfigImprovement(control_improvement); 
                optimizers_[i].traj_param_last_.traget += 1.0f; //重置 
            }
        }
    }

    void ConfigLimit(float lift_meter_min, float lift_meter_max)
    {
        liftlimit_.min = lift_meter_min;
        liftlimit_.max = lift_meter_max;
    }

private:
    std::array<LinearAxis*, kCount> motors_{};
 
    LiftLimit liftlimit_ = {0.01f,0.01f};

    std::array<float, kCount> current_BLM_meter_{};
    std::array<float, kCount> current_BLM_Speed_meter_s_{};
    std::array<float, kCount> target_BLM_meter_{};

    std::array<float, kCount> output_BLM_pos_meter_{};
    std::array<float, kCount> output_BLM_speed_meter_s_{};

    //规划
    std::array<ControlOptimizer, kCount> optimizers_{};
    ControlMode controlmode_ = SPEEDPLAN;
    float common_T_s_ = 1.0f;
};