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

template <std::size_t AxisCount>
class SLM
{
    using ValueType = std::conditional_t<
        AxisCount == 1,
        float,
        std::array<float, AxisCount>
    >;

public:
    enum ControlMode : size_t {SPEEDPLAN = 0, INTERPOLATION = 1};
    struct LiftLimit
    {
        float min;
        float max;
    };
    
    SLM(const std::array<LinearAxis*, AxisCount> &motors)
    : motors_(motors){};

    void init(float Limit_min, float Limit_max, PID_t &path_pid, typename ControlOptimizer<AxisCount>::PlanParam &plan,
        ControlMode controlmode = SPEEDPLAN,
        typename ControlOptimizer<AxisCount>::Control_Improvement control_improvement = ControlOptimizer<AxisCount>::Control_Improvement::COSPLAN)
    {
        optimizer_.ConfigImprovement(control_improvement);
        controlmode_ = controlmode;
        ConfigLimit(Limit_min, Limit_max);
        ConfigOptimizer(path_pid, plan);
    }
    void updateFeedback(){
        for (size_t i = 0; i < motors_.size(); ++i) {
            current_SLM_meter_[i] = motors_[i]->getAxisPos_meter();
            current_SLM_Speed_meter_s_[i] = motors_[i]->getAxisSpeed_meter();
        }
    }

    void setTrajectory(float lift_meter, std::optional<float> common_duration_s = std::nullopt)
    {
        lift_meter = std::clamp(lift_meter, liftlimit_.min, liftlimit_.max);

        for(size_t i = 0;i < AxisCount; ++i)
        {
            target_SLM_meter_[i] = lift_meter;
        }
        updateFeedback();

        optimizer_.Set(target_SLM_meter_, current_SLM_meter_, common_duration_s); 
    }

    void PlanSpeedSolution()
    {
        updateFeedback();
        // optimizer_.Set(target_SLM_meter_, current_SLM_meter_);         
        optimizer_.Run();
        output_SLM_speed_meter_s_ = optimizer_.GetSpeedOutput();
    }

    void PlanInterpolationSolution() 
    {
        updateFeedback();
        // optimizer_.Set(target_SLM_meter_, current_SLM_meter_);
        optimizer_.Run();
        output_SLM_pos_meter_ = optimizer_.GetPosRef();           
            
    }

    void stop()
    {
        updateFeedback();
        for (size_t i = 0; i < motors_.size(); ++i) {
            output_SLM_pos_meter_[i] = current_SLM_meter_[i];
        }
        for (size_t i = 0; i < motors_.size(); ++i) {
                    motors_[i]->setAxisPos_meter(output_SLM_pos_meter_[i]);
        }
    }

    void PlanCommand()
    {
        switch (controlmode_) {
            case SPEEDPLAN:
                PlanSpeedSolution();
                for (size_t i = 0; i < motors_.size(); ++i) {
                    motors_[i]->setAxisSpeed_meter_s(output_SLM_speed_meter_s_[i]);
                }
                break;
            case INTERPOLATION:
                PlanInterpolationSolution();
                for (size_t i = 0; i < motors_.size(); ++i) {
                    motors_[i]->setAxisPos_meter(output_SLM_pos_meter_[i]);
                }
                break;
            default :
                stop();
                break;
        }
    }
    
    void ConfigControlPlan(ControlMode controlmode = SPEEDPLAN, 
                            typename ControlOptimizer<AxisCount>::Control_Improvement control_improvement = ControlOptimizer<AxisCount>::Control_Improvement::COSPLAN)
    {
        controlmode_ = controlmode;
        optimizer_.ConfigImprovement(control_improvement); 
    }

    void ConfigLimit(float lift_meter_min, float lift_meter_max)
    {
        liftlimit_.min = lift_meter_min;
        liftlimit_.max = lift_meter_max;

    }

    void ConfigOptimizer(PID_t &path_pid, typename ControlOptimizer<AxisCount>::PlanParam &plan)
    {
        optimizer_.ConfigPid(path_pid);         
        optimizer_.ConfigPlanParam(plan);
    }

    bool isFinished(){ return optimizer_.Getfinish();}
private:
    std::array<LinearAxis*, AxisCount> motors_{};
 
    LiftLimit liftlimit_ = {0.0f,1.0f};


    std::array<float, AxisCount> current_SLM_meter_{};
    std::array<float, AxisCount> current_SLM_Speed_meter_s_{};
    std::array<float, AxisCount> target_SLM_meter_{};

    std::array<float, AxisCount> output_SLM_pos_meter_{};
    std::array<float, AxisCount> output_SLM_speed_meter_s_{};

    //规划
    ControlOptimizer<AxisCount> optimizer_;
    ControlMode controlmode_ = SPEEDPLAN;


};