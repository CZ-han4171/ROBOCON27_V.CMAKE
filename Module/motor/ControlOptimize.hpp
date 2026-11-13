/**
 * @file ControlOptimizer.hpp
 * @author CZhan
 * @brief
 * @version 1.0
 * @date 2026-11-1
 *
 * @copyright
 *
 * @attention :
 * @note :
 * @versioninfo :
 */
#pragma once

#include "Canbus.hpp"
#include "Encoder.hpp"
#include "main.h"
#include <cmath>
#include <cstdint>
#include "optional"
#include <type_traits>
#include <stdatomic.h>
#include <stdint.h>
#include "bsp_dwt.h"
#include "Motor.hpp"
#include "pid_controller.hpp"
#include "tool.hpp"
#include "topic_pool.h"
#include <functional>

template <std::size_t dimensions>
class ControlOptimizer {
public:
    using ValueType = std::conditional_t<
        dimensions == 1,
        float,
        std::array<float, dimensions>
    >;
    
    enum class Control_Improvement{
        NONE,
        SPEEDPLAN,
        COSPLAN,
        POLYNOMIAL
    };
    struct PlanParam {
        float Acc;
        float Dec;
        float w_Max;

        PlanParam(float Acc_in = M_PI, float Dec_in = M_PI, float w_Max_in = 10*M_PI)
            :Acc(Acc_in), Dec(Dec_in), w_Max(w_Max_in){}
    };
    struct TrajectParam {
        float ref{0};
        float start{0};
        float traget{0};
        float now{0};
        float T{0};
        float t{0};
        bool auto_calculate_T = true;

        float delat_traj(){ return traget - start; }
        float delat_now(){ return traget - now; }
        float delat_ref(){ return traget - ref; }
        float delat_now_start(){ return now - start; }
    };

    void ConfigImprovement( Control_Improvement Improvement){
        Improvement_ = Improvement;
    }
    void ConfigPlanParam(const PlanParam &plan_param){
        plan_param_ = plan_param;
    }
    void ConfigPid(PID_t pid_in){
        track_path_pid_ = pid_in;
        for(std::size_t i = 0; i < dimensions; ++i){
            track_lateral_pid_[i] = pid_in;
        }
    }
    bool is_Change(std::array<float, dimensions> now, std::array<float, dimensions> last){
        for(std::size_t i = 0; i < dimensions; ++i){
            if(fabs(now[i] - last[i]) >= 0.001f)return true;
        }
        return false;
    }
    bool is_lateral_finish(std::array<float, dimensions> correction){
        for(std::size_t i = 0; i < dimensions; ++i){
            if(fabs(correction[i]) >= 0.01f)return false;
        }
        return true;
    }
    void init(std::array<float, dimensions> &traget_last){
        if(!is_init){
            for(std::size_t i = 0; i < dimensions; ++i){
                traget_last[i] = traj_param_components_[i].now;
            }
            is_init = !is_init;
        }
    }
    void Set(ValueType traget_pos, ValueType actual_pos,
            std::optional<float> T = std::nullopt){
        std::array<float, dimensions> traget_now{0};
        std::array<float, dimensions> traget_last{0};
        float magnitude_tmp;
        
        if constexpr (dimensions == 1) {
            traj_param_components_[0].traget = traget_pos;
            traj_param_components_[0].now = actual_pos;
            traget_now[0] = traj_param_components_[0].traget;
            traget_last[0] = traj_param_components_last_[0].traget;
            magnitude_tmp = fabs(traj_param_components_[0].delat_now());
        }else{
            float sum_sq = 0.0f;
            for(std::size_t i = 0; i < dimensions; ++i){
                traj_param_components_[i].traget = traget_pos[i];
                traj_param_components_[i].now = actual_pos[i];
                traget_now[i] = traj_param_components_[i].traget;
                traget_last[i] = traj_param_components_last_[i].traget;
                sum_sq += (traj_param_components_[i].delat_now())*(traj_param_components_[i].delat_now());
            }
            magnitude_tmp = sqrtf(sum_sq);
        }
        // if(magnitude_tmp < 0.001f)magnitude_tmp = 0.001f;

        if(!is_init || is_Change(traget_now,traget_last)){
            is_init = true;
            is_point_hold_ = magnitude_tmp < 0.001f;

            for(std::size_t i = 0; i < dimensions; ++i){
                traj_param_components_[i].start = traj_param_components_[i].now;
                traj_param_components_[i].ref = traj_param_components_[i].now;
                unit_direction_[i] = is_point_hold_ ? 0.0f : traj_param_components_[i].delat_traj()/magnitude_tmp;
            }

            is_finish = false;
            traj_param_magnitude_.start = 0.0f;
            traj_param_magnitude_.ref = 0.0f;
            traj_param_magnitude_.t = 0.0f;
            traj_param_magnitude_.traget = magnitude_tmp;
            if(T.has_value()){
                traj_param_magnitude_.auto_calculate_T = false;
                traj_param_magnitude_.T = *T;
            }else {
                traj_param_magnitude_.auto_calculate_T = true;
            }
        }
    }

    void Run(){
        if constexpr (dimensions == 1) {
            traj_param_magnitude_.now = unit_direction_[0]*traj_param_components_[0].delat_now_start();
        }else{
            float projection = 0.0f; // 点积
            for(std::size_t i = 0; i < dimensions; ++i){
                projection += unit_direction_[i]*traj_param_components_[i].delat_now_start();
            }
            correction_ = vectorToLine(projection);
            traj_param_magnitude_.now = projection;
        }

        if(is_point_hold_){
            output_path_ff_ = 0.0f;
            if constexpr (dimensions == 1){
                pos_output_components_[0] = PID_Calculate(&track_lateral_pid_[0], traj_param_components_[0].now, traj_param_components_[0].traget);
            }else{
                for(std::size_t i = 0; i < dimensions; ++i){
                    pos_output_components_[i] = PID_Calculate(&track_lateral_pid_[i], traj_param_components_[i].now, traj_param_components_[i].traget);
                }
            }
        }else {
            dt = DWT_GetDeltaT(&DWT_CNT);
            if(dt > 0.01f) dt = 0.01f;
            traj_param_magnitude_.t += dt;
            pos_output_path_ = ControlImprovementChoose(Improvement_);
            for(std::size_t i = 0; i < dimensions; ++i){
                if(dimensions != 1)output_lateral_pid_[i] = PID_Calculate(&track_lateral_pid_[i],0,correction_[i]);
                else output_lateral_pid_[i] = 0.0f;
                pos_output_components_[i] = pos_output_path_*unit_direction_[i] + output_lateral_pid_[i];
                traj_param_components_[i].ref = traj_param_components_[i].start + traj_param_magnitude_.ref*unit_direction_[i];
            }    
        }
        
        traj_param_magnitude_last_ = traj_param_magnitude_;
        traj_param_components_last_ = traj_param_components_;

        is_finish = true;
        for(std::size_t i = 0; i < dimensions; ++i){
            const float error = traj_param_components_[i].delat_now();
            const float speed = pos_output_components_[i];

            if (!std::isfinite(error) || !std::isfinite(speed) ||
                fabsf(error) >= 0.01f || fabsf(speed) >= 0.01f) {
                is_finish = false;
                break;
            }
        }
    }

    std::array<float, dimensions> vectorToLine(const float projection){
        std::array<float, dimensions> correction{};
        for(std::size_t i = 0; i < dimensions; ++i){
            correction[i] = projection*unit_direction_[i] - traj_param_components_[i].delat_now_start();
        }
        return correction;
    }

    ValueType GetSpeedOutput(){
        if constexpr (dimensions == 1) {
            return pos_output_components_[0];
        }else{
            return pos_output_components_; 
        }
    }
    ValueType GetPosRef(){
        if constexpr (dimensions == 1){
            return traj_param_components_[0].ref;
        }else {
            std::array<float, dimensions> result{};
            for(std::size_t i = 0; i < dimensions; ++i){
                result[i] = traj_param_components_[i].ref;
            }
            return result;
        }
    }
    bool Getfinish(){ return is_finish; }
   
private:

    float ControlImprovementChoose(Control_Improvement Improvement){
        switch (Improvement) {
            case Control_Improvement::NONE :
                return PointTrackSolution(output_path_pid_,traj_param_magnitude_);

            case Control_Improvement::SPEEDPLAN :
                return PlanSpeedSolution(output_path_ff_,output_path_pid_,traj_param_magnitude_);

            case Control_Improvement::COSPLAN : 
                return CosSpeedSolution(output_path_ff_,output_path_pid_,traj_param_magnitude_);

            case Control_Improvement::POLYNOMIAL :
                return QuinticPolynomialSpeedSolution(output_path_ff_,output_path_pid_,traj_param_magnitude_);

            default:break;
        }
        return 0;
    }
    float PointTrackSolution(float &output_pid, TrajectParam &traj_param){
        output_pid = PID_Calculate(&track_path_pid_,traj_param.now,traj_param.traget);
        traj_param.ref = traj_param.now;
        return output_pid;
    }
    float PlanSpeedSolution(float &output_ff, float &output_pid, TrajectParam &traj_param){
        float previous_speed = output_ff;
        // float delat_ref = traget_pos - pos_ref;
        float w_Acc = output_ff + sign(traj_param.delat_ref())*plan_param_.Acc*dt;
        float w_Dec = sign(traj_param.delat_ref())*sqrt(2.0f*plan_param_.Dec*fabs(traj_param.delat_ref()));
        if(plan_param_.w_Max < fabs(w_Acc) && plan_param_.w_Max < fabs(w_Dec)){
            output_ff = sign(traj_param.delat_ref())*plan_param_.w_Max;
        }else if(fabs(w_Acc) > fabs(w_Dec) && sign(w_Acc)==sign(w_Dec)){
            output_ff = w_Dec;
        }else {
            output_ff = w_Acc;
        };
        traj_param.ref += 0.5f * (previous_speed + output_ff) * dt;
        output_pid = PID_Calculate(&track_path_pid_,traj_param.now,traj_param.ref);
        return output_ff + output_pid;
    }
    float CosSpeedSolution(float &output_ff, float &output_pid, TrajectParam &traj_param){
        if(traj_param.auto_calculate_T)traj_param.T = fabs(traj_param.delat_traj()*M_PI*sinf(M_PI*0.5f)/(2.0f*plan_param_.w_Max));
    
        float s_now = traj_param.delat_now()/traj_param.delat_traj();
        s_now = ( s_now > 1.0f ) ? 1.0f : s_now ;
        float s_ref = traj_param.t/traj_param.T ;
        s_ref = ( s_ref > 1.0f ) ? 1.0f : s_ref ;
        traj_param.ref = traj_param.start + traj_param.delat_traj()*( 1 - cosf(M_PI*s_ref))/2.0f;

        if(traj_param.T > 0.001f)output_ff = traj_param.delat_traj()*M_PI*sinf(M_PI*s_now)/(2.0f*traj_param.T);
        else output_ff = 0.0f;
        output_pid = PID_Calculate(&track_path_pid_,traj_param.now,traj_param.ref);

        return output_ff + output_pid;
    }
    float QuinticPolynomialSpeedSolution(float &output_ff, float &output_pid, TrajectParam &traj_param){
        if(traj_param.auto_calculate_T)traj_param.T = fabs(traj_param.delat_traj()*( 30.0f*0.5f*0.5f - 60.0f*0.5f*0.5f*0.5f + 30.0f*0.5f*0.5f*0.5f*0.5f )/plan_param_.w_Max);
 
        float s_now = traj_param.delat_now()/traj_param.delat_traj();
        s_now = ( s_now > 1.0f ) ? 1.0f : s_now ;
        float s_ref = traj_param.t/traj_param.T ;
        s_ref = ( s_ref > 1.0f ) ? 1.0f : s_ref ;
        traj_param.ref = traj_param.start + traj_param.delat_traj()*( 10.0f*s_ref*s_ref*s_ref - 15.0f*s_ref*s_ref*s_ref*s_ref + 6.0f*s_ref*s_ref*s_ref*s_ref*s_ref );

        if(traj_param.T > 0.001f)output_ff = traj_param.delat_traj()*( 30.0f*s_now*s_now - 60.0f*s_now*s_now*s_now + 30.0f*s_now*s_now*s_now*s_now )/traj_param.T;
        else output_ff = 0.0f;
        output_pid = PID_Calculate(&track_path_pid_,traj_param.now,traj_param.ref);

        return output_ff + output_pid;
    }

    float pos_output_path_{0};
    float output_path_pid_{0};
    float output_path_ff_{0};

    bool is_finish = false;
    bool is_init = false;
    bool is_point_hold_ = false;
    float dt{0};
    uint32_t DWT_CNT{0};

    PID_t track_path_pid_{};
    std::array<PID_t, dimensions> track_lateral_pid_{};

    std::array<float, dimensions> pos_output_components_{0};
    std::array<float, dimensions> unit_direction_{0};
    std::array<float, dimensions> correction_{0};
    std::array<float, dimensions> output_lateral_pid_{0};
    PlanParam plan_param_;
    TrajectParam traj_param_magnitude_;
    TrajectParam traj_param_magnitude_last_;
    std::array<TrajectParam, dimensions> traj_param_components_;
    std::array<TrajectParam, dimensions> traj_param_components_last_;
    Control_Improvement Improvement_{Control_Improvement::NONE};
};