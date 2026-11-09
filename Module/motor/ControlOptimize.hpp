/**
 * @file MotorControlOptimizer.hpp
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
#include <stdatomic.h>
#include <stdint.h>
#include "bsp_dwt.h"
#include "Motor.hpp"
#include "pid_controller.hpp"
#include "tool.hpp"
#include "topic_pool.h"
#include <functional>



class ControlOptimizer {
public:
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
        float ref;
        float start;
        float traget;
        float now;
        float T;
        float t;
        bool auto_calculate_T = true;

        float delat_traj(){ return traget - start; }
        float delat_now(){ return traget - now; }
        float delat_ref(){ return traget - ref; }
    };

    void ConfigImprovement( Control_Improvement Improvement){
        Improvement_ = Improvement;
    }
    void ConfigPlanParam(const PlanParam &plan_param){
        plan_param_ = plan_param;
    }
    // void Init(float actual_pos){
    //     traj_param_.ref = actual_pos;
    // }
    void Set(float traget_pos, float actual_pos){
        traj_param_.traget = traget_pos;
        traj_param_.auto_calculate_T = true;
        if(fabs(traj_param_.traget - traj_param_last_.traget) >= 0.001f ){
            is_finish = false;
            traj_param_.start = actual_pos;
            traj_param_.ref = actual_pos;
            traj_param_.t = 0.0f;
        };
    }
    void Set(float traget_pos, float actual_pos, float T){
        traj_param_.traget = traget_pos;
        traj_param_.auto_calculate_T = false;
        if(fabs(traj_param_.traget - traj_param_last_.traget) >= 0.001f ){
            is_finish = false;
            traj_param_.start = actual_pos;
            traj_param_.ref = actual_pos;
            traj_param_.t = 0.0f;
            traj_param_.T = T;
        };
    }
    void Run(float actual_pos){
        traj_param_.now = actual_pos;
        if(fabs( - traj_param_.delat_ref()) < 0.001f && fabs(pos_output_) < 0.001f){
            traj_param_.ref = traj_param_.traget;
            output_ff_ = 0.0f;
            is_finish = true;
        }
        dt = DWT_GetDeltaT(&DWT_CNT);
        if(dt > 0.01f) dt = 0.01f;
        traj_param_.t += dt;

        pos_output_ = ControlImprovementChoose(Improvement_);
        traj_param_last_ = traj_param_;
    }
    float GetPosOutput(){ return pos_output_; }
    float GetPosRef(){ return traj_param_.ref; }

    float ControlImprovementChoose(Control_Improvement Improvement){
        switch (Improvement) {
            case Control_Improvement::NONE :
                return PointTrackSolution(output_pid_,traj_param_);

            case Control_Improvement::SPEEDPLAN :
                return PlanSpeedSolution(output_ff_,output_pid_,traj_param_);

            case Control_Improvement::COSPLAN : 
                return CosSpeedSolution(output_ff_,output_pid_,traj_param_);

            case Control_Improvement::POLYNOMIAL :
                return QuinticPolynomialSpeedSolution(output_ff_,output_pid_,traj_param_);

            default:break;
        }
        return 0;
    }
    float PosPidCalculate(float delat){
        return PID_Calculate(
        &pos_pid_,
        0, // 输出轴角度(rad)
        delat);
    }
    float PointTrackSolution(float &output_pid, TrajectParam &traj_param){
        output_pid = PosPidCalculate(traj_param.traget - traj_param.now);
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
        output_pid = PosPidCalculate(traj_param.ref - traj_param.now);
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
        output_pid = PosPidCalculate(traj_param.ref - traj_param.now);

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
        output_pid = PosPidCalculate(traj_param.ref - traj_param.now);

        return output_ff + output_pid;
    }

    PID_t pos_pid_{};
    float pos_output_;
    float output_pid_;
    float output_ff_;

    bool is_finish = false;
    float dt;
    uint32_t DWT_CNT;

    PlanParam plan_param_;
    TrajectParam traj_param_;
    TrajectParam traj_param_last_;
    Control_Improvement Improvement_;
};