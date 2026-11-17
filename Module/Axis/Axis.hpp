/**
 * @file Axis.hpp
 * @author CZhan
 * @brief
 * @version 1.0
 * @date 2026-10-10
 *
 * @copyright
 *
 * @attention :
 * @note :
 * @versioninfo :
 */
#pragma once

#include "main.h"
#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <stdatomic.h>
#include <stdint.h>
#include "MotorControl.hpp"
#include "ControlOptimize.hpp"
#include "pid_controller.hpp"
#include "tool.hpp"
#include <initializer_list>

#define stall_timeout_s_ 1.0f // 自动归位检测时长
#define unstuck_timeout_s_ 0.05f // 判断电机正常运转时长

template <std::size_t ControlCount,std::size_t MotorCount> // 控制对象个数，电机个数
class AxisBase{
public:
    using ControlFloatArray = std::array<float, ControlCount>;
    using MotorFloatArray   = std::array<float, MotorCount>;

    AxisBase() = default;

    enum ControlOutputMode : size_t {
        SPEEDPLAN,
        INTERPOLATION
    };

    struct PositionControlOptions{
        bool isEnableWrapped{false};
        bool isEnableForbidden_{false};
        float forbiddenPoint_ = M_PI;
    };

    struct TravelLimit
    {
        float min{0};
        float max{0};
    };

    struct AxisStallParam{
        bool is_stall{false};
        float Stall_time{0};
        float Unstall_time{0};
    };

    struct AutoHomeParam{
        MitParam mit_param;
        float Aixs_home_pos{0};
        bool is_auto_home{false};
    };

    void ConfigLimit(std::initializer_list<std::size_t> ControlObject, std::initializer_list<TravelLimit> travel_limit){
        if (ControlObject.size() != travel_limit.size() || ControlObject.size() > ControlCount)return;
        float ControlObject_size = ControlObject.size();
        for(std::size_t i = 0 ; i < ControlObject_size ; ++i){
            if(travel_limit.begin()[i].min >= travel_limit.begin()[i].max) return;
            travel_limit_[ControlObject.begin()[i]] = travel_limit.begin()[i];
            is_limit_[ControlObject.begin()[i]] = true;
        }
    }

    void ConfigOptions(std::initializer_list<std::size_t> ControlObject,
                       std::initializer_list<PositionControlOptions> position_options){
        if (ControlObject.size() != position_options.size() || ControlObject.size() > ControlCount)return;
        float ControlObject_size = ControlObject.size();
        for(std::size_t i = 0 ; i < ControlObject_size ; ++i){
            std::size_t j = ControlObject.begin()[i];
            position_options_[j] = position_options.begin()[i];
            travel_limit_[j] = {-M_PI,M_PI};
            is_limit_[j] = true;
        }
    };

    void ConfigAutoHome(std::initializer_list<std::size_t> ControlObject,
                       std::initializer_list<AutoHomeParam> AutoHomeParams){
        if (ControlObject.size() != AutoHomeParams.size() || ControlObject.size() > ControlCount)return;
        float ControlObject_size = ControlObject.size();
        for(std::size_t i = 0 ; i < ControlObject_size ; ++i){
            std::size_t j = ControlObject.begin()[i];
            auto_home_params_[j] = AutoHomeParams.begin()[i];
            auto_home_params_[j].is_auto_home = true;
            stall_params_[j] = {};
        }
        AutoHome_complete = false;
    }

    void Limit_solve(ControlFloatArray &Axis_pos_in){
        for(std::size_t i = 0 ; i < ControlCount ; ++i){
            if(is_limit_[i])Axis_pos_in[i] = std::clamp(Axis_pos_in[i], travel_limit_[i].min, travel_limit_[i].max);
        }
    };
    
    void PositionOptions_solve(ControlFloatArray &Axis_traget_pos){
        for(std::size_t i = 0 ; i < ControlCount ; ++i){
            if(position_options_[i].isEnableWrapped){
                float delat = 0.0f;
                if(position_options_[i].isEnableForbidden_){
                    position_options_[i].forbiddenPoint_ = Warp_ToRange(position_options_[i].forbiddenPoint_, 0, 2*M_PI);
                    float Axis_traget_pos_forbidden = Warp_ToRange(Axis_traget_pos[i] - position_options_[i].forbiddenPoint_, 0, 2*M_PI);
                    float Axis_actual_pos_forbidden = Warp_ToRange(Axis_actual_pos_[i] - position_options_[i].forbiddenPoint_, 0, 2*M_PI);
                    delat = Axis_traget_pos_forbidden - Axis_actual_pos_forbidden;
                }else{
                    delat = Warp_ToRange(Axis_traget_pos[i] - Axis_actual_pos_[i], -M_PI, M_PI);
                }
                Axis_traget_pos[i] = Axis_actual_pos_[i] + delat;
            }
        }
    }

    void AxisStall_update(){
        std::size_t motor_count = 0;
        for(std::size_t i = 0 ; i < ControlCount ; ++i){
            float estimated_torque = 0.0f;
            float torque = 0.0f;
            for(std::size_t j = 0 ; j < ComtrollerMotorCounts_[i] ; ++j){
                MitParam mit_motor_in = MotorControllers_[motor_count]->getMitParam_In();
                torque += MotorControllers_[motor_count]->getMotorTorque()*
                          motor_rad_per_output_[motor_count];
                estimated_torque += (mit_motor_in.torque + 
                                    mit_motor_in.Kp*(mit_motor_in.pos - MotorControllers_[motor_count]->getMotorSumPos_rad()) + 
                                    mit_motor_in.Kd*mit_motor_in.speed)*
                                    motor_rad_per_output_[motor_count];
                motor_count++;
            }
            if(fabsf(Axis_actual_speed_[i]) < 0.1f && fabsf(torque) > fabsf(estimated_torque)*0.8f){
                stall_params_[i].Unstall_time = 0.0f;
                stall_params_[i].Stall_time += dt;
                if(stall_params_[i].Stall_time >= stall_timeout_s_){
                    stall_params_[i].Stall_time = 0.0f;
                    stall_params_[i].is_stall = true;
                }
            }else {
                stall_params_[i].Unstall_time += dt;
                if(stall_params_[i].Unstall_time >= unstuck_timeout_s_){
                    stall_params_[i].Unstall_time = 0.0f;
                    stall_params_[i].Stall_time = 0.0f;
                    stall_params_[i].is_stall = false;
                }
            }
        }
    };

    bool AutoHome_solve(){
        std::size_t motor_count = 0;
        bool is_all_ready = true;
        if(AutoHome_complete) return false;
        for(std::size_t i = 0 ; i < ControlCount ; ++i ){
            bool is_part_ready = true;
            for(std::size_t j = 0 ; j < ComtrollerMotorCounts_[i] ; ++j){
                if(auto_home_params_[i].is_auto_home){
                    if(stall_params_[i].is_stall){
                        // auto_home_params_[i].is_auto_home = false;
                        MotorControllers_[motor_count]->ResetMotorPos(auto_home_params_[i].Aixs_home_pos*motor_rad_per_output_[motor_count]);
                    }else {
                        std::size_t N = ComtrollerMotorCounts_[i];
                        float r = motor_rad_per_output_[motor_count];
                        MitParam mit_motor_in;
                        mit_motor_in.torque = auto_home_params_[i].mit_param.torque/( N * r );
                        mit_motor_in.Kd     = auto_home_params_[i].mit_param.Kd/( N * r * r );
                        mit_motor_in.Kp     = auto_home_params_[i].mit_param.Kp/( N * r * r );
                        mit_motor_in.pos    = auto_home_params_[i].mit_param.pos * r ;
                        mit_motor_in.speed  = auto_home_params_[i].mit_param.speed * r ;
                        MotorControllers_[motor_count]->MotorMitCtrl(mit_motor_in);
                        is_all_ready = false;
                        is_part_ready = false;
                    }
                }
                motor_count++;
            }
            if(is_part_ready)auto_home_params_[i].is_auto_home = false;
        }
        if(is_all_ready){
            AutoHome_complete = true;
            optimizer_.Reset_is_init();
        }
        return true;
    };

    void Special_solve(ControlFloatArray &Axis_traget_pos){
        Limit_solve(Axis_traget_pos);
        PositionOptions_solve(Axis_traget_pos);
    }

    void Special_update(){
        AxisStall_update();
    }

    virtual void init(Control_Improvement improvement,
                      PlanParam planparam,
                      PidParam pidparam){
        optimizer_.init(improvement,planparam,pidparam);
    };

    virtual void UpdateAxisData();

    virtual void setAxisPos(ControlFloatArray Axis_pos,std::optional<float> T = std::nullopt);
    virtual void setAxisSpeed(ControlFloatArray Axis_speed);
    virtual ControlFloatArray getAxisPos(){ return Axis_actual_pos_; };
    virtual ControlFloatArray getAxisSpeed(){ return Axis_actual_speed_; };

    virtual void dispatchCommand(){
        switch (controlmode_) {
            case SPEEDPLAN:{
                MotorFloatArray optimizer_speed_output = optimizer_.GetSpeedOutput();
                for (size_t i = 0; i < MotorCount; ++i) {
                    float Motor_speed_rad = optimizer_speed_output[i]*motor_rad_per_output_[i];
                    MotorControllers_[i]->MotorSpeedCtrl_rad_s(Motor_speed_rad);
                }
                break;
            }
            case INTERPOLATION:{
                MotorFloatArray optimizer_pos_ref = optimizer_.GetPosRef();
                for (size_t i = 0; i < MotorCount; ++i) {
                    float Motor_sum_pos = optimizer_pos_ref[i]*motor_rad_per_output_[i];
                    MotorControllers_[i]->MotorSumPosCtrl_rad(Motor_sum_pos);
                }
                break;
            }
            default :
                break;
        }
    };

    MotorFloatArray getMotorPerOutput(){ return motor_rad_per_output_; }
    
    std::array<MotorControllerBase*, MotorCount> MotorControllers_;
    MotorFloatArray motor_rad_per_output_;
    std::array<std::size_t, ControlCount> ComtrollerMotorCounts_;

    ControlFloatArray Axis_traget_pos_{0};
    ControlFloatArray Axis_actual_pos_{0};
    ControlFloatArray Axis_actual_speed_{0};

    MotorFloatArray Axis_traget_pos_optimizer_in_{0};
    MotorFloatArray Axis_actual_pos_optimizer_in_{0};
    MotorFloatArray Axis_actual_speed_optimizer_in_{0};

    std::array<TravelLimit, ControlCount> travel_limit_{0};
    std::array<bool,ControlCount> is_limit_{false};

    std::array<PositionControlOptions, ControlCount> position_options_;
    std::array<AxisStallParam, ControlCount> stall_params_;
    std::array<AutoHomeParam, ControlCount> auto_home_params_;
    bool AutoHome_complete = true;

    ControlOptimizer<MotorCount> optimizer_;
    ControlOutputMode controlmode_ = SPEEDPLAN;

    float dt{0};
    uint32_t DWT_CNT{0};

};


template <std::size_t ControlCount,std::size_t MotorCount>
class GeneralAxis : public AxisBase<ControlCount,MotorCount>{
    using Base = AxisBase<ControlCount,MotorCount>;

public:
    using ControlFloatArray = typename Base::ControlFloatArray;
    using MotorFloatArray   = typename Base::MotorFloatArray;
    using TravelLimit       = typename Base::TravelLimit;
    using PositionControlOptions = typename Base::PositionControlOptions;

    // using Base::init;

    explicit GeneralAxis(std::array<std::initializer_list<MotorControllerBase*>, ControlCount> MotorControllers, 
                        std::array<float, MotorCount> motor_rad_per_output){
        std::size_t motor_count = 0; 
        this->motor_rad_per_output_ = motor_rad_per_output;
        for(std::size_t i = 0 ; i < ControlCount ; ++i){
            this->ComtrollerMotorCounts_[i] = MotorControllers[i].size();
            for(std::size_t j = 0 ; j < this->ComtrollerMotorCounts_[i] ; ++j){
                this->MotorControllers_[motor_count++] = MotorControllers[i].begin()[j];
            }
        }
    }

    // void init(Control_Improvement improvement,
    //                   PlanParam planparam,
    //                   PidParam pidparam,
    //                   std::initializer_list<std::size_t> ControlObject,
    //                   std::initializer_list<PositionControlOptions> position_options){
    //     this->optimizer_.init(improvement,planparam,pidparam);
    //     this->ConfigOptions(ControlObject,position_options);
    // };

    void UpdateAxisData() override {
        this->dt = DWT_GetDeltaT(&this->DWT_CNT);
        this->dt = this->dt > 0.005f ? 0.005f : this->dt;
        std::size_t motor_count = 0;
        for(std::size_t i = 0 ; i < ControlCount ; ++i){
            float sum_pos = 0.0f;
            float sum_speed = 0.0f;
            for(std::size_t j = 0 ; j < this->ComtrollerMotorCounts_[i] ; ++j){
                this->Axis_actual_pos_optimizer_in_[motor_count] = this->MotorControllers_[motor_count]->getMotorSumPos_rad()/this->motor_rad_per_output_[motor_count];
                this->Axis_actual_speed_optimizer_in_[motor_count] = this->MotorControllers_[motor_count]->getMotorSpeed_rad()/this->motor_rad_per_output_[motor_count];
                sum_pos += this->Axis_actual_pos_optimizer_in_[motor_count];
                sum_speed += this->Axis_actual_speed_optimizer_in_[motor_count];
                motor_count++;
            }
            this->Axis_actual_pos_[i] = sum_pos/this->ComtrollerMotorCounts_[i];
            this->Axis_actual_speed_[i] = sum_speed/this->ComtrollerMotorCounts_[i];
        }
    }

    void setAxisPos(ControlFloatArray Axis_pos,std::optional<float> T = std::nullopt) override {
        UpdateAxisData();
        this->AxisStall_update();
        if(this->AutoHome_solve())return ;

        std::size_t optimizer_in_count = 0;

        this->Special_solve(Axis_pos);

        for(std::size_t i = 0 ; i < ControlCount ; ++i){
            this->Axis_traget_pos_[i] = Axis_pos[i];
            for(std::size_t j = 0 ; j < this->ComtrollerMotorCounts_[i] ; ++j){
                this->Axis_traget_pos_optimizer_in_[optimizer_in_count] = this->Axis_traget_pos_[i];
                optimizer_in_count++;
            }
        }
        this->optimizer_.Run(this->Axis_traget_pos_optimizer_in_,this->Axis_actual_pos_optimizer_in_,T);
        this->dispatchCommand();
    }

    void setAxisSpeed(ControlFloatArray Axis_speed) override {
        UpdateAxisData();
        this->AxisStall_update();
        if(this->AutoHome_solve())return ;
        
        std::array<float, MotorCount> speed_in;
        std::size_t speed_in_count = 0;
        for(std::size_t i = 0 ; i < ControlCount ; ++i){
            for(std::size_t j = 0 ; j < this->ComtrollerMotorCounts_[i] ; ++j){
                float Motor_speed = Axis_speed[i]*this->motor_rad_per_output_[speed_in_count];
                speed_in[speed_in_count] = Motor_speed;
                this->MotorControllers_[speed_in_count]->MotorSpeedCtrl_rad_s(Motor_speed);
                speed_in_count++;
            }
        }
    }

private:

};
