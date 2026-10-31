/**
 * @file MotorControl.hpp
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

class MotorControllerBase {
public:
    MotorControllerBase() = default;

    enum class Control_Improvement{
        NONE,
        SPEEDPLAN,
    };

    enum class FeedBackDirection{
        kPositive,
        kNegative,
    };

    struct PlanParam {
        float Acc;
        float Dec;
        float kPlan2Pid;
        float MixBegin;
        float MixEnd;

        PlanParam(float Acc_in = M_PI, float Dec_in = M_PI, float MixBegin_in = M_PI/4.0f, float Mix_End_in = M_PI/10.0f)
            :Acc(Acc_in), Dec(Dec_in), MixBegin(MixBegin_in), MixEnd(Mix_End_in){}
    };

    virtual void MotorCurrentCtrl(float current){};
    virtual void MotorSpeedCtrl_rad_s(float speed){};
    virtual void MotorSinglePosCtrl_rad(float single_pos){};
    virtual void MotorSumPosCtrl_rad(float sum_pos){};
    virtual void MotorMitCtrl(float speed, float pos, float torque, float Kp, float Kd){};
    virtual void MotorPsiCtrl(float pos, float speed, float current){};

    virtual float getMotorSpeed_rad(void){ return 0; };
    float getMotorSumPos_rad(void){ return feedback_rad_per_motor_rad_*getFeedBackSumPos_rad(); };
    float getMotorSinglePos_rad(void){ return Warp_ToRange(getMotorSumPos_rad(), -M_PI, M_PI); };

    std::function<float()> getFeedBackSumPos_rad;
    std::function<float()> getFeedBackSinglePos_rad;
    void ConfigSumPosSource(std::function<float()> func){ getFeedBackSumPos_rad = func; }
    void ConfigSinglePosSource(std::function<float()> func){ getFeedBackSinglePos_rad = func; }
    void ConfigFeedBackRatio(float ratio){ feedback_rad_per_motor_rad_ = ratio; }

    void setMaxPosSpeed_rad_s(float MaxSpeed_rad_s){
        pos_pid_.MaxOut = MaxSpeed_rad_s;
    }
    void ConfigControlImprovement(const Control_Improvement &Improvement){
        Improvement_ = Improvement;
    }
    void ConfigSpeedPid(const PidParam &param){
        applyPidParam(speed_pid_, param);
    }
    void ConfigPosPid(const PidParam &param){
        applyPidParam(pos_pid_, param);
    }
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
    void applyPlanParam(const PlanParam &plan_param){
        plan_param_ = plan_param;
    }

    float UpdatePlanSpeed(float delat){
        float w_Acc = fabs(pos_output_) + plan_param_.Acc*dt;
        float w_Dec = sqrt(2.0f*plan_param_.Dec*fabs(delat));
        float plan_pos_output_tmp;
        if(pos_pid_.MaxOut < w_Acc && pos_pid_.MaxOut < w_Dec){
            plan_pos_output_tmp = pos_pid_.MaxOut;
        }else if(w_Acc > w_Dec){
            plan_pos_output_tmp = w_Dec;
        }else {
            plan_pos_output_tmp = w_Acc;
        };
        return plan_pos_output_tmp*sign(delat);
    };

    float UpdatePlanK(float delat){
        float K_tmp;
        if(fabs(delat) > plan_param_.MixBegin){
            K_tmp = 1.0f;
        }else if(fabs(delat) <= plan_param_.MixBegin && fabs(delat) > plan_param_.MixEnd){
            K_tmp = (fabs(delat)-(plan_param_.MixEnd))/((plan_param_.MixBegin)-(plan_param_.MixEnd));
        }else if(fabs(delat) <= plan_param_.MixEnd){
            K_tmp = 0.0f;
        }
        return K_tmp;
    }

    float watch[10] = {0};

    float feedback_rad_per_motor_rad_ = 1.0f;

    float pid_speed_output_;
    float pid_single_pos_output_;
    float pid_sum_pos_output_;
    PID_t speed_pid_{};
    PID_t pos_pid_{};

    float plan_pos_output_;
    float pos_output_;

    float dt;
    uint32_t DWT_CNT;

    Control_Improvement Improvement_{Control_Improvement::NONE};
    PlanParam plan_param_;
};

class DJIMotorController : public MotorControllerBase{
public:
    explicit DJIMotorController(MotorBase& motor) : motor_(motor) {
        // getFeedBackSpeed_rad = [this]() -> float {return motor_.getCurrentSpeed();};
        getFeedBackSumPos_rad = [this]() -> float {return motor_.getCurrentSumPos();};
        getFeedBackSinglePos_rad = [this]() -> float {return motor_.getCurrentSinglePos();};
    }
    explicit DJIMotorController(MotorBase& motor, BRT38B& Encoder, float ratio) : motor_(motor) {
        // getFeedBackSpeed_rad = [this]() -> float {return motor_.getCurrentSpeed();};
        getFeedBackSumPos_rad = [&Encoder]() -> float {return Encoder.getSumPos_rad();};
        getFeedBackSinglePos_rad = [&Encoder]() -> float {return Encoder.getSinglePos_rad();};
        feedback_rad_per_motor_rad_ = ratio;
    }

    void init(const PlanParam &plan_param, const PidParam &speed_param, const PidParam &pos_param, const Control_Improvement &improvement){
        applyPlanParam(plan_param);
        ConfigSpeedPid(speed_param);
        ConfigPosPid(pos_param);
        ConfigControlImprovement(improvement);
    }

    void MotorCurrentCtrl(float Current) override {
        motor_.setMotorCmd(Current);
    }

    void MotorSpeedCtrl_rad_s(float speed) override {
        pid_speed_output_ = PID_Calculate(
                &speed_pid_,
                getMotorSpeed_rad(), // 输出轴转速(rad/s)
                speed);
        MotorCurrentCtrl(pid_speed_output_);
    }

    void MotorSinglePosCtrl_rad(float single_pos) override {
        float delat = Warp_ToRange(single_pos - getMotorSinglePos_rad(), -M_PI, M_PI);
        MotorSumPosCtrl_rad(delat + getMotorSumPos_rad());
    }

    void MotorSumPosCtrl_rad(float sum_pos) override {
        float actual_sum_pos = getMotorSumPos_rad();
        float delat = sum_pos - actual_sum_pos;
        dt = DWT_GetDeltaT(&DWT_CNT);
        switch(Improvement_) {
            case Control_Improvement::NONE:
                pid_sum_pos_output_ = PID_Calculate(
                &pos_pid_,
                actual_sum_pos, // 输出轴角度(rad)
                sum_pos);
                pos_output_ = pid_sum_pos_output_;
                break;
            case Control_Improvement::SPEEDPLAN:
                pid_sum_pos_output_ = PID_Calculate(
                &pos_pid_,
                actual_sum_pos, // 输出轴角度(rad)
                sum_pos);
                plan_pos_output_ = UpdatePlanSpeed(delat);
                plan_param_.kPlan2Pid = UpdatePlanK(delat);
                pos_output_ = plan_pos_output_*plan_param_.kPlan2Pid + pid_sum_pos_output_*(1-plan_param_.kPlan2Pid);
                break;
        }
        MotorSpeedCtrl_rad_s(pos_output_);
    }

    float getMotorSpeed_rad() override {
        return motor_.getCurrentSpeed();
    }

private:
    MotorBase& motor_;
};

class DM4310MotorController : public MotorControllerBase{
public:
    explicit DM4310MotorController(DM4310Motor& motor) : motor_(motor) {
        getFeedBackSumPos_rad = [this]() -> float {return motor_.getCurrentSumPos();};
        getFeedBackSinglePos_rad = [this]() -> float {return motor_.getCurrentSinglePos();};
    }

    void MotorSpeedCtrl_rad_s(float speed){
        motor_.speedControl(speed);
    };
    void MotorPosCtrl_rad(float pos){
        motor_.posWithSpeedControl(pos,pos_pid_.MaxOut);
    };
    void MotorMitCtrl(float speed, float pos, float torque, float Kp, float Kd){
        motor_.mitControl(speed,pos,torque,Kp,Kd);
    };
    void MotorPsiCtrl(float pos, float speed, float current){
        motor_.psiControl(pos,speed,current);
    };

private:
    DM4310Motor& motor_;
};

class VESCMotorController : public MotorControllerBase{
public:
    explicit VESCMotorController(VESCMotor& motor) : motor_(motor) {
        getFeedBackSumPos_rad = [this]() -> float {return motor_.getCurrentSumPos();};
        getFeedBackSinglePos_rad = [this]() -> float {return motor_.getCurrentSinglePos();};
    }
    
    enum VESC_SpeedControlMode {
        kVescPid,
        kLocalPid
    };

    void MotorCurrentCtrl(float current) override {
        motor_.setMotorCtrl(current , VESCMotor::SET_CURRENT);
    }

    void MotorSpeedCtrl_rad_s(float target_speed_rad_s) override {
        if(speed_control_mode_ == VESC_SpeedControlMode::kLocalPid){
            pid_speed_output_ = PID_Calculate(
                &speed_pid_,
                getMotorSpeed_rad(), // 输出轴转速(rad/s)
                target_speed_rad_s);
            MotorCurrentCtrl(pid_speed_output_);
        }else if(speed_control_mode_ == VESC_SpeedControlMode::kVescPid){
            motor_.setMotorCtrl( target_speed_rad_s*motor_.getMotorPoles()/RPM_2_RAD_PER_SEC, VESCMotor::SET_ERPM);
        }
    }

    void MotorSinglePosCtrl_rad(float single_pos) override {
        float delat = Warp_ToRange(single_pos - getMotorSinglePos_rad(), -M_PI, M_PI);
        MotorSumPosCtrl_rad(delat + getMotorSumPos_rad());
    }

    void MotorSumPosCtrl_rad(float sum_pos) override {
        pid_sum_pos_output_ = PID_Calculate(
                &pos_pid_,
                getMotorSumPos_rad(), // 输出轴角度(rad)
                sum_pos);
        MotorSpeedCtrl_rad_s(pid_sum_pos_output_);
    }

    void setSpeedPidMode(VESC_SpeedControlMode mode){
        speed_control_mode_ = mode;
    }

    float getMotorSpeed_rad() override {
        return motor_.getCurrentSpeed();
    }

private:
    VESCMotor& motor_;
    VESC_SpeedControlMode speed_control_mode_ = VESC_SpeedControlMode::kVescPid;

};