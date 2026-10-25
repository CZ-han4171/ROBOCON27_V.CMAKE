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

class MotorControllerBase {
public:
    MotorControllerBase() = default;

    enum Control_Improvement{
        NONE,
        SPEEDPLAN,
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
    virtual void MotorSinglePosCtrl_rad(float traget_single_pos, float actual_single_pos){};
    virtual void MotorSumPosCtrl_rad(float sum_pos){};
    virtual void MotorSumPosCtrl_rad(float traget_sum_pos, float actual_sum_pos){};
    virtual void MotorMitCtrl(float speed, float pos, float torque, float Kp, float Kd){};
    virtual void MotorPsiCtrl(float pos, float speed, float current){};

    virtual float getMotorSpeed_rad(void){ return 0; };
    virtual float getMotorSumPos_rad(void){ return 0; };
    virtual float getMotorSinglePos_rad(void){ return 0; };

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
    void applySpeedParam(const PlanParam &plan_param){
        plan_ = plan_param;
    }

    float watch[10] = {0};

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
    PlanParam plan_;
};

class DJIMotorController : public MotorControllerBase{
public:
    explicit DJIMotorController(MotorBase& motor) : motor_(motor) {}

    void MotorCurrentCtrl(float Current) override {
        motor_.setMotorCmd(Current);
    }

    void MotorSpeedCtrl_rad_s(float speed) override {
        pid_speed_output_ = PID_Calculate(
                &speed_pid_,
                motor_.getCurrentSpeed(), // 输出轴转速(rad/s)
                speed);
        MotorCurrentCtrl(pid_speed_output_);
    }

    void MotorSinglePosCtrl_rad(float single_pos) override {
        MotorSinglePosCtrl_rad(single_pos,motor_.getCurrentSinglePos());
    }

    void MotorSumPosCtrl_rad(float sum_pos) override {
        MotorSumPosCtrl_rad(sum_pos,motor_.getCurrentSumPos());
    }

    void MotorSinglePosCtrl_rad(float traget_single_pos, float actual_single_pos) override {
        float delat = Warp_ToRange(traget_single_pos - actual_single_pos, -M_PI, M_PI);
        MotorSumPosCtrl_rad(delat,0.0f);
    }

    void MotorSumPosCtrl_rad(float traget_sum_pos, float actual_sum_pos) override {
        float delat = traget_sum_pos - actual_sum_pos;
        watch[0] = delat;
        dt = DWT_GetDeltaT(&DWT_CNT);
        switch(Improvement_) {
            case Control_Improvement::NONE:
                pid_sum_pos_output_ = PID_Calculate(
                &pos_pid_,
                actual_sum_pos, // 输出轴角度(rad)
                traget_sum_pos);
                pos_output_ = pid_sum_pos_output_;
                break;
            case Control_Improvement::SPEEDPLAN:
                pid_sum_pos_output_ = PID_Calculate(
                &pos_pid_,
                actual_sum_pos, // 输出轴角度(rad)
                traget_sum_pos);
                plan_pos_output_ = UpdatePlanSpeed(delat);
                plan_.kPlan2Pid = UpdatePlanK(delat);
                pos_output_ = plan_pos_output_*plan_.kPlan2Pid + pid_sum_pos_output_*(1-plan_.kPlan2Pid);
                break;
        }

        MotorSpeedCtrl_rad_s(pos_output_);
    }

    float UpdatePlanSpeed(float delat){
        float w_Acc = fabs(pos_output_) + plan_.Acc*dt;
        float w_Dec = sqrt(2.0f*plan_.Dec*fabs(delat));
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
        if(fabs(delat) > plan_.MixBegin){
            K_tmp = 1.0f;
        }else if(fabs(delat) < plan_.MixBegin && fabs(delat) > plan_.MixEnd){
            K_tmp = (fabs(delat)-(plan_.MixEnd))/((plan_.MixBegin)-(plan_.MixEnd));
        }else if(fabs(delat) < plan_.MixEnd){
            K_tmp = 0.0f;
        }
        return K_tmp;
    }

    float getMotorSpeed_rad(void) override { return motor_.getCurrentSpeed(); }
    float getMotorSumPos_rad(void) override { return motor_.getCurrentSumPos(); }
    float getMotorSinglePos_rad(void) override { return motor_.getCurrentSinglePos(); }

private:
    MotorBase& motor_;
};

class DM4310MotorController : public MotorControllerBase{
public:
    explicit DM4310MotorController(DM4310Motor& motor) : motor_(motor) {}

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
    explicit VESCMotorController(VESCMotor& motor) : motor_(motor) {}
    
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
                motor_.getCurrentSpeed(), // 输出轴转速(rad/s)
                target_speed_rad_s);
            MotorCurrentCtrl(pid_speed_output_);
        }else if(speed_control_mode_ == VESC_SpeedControlMode::kVescPid){
            motor_.setMotorCtrl( target_speed_rad_s*motor_.getMotorPoles()/RPM_2_RAD_PER_SEC, VESCMotor::SET_ERPM);
        }
    }

    void MotorSinglePosCtrl_rad(float traget_single_pos, float actual_single_pos) override {
        pid_single_pos_output_ = PID_Calculate(
                &pos_pid_,
                actual_single_pos, // 输出轴角度(rad)
                traget_single_pos);
        MotorSpeedCtrl_rad_s(pid_single_pos_output_);
    }

    void MotorSumPosCtrl_rad(float traget_sum_pos, float actual_sum_pos) override {
        pid_sum_pos_output_ = PID_Calculate(
                &pos_pid_,
                actual_sum_pos, // 输出轴角度(rad)
                traget_sum_pos);
        MotorSpeedCtrl_rad_s(pid_sum_pos_output_);
    }

    void setSpeedPidMode(VESC_SpeedControlMode mode){
        speed_control_mode_ = mode;
    }

    float getMotorSpeed_rad(void){ return motor_.getCurrentSpeed(); }


private:
    VESCMotor& motor_;
    VESC_SpeedControlMode speed_control_mode_ = VESC_SpeedControlMode::kLocalPid;

};