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
#include "optional"
#include <stdatomic.h>
#include <stdint.h>
#include "bsp_dwt.h"
#include "Motor.hpp"
#include "pid_controller.hpp"
#include "ControlOptimize.hpp"
#include "tool.hpp"
#include "topic_pool.h"
#include <functional>

#define stall_timeout_s_ 1.0f // 自动归位检测时长
#define unstuck_timeout_s_ 0.05f // 判断电机正常运转时长

struct MitParam{
    float speed;
    float pos;
    float torque;
    float Kp;
    float Kd;

    MitParam(float speed_in = 0.0f,float pos_in = 0.0f,
            float torque_in = 0.0f,float Kp_in = 0.0f,float Kd_in = 0.0f):
                speed(speed_in),pos(pos_in),
                torque(torque_in),Kp(Kp_in),Kd(Kd_in){}
};

class MotorControllerBase {
public:
    MotorControllerBase() = default;

    enum class FeedBackDirection{
        kPositive,
        kNegative,
    };

    virtual void MotorCurrentCtrl(float current){};
    virtual void MotorSpeedCtrl_rad_s(float speed){};
    // virtual void MotorSinglePosCtrl_rad(float single_pos){};
    // virtual void MotorSumPosCtrl_rad(float sum_pos){};
    virtual void MotorSinglePosCtrl_rad(float single_pos,std::optional<float> T = std::nullopt){};
    virtual void MotorSumPosCtrl_rad(float sum_pos,std::optional<float> T = std::nullopt){};
    virtual void MotorMitCtrl(MitParam mit_param){};
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
        optimizer_.plan_param_.w_Max = MaxSpeed_rad_s;
    }
    void ConfigControlImprovement(const ControlOptimizer::Control_Improvement Improvement){
        optimizer_.ConfigImprovement(Improvement);
    }
    void ConfigSpeedPid(const PidParam &param){
        applyPidParam(speed_pid_, param);
    }
    void ConfigPosPid(const PidParam &param){
        applyPidParam(pos_pid_, param);
        applyPidParam(optimizer_.pos_pid_, param);
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
    void applyPlanParam(const ControlOptimizer::PlanParam &plan_param){
        optimizer_.ConfigPlanParam(plan_param);
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
    float torque_output_;

    float dt;
    uint32_t DWT_CNT;

    float Stall_time_{0.0f};
    float Unstuck_time_{0.0f};
    bool is_auto_home_{false};
    float home_pos_{0.0f};
    MitParam AutoHomeParam_;

    ControlOptimizer optimizer_;
    ControlOptimizer::Control_Improvement Improvement_{ControlOptimizer::Control_Improvement::NONE};
    // PlanParam plan_param_;
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

    void init(const ControlOptimizer::PlanParam &plan_param, const PidParam &speed_param, 
              const PidParam &pos_param, const ControlOptimizer::Control_Improvement &improvement){
        ConfigSpeedPid(speed_param);
        ConfigPosPid(pos_param);
        applyPlanParam(plan_param);
        ConfigControlImprovement(improvement);
    }

    void init(const ControlOptimizer::PlanParam &plan_param, const PidParam &speed_param, 
              const PidParam &pos_param, const ControlOptimizer::Control_Improvement &improvement,
              const MitParam &mit_homeparam, const float &HomePos){
        ConfigSpeedPid(speed_param);
        ConfigPosPid(pos_param);
        applyPlanParam(plan_param);
        ConfigControlImprovement(improvement);
        AutoHomeParam_ = mit_homeparam;
        home_pos_ = HomePos;
        is_auto_home_ = true;
    }

    void MotorCurrentCtrl(float Current) override {
        if(AutoHome())return;

        motor_.setMotorCmd(Current);
    }

    void MotorSpeedCtrl_rad_s(float speed) override {
        if(AutoHome())return;

        pid_speed_output_ = PID_Calculate(
                &speed_pid_,
                getMotorSpeed_rad(), // 输出轴转速(rad/s)
                speed);
        MotorCurrentCtrl(pid_speed_output_);
    }
    
    void MotorSinglePosCtrl_rad(float single_pos,std::optional<float> T = std::nullopt) override {
        if(AutoHome())return;

        float delat = Warp_ToRange(single_pos - getMotorSinglePos_rad(), -M_PI, M_PI);
        MotorSumPosCtrl_rad(delat + getMotorSumPos_rad(), T);
        
    }

    void MotorSumPosCtrl_rad(float sum_pos,std::optional<float> T = std::nullopt) override {
        if(AutoHome())return;

        float actual_sum_pos = getMotorSumPos_rad();
        optimizer_.Set(sum_pos,actual_sum_pos, T);
        optimizer_.Run(actual_sum_pos);
        pos_output_ = optimizer_.GetPosOutput();
        MotorSpeedCtrl_rad_s(pos_output_);
    }

    void MotorMitCtrl(MitParam mit_param) override {
        if(AutoHome())return;
        
        const float kt = motor_.getMotorTurqueConstant();
        if (!std::isfinite(kt) || kt <= 0.0f) {
            MotorCurrentCtrl(0.0f);
            return;
        }
        torque_output_ = mit_param.torque + mit_param.Kp*(mit_param.pos - getMotorSumPos_rad()) + mit_param.Kd*(mit_param.speed - getMotorSpeed_rad());
        const float current = torque_output_ / kt;
        MotorCurrentCtrl(std::isfinite(current) ? current : 0.0f);
    }

    bool isMotorStalled(){
        dt = DWT_GetDeltaT(&DWT_CNT);
        if(dt > 0.005f)dt = 0.005;
        float estimated_torque_ = AutoHomeParam_.torque + AutoHomeParam_.Kd*fabs(AutoHomeParam_.speed);
        if(fabs(getMotorSpeed_rad()) < 0.1f && fabs(motor_.getCurrentTorque()) > estimated_torque_/2.0f){
            Unstuck_time_ = 0.0f;
            Stall_time_ = Stall_time_ + dt;
            if(Stall_time_ >= stall_timeout_s_){
                Stall_time_ = 0.0f;
                return true;
            }
        }else {
            Unstuck_time_ = Unstuck_time_ + dt;
            if(Unstuck_time_ >= unstuck_timeout_s_){
                Unstuck_time_ = 0.0f;
                Stall_time_ = 0.0f;
            }
        }
        return false;
    }

    bool AutoHome(){
        if(is_auto_home_){
            if(isMotorStalled()){
                is_auto_home_ = false;
                motor_.ResetMotorPos(home_pos_);
                return is_auto_home_;
            }else {
                const float kt = motor_.getMotorTurqueConstant();
                if (!std::isfinite(kt) || kt <= 0.0f) {
                    motor_.setMotorCmd(0.0f);
                }else{
                    torque_output_ = AutoHomeParam_.torque
                             + AutoHomeParam_.Kp*(AutoHomeParam_.pos - getMotorSumPos_rad())
                             + AutoHomeParam_.Kd*(AutoHomeParam_.speed - getMotorSpeed_rad());
                    const float current = torque_output_ / kt;
                    motor_.setMotorCmd(std::isfinite(current) ? current : 0.0f);
                }
                return is_auto_home_;
            }
        }
        return is_auto_home_;
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

    void MotorSpeedCtrl_rad_s(float speed) override {
        motor_.speedControl(speed);
    };
    void MotorSumPosCtrl_rad(float pos,std::optional<float> T = std::nullopt) override {
        motor_.posWithSpeedControl(pos,pos_pid_.MaxOut);
    };
    void MotorMitCtrl(MitParam mit_param) override {
        motor_.mitControl(mit_param.speed,mit_param.pos,mit_param.torque,mit_param.Kp,mit_param.Kd);
    };
    void MotorPsiCtrl(float pos, float speed, float current) override {
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

    void MotorSinglePosCtrl_rad(float single_pos,std::optional<float> T = std::nullopt) override {
        float delat = Warp_ToRange(single_pos - getMotorSinglePos_rad(), -M_PI, M_PI);
        MotorSumPosCtrl_rad(delat + getMotorSumPos_rad());
    }

    void MotorSumPosCtrl_rad(float sum_pos,std::optional<float> T = std::nullopt) override {
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