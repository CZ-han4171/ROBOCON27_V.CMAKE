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

    struct SpeedPlan{
        float Vec;
        float Dec;
        float Max;
    };

    // struct PidParam {
    //     float kp;
    //     float ki;
    //     float kd;
    //     float max_out;
    //     float deadband;
    //     uint16_t improve;

    //     PidParam(float kp_in = 100.0f, float ki_in = 80.0f,
    //                   float kd_in = 0.0f, float max_out_in = 20000.0f,
    //                   float deadband_in = 0.3f, uint16_t improve_in = NONE)
    //         : kp(kp_in), ki(ki_in), kd(kd_in), max_out(max_out_in),
    //           deadband(deadband_in), improve(improve_in) {}
    // };

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

    void applySpeedPlan(const SpeedPlan &speed_plan){
        speed_plan_ = speed_plan;
    }

    float watch1;
    float watch2;
    float watch3;
    float watch4;
    PID_t watch5;

    float pid_speed_output_;
    float pid_single_pos_output_;
    float pid_sum_pos_output_;
    PID_t speed_pid_{};
    PID_t pos_pid_{};

    SpeedPlan speed_plan_;
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
        watch2 = pid_speed_output_;
        MotorCurrentCtrl(pid_speed_output_);
    }

    void MotorSinglePosCtrl_rad(float single_pos) override {
        MotorSinglePosCtrl_rad(single_pos,motor_.getCurrentSinglePos());
    }

    void MotorSumPosCtrl_rad(float sum_pos) override {
        MotorSumPosCtrl_rad(sum_pos,motor_.getCurrentSumPos());
    }

    void MotorSinglePosCtrl_rad(float traget_single_pos, float actual_single_pos) override {
        float delat = Warp_ToRange(actual_single_pos - traget_single_pos, -M_PI, M_PI);
        pid_single_pos_output_ = PID_Calculate(
                &pos_pid_,
                delat, // 输出轴单圈角度(rad)
                0);
        MotorSpeedCtrl_rad_s(pid_single_pos_output_);
    }

    void MotorSumPosCtrl_rad(float traget_sum_pos, float actual_sum_pos) override {
        watch3 = traget_sum_pos;
        watch4 = actual_sum_pos;
        pid_sum_pos_output_ = PID_Calculate(
                &pos_pid_,
                actual_sum_pos, // 输出轴角度(rad)
                traget_sum_pos);
        watch5 = pos_pid_;
        watch1 = pid_sum_pos_output_;
        MotorSpeedCtrl_rad_s(pid_sum_pos_output_);
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
        kLocal_Pid
    };

    void MotorCurrentCtrl(float current) override {
        motor_.setMotorCtrl(current , VESCMotor::SET_CURRENT);
    }

    void MotorSpeedCtrl_rad_s(float target_speed_rad_s) override {
        if(speed_control_mode_ == VESC_SpeedControlMode::kLocal_Pid){
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
    VESC_SpeedControlMode speed_control_mode_ = VESC_SpeedControlMode::kLocal_Pid;

};