/**
 * @file chassis_solution.hpp
 * @author CZhan
 * @brief 底盘解算方案，X型全向轮运动学解算
 * @version 1.0
 * @date 2026-11-19
 *
 * @copyright Copyright (c) 2026
 *
 * @attention : 底盘解算
 * @note :
 * @versioninfo :
 */
#pragma once

#include "Axis.hpp"
#include "Encoder.hpp"
#include "Motor.hpp"
#include "bsp_dwt.h"
#include "pid_controller.hpp"
#include "topic_pool.hpp"
#include "tool.hpp"

#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <math.h>

struct Chassis_data{
    float x;
    float y;
    float yaw;
    float vx;
    float vy;
    float w;
};

template <std::size_t WheelCount>
class OmniChassis {
public:
    // enum WheelIndex : size_t {
    //     kLeftUp = 0,
    //     kRightUp = 1,
    //     kLeftDown = 2,
    //     kRightDown = 3,
    //     kWheelCount = 4,
    // };

    struct Geometry {
        float wheel_diameter_m;
        float x_m;
        float y_m;
        float rolling_angle_rad;
        

        Geometry(float wheel_diameter = 0.129f,
                 float x = 0.0f,float y = 0.0f,
                 float rolling_angle = 0.0f)
            : wheel_diameter_m(wheel_diameter),
              x_m(x),y_m(y),
              rolling_angle_rad(rolling_angle) {}
    };

    // 轮序固定为 [左上, 右上, 左下, 右下]
    OmniChassis(GeneralAxis<4,4> &OmniWheels,
                float &yaw,
                const std::array<Geometry,WheelCount> geometries)
        : OmniWheels_{OmniWheels},
          yaw_(yaw),
          geometries_(geometries) {
    }

    // void configureGeometry(const Geometry &geometry) { geometry_ = geometry; }

    // 运动学正解
    Chassis_data forwardKinematics(Chassis_data chassis_data_in) const {
        Chassis_data chassis_data_tmp{};

        // const float kCosYaw = cosf(yaw_);
        // const float kSinYaw = sinf(yaw_);

        // constexpr float kSqrt2 = sqrt(2.0f); // √2
        // const float rad_s_to_mps = geometry_.wheel_diameter_m / 2.0f;
        // const float vx_body = (-Wheel_actual_speed_rad[0] + Wheel_actual_speed_rad[1] - Wheel_actual_speed_rad[2] + Wheel_actual_speed_rad[3])
        //                   *rad_s_to_mps*kSqrt2/4.0f;
        // const float vy_body = ( Wheel_actual_speed_rad[0] + Wheel_actual_speed_rad[1] - Wheel_actual_speed_rad[2] - Wheel_actual_speed_rad[3])
        //                   *rad_s_to_mps*kSqrt2/4.0f;
        // const float w_body  = ( Wheel_actual_speed_rad[0] + Wheel_actual_speed_rad[1] + Wheel_actual_speed_rad[2] + Wheel_actual_speed_rad[3])
        //                   /(4.0f*);

        // chassis_data_tmp.vx =  vx_body*kCosYaw - vy_body*kSinYaw;
        // chassis_data_tmp.vy =  vx_body*kSinYaw + vy_body*kCosYaw;
        // chassis_data_tmp.w  =  w_body;

        // chassis_data_tmp.x  = chassis_data_in.x + chassis_data_tmp.vx*dt;
        // chassis_data_tmp.y  = chassis_data_in.y + chassis_data_tmp.vy*dt;
        // chassis_data_tmp.yaw= yaw_;

        return chassis_data_tmp;
    }

    // 运动学逆解：输入底盘速度命令，输出每个轮子的目标角速度(rad/s)
    std::array<float, 4> inverseKinematics(
        const pub_chassis_cmd &cmd) const {
        const float vx = cmd.linear_x_;
        const float vy = cmd.linear_y_;
        const float wz = cmd.omega_;

        // const float rotation_term = geometry_.track_width_m * 0.5f * wz;

        // constexpr float kInvSqrt2 = 0.70710678f; // 1/√2

        std::array<float,WheelCount> volatiles;
        for( std::size_t i = 0 ; i < WheelCount ; ++i){
            const float Cos = cosf(geometries_[i].rolling_angle_rad);
            const float Sin = sinf(geometries_[i].rolling_angle_rad);

            volatiles[i] = (vx*Cos + vy*Sin + wz*(geometries_[i].x_m*Sin - geometries_[i].y_m*Cos))/geometries_[i].wheel_diameter_m;
        }
        return volatiles;
        // const float v_lu = (-vx + vy) * kInvSqrt2 + rotation_term;
        // const float v_ru = ( vx + vy) * kInvSqrt2 + rotation_term;
        // const float v_ld = (-vx - vy) * kInvSqrt2 + rotation_term;
        // const float v_rd = ( vx - vy) * kInvSqrt2 + rotation_term;

        // const float mps_to_rad_s = 2.0f / geometry_.wheel_diameter_m;
    }

    // 输入底盘速度命令，完成 解算→Axis速度设置
    void run(const pub_chassis_cmd &cmd) {
        dt = DWT_GetDeltaT(&DWT_CNT);
        Wheel_traget_speed_rad = inverseKinematics(cmd);
        OmniWheels_.setAxisSpeed(Wheel_traget_speed_rad);
        Wheel_actual_speed_rad = OmniWheels_.getAxisSpeed();

        chassis_date_out_ = forwardKinematics(chassis_data_in_);
    }

    Chassis_data getChassisData(){ return chassis_date_out_; }
    void SendChassisData(Chassis_data chassis_data_in){ chassis_data_in_ = chassis_data_in; }

private:

    GeneralAxis<4,4>& OmniWheels_;
    float &yaw_;
    std::array<Geometry, WheelCount> geometries_{};
    std::array<float, WheelCount> Wheel_traget_speed_rad;
    std::array<float, WheelCount> Wheel_actual_speed_rad;

    Chassis_data chassis_data_in_{};
    Chassis_data chassis_date_out_{};

    float dt;
    uint32_t DWT_CNT;
    
};

// class SwerveChassis4 {
// public:
//     enum WheelIndex : size_t {
//         kOne = 0,
//         kTwo = 1,
//         kThree = 2,
//         kFour = 3,
//         kWheelCount = 4,
//     };

//     enum SwerveDirection : int8_t {
//         kPositive = 1,
//         kNegative = -1,
//     };

//     struct Geometry {
//         float_t wheel_diameter_m; // 轮径
//         Vec2_t wheel_position_m;   // 舵轮定位

//         Geometry(float wheel_diameter = 0.8f,
//                  Vec2_t wheel_position = {.x = 0.1915f,.y = 0.208f})
//             : wheel_diameter_m(wheel_diameter),
//               wheel_position_m(wheel_position) {}
//     };

//     // 轮序固定为 [左上, 右上, 左下, 右下]
//     SwerveChassis4(RotaryAxis<1,1> &DriveMotor_1, RotaryAxis<1,1> &DriveMotor_2, RotaryAxis<1,1> &DriveMotor_3, RotaryAxis<1,1> &DriveMotor_4,
//                    RotaryAxis<1,1> &SteerMotor_1, RotaryAxis<1,1> &SteerMotor_2, RotaryAxis<1,1> &SteerMotor_3, RotaryAxis<1,1> &SteerMotor_4,
//                    BRT38B     &Encoder_1,    BRT38B     &Encoder_2,    BRT38B     &Encoder_3,    BRT38B     &Encoder_4)
//         : DriveMotors_{&DriveMotor_1, &DriveMotor_2, &DriveMotor_3, &DriveMotor_4},
//           SteerMotors_{&SteerMotor_1, &SteerMotor_2, &SteerMotor_3, &SteerMotor_4},
//           Encoders_{&Encoder_1, &Encoder_2, &Encoder_3, &Encoder_4}
//         {
//         setWheelDirectionSign({SwerveDirection::kPositive, SwerveDirection::kPositive, SwerveDirection::kPositive, SwerveDirection::kPositive});
//     }

//     void configureGeometries(const std::array<Geometry, kWheelCount> &Geometries) {
//         for (size_t i = 0; i < Geometries_.size(); ++i) {
//             Geometries_[i] = Geometries[i];
//         }
//     }

//     void setWheelDirectionSign(const std::array<SwerveDirection, 4> &direction_sign) {
//         direction_sign_ = direction_sign;
//     }

//     // 运动学逆解：输入底盘速度命令，输出每个轮子的目标角速度(rad/s)和航向电机角度(rad)
//     void solveWheelRadPerSec(const pub_chassis_cmd &cmd){
//         const float vx = cmd.linear_x_;
//         const float vy = cmd.linear_y_;
//         const float wz = cmd.omega_;

//         for (size_t i = 0; i < kWheelCount; ++i){
//             Vec2_t Swerve_speed{0,0};
//             Swerve_speed.x = vx - wz*Geometries_[i].wheel_position_m.y;
//             Swerve_speed.y = vy + wz*Geometries_[i].wheel_position_m.x;

//             const float mps_to_rad_s =  2.0f / Geometries_[i].wheel_diameter_m;
//             target_speed_rad_s_[i]  = sqrtf(Swerve_speed.x*Swerve_speed.x + Swerve_speed.y*Swerve_speed.y)*mps_to_rad_s;
//             target_raw_steer_rad[i] = atan2f(Swerve_speed.y, Swerve_speed.x);

//             // SteerMotors_[i]->UpdateAxisPos(Encoders_[i]->getSumPos_rad());
//             SteerMotors_[i]->UpdateAxisData();
//             const float angle_offset = (direction_sign_[i] == SwerveDirection::kNegative)?M_PI:0.0f;
//             actual_steer_motor_rad[i] = Warp_ToRange(SteerMotors_[i]->getAxisSinglePos_rad()[0] + angle_offset, -M_PI, M_PI);
//             if( Swerve_speed.x == 0.0f && Swerve_speed.y == 0.0f){
//                 target_raw_steer_rad[i] = target_raw_steer_rad_last[i];
//             }else{
//                 if(abs(Warp_ToRange(target_raw_steer_rad[i] - actual_steer_motor_rad[i], -M_PI, M_PI)) > M_PI_2){
//                     direction_sign_[i] = (direction_sign_[i] == SwerveDirection::kNegative) ? SwerveDirection::kPositive : SwerveDirection::kNegative;
//                 }
//                 const float angle_offset = (direction_sign_[i] == SwerveDirection::kNegative)?M_PI:0.0f;
//                 target_steer_motor_rad[i] = Warp_ToRange(target_raw_steer_rad[i] + angle_offset, -M_PI, M_PI);
//             }
//             float steer_error = Warp_ToRange(target_steer_motor_rad[i] - SteerMotors_[i]->getAxisSinglePos_rad()[0], -M_PI, M_PI);
//             watch[i] = steer_error;
//             float scale = fmaxf(0.0f, cosf(steer_error));
//             target_speed_rad_s_[i] *= direction_sign_[i]*scale;

//             target_raw_steer_rad_last[i] = target_raw_steer_rad[i];
//         }
//     }

//     // 输入底盘速度命令，完成 解算→PID→setMotorCmd
//     void run(const pub_chassis_cmd &cmd) {
//         solveWheelRadPerSec(cmd);
//         for (size_t i = 0; i < kWheelCount; ++i) {
//             float DriveSpeed_In = target_speed_rad_s_[i] + SteerMotors_[i]->getAxisSpeed_rad()[0]/DriveMotors_[i]->getMotorPerOutput()[0];
//             DriveMotors_[i]->setAxisSpeed_rad_s({DriveSpeed_In});
//             SteerMotors_[i]->setAxisSinglePos_rad({target_steer_motor_rad[i]});
//             // SteerMotors_[i]->setAxisPos_rad(target_steer_motor_rad[i],Encoders_[i]->getSumPos_rad());
//         }
//     }

//     const std::array<float, 4> &targetRADPerSec() const { return target_speed_rad_s_; }

//     float watch[10];

// private:
//     static constexpr float kPi = 3.14159265358979323846f;

//     static void applyPidParam(PID_t &pid, const PidParam &param) {
//         pid = {};
//         pid.Kp = param.kp;
//         pid.Ki = param.ki;
//         pid.Kd = param.kd;
//         pid.MaxOut = param.max_out;
//         pid.DeadBand = param.deadband;
//         pid.Improve = param.improve;
//         PID_Init(&pid);
//     }

//     std::array<RotaryAxis<1,1> *, 4> DriveMotors_{};
//     std::array<RotaryAxis<1,1> *, 4> SteerMotors_{};
//     std::array<BRT38B *, 4> Encoders_{};
//     std::array<Geometry, kWheelCount> Geometries_{};
//     std::array<SwerveDirection, kWheelCount> direction_sign_{};
//     std::array<float, kWheelCount> target_speed_rad_s_{};
//     std::array<float, kWheelCount> target_raw_steer_rad{};
//     std::array<float, kWheelCount> target_raw_steer_rad_last{};
//     std::array<float, kWheelCount> target_steer_motor_rad{};
//     std::array<float, kWheelCount> actual_steer_motor_rad{};
// };