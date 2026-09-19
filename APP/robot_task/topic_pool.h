/**
 * @file topic_pool.h
 * @author 大帅将军 ，Keten (2863861004@qq.com)
 * @brief
 * 模块所依赖的数据类型结构体，有些模块会依赖这些数据类型结构体进行数据传输，所以移植module层都
 *        必须携带这个包
 * @version 0.1
 * @date 2024-10-03
 *
 * @copyright Copyright (c) 2024
 *
 * @attention :
 * @note :
 * @versioninfo :
 */
#pragma once
#include "fdcan.h"
#include "rm_pocket.hpp"
#include "usart.h"
#include <stdbool.h>
#include <stdint.h>

#pragma pack(1)

typedef struct {
  UART_HandleTypeDef *huart; // 串口句柄
  uint16_t len;              // 数据长度
  void *data_addr; // 数据地址，使用时把地址赋值给这个指针，数值强转为uint8_t
} UART_TxMsg;

typedef enum {
    Left = -1,
    right = 1,
}FieldSide_t;

typedef struct {
  //摇杆，值域172-1810 建议映射值：min200 max1780 mid~=985 死区+-100
  uint16_t joyLHori;  //左小
  uint16_t joyLVert;  //下小
  uint16_t joyRHori;  //左小
  uint16_t joyRVert;  //下小

  RM_2_POS_SW_State_t swA;        //阴刻有SA的两端开关
  RM_2_POS_SW_State_t swA_last;
  RM_3_POS_SW_State_t swB;        //阴刻有SB的三段开关
  RM_3_POS_SW_State_t swB_last;
  RM_3_POS_SW_State_t swC;        //阴刻有SC的三段开关
  RM_3_POS_SW_State_t swC_last;
  RM_2_POS_SW_State_t swD;        //阴刻有SD的两段开关
  RM_2_POS_SW_State_t swD_last;
  RM_2_POS_SW_State_t swE;        //阴刻有SE的按钮
  RM_2_POS_SW_State_t swE_last;

  RM_Trim_State_t trimLeft;        //左微调按钮
  RM_Trim_State_t trimLeft_last;
  RM_Trim_State_t trimRight;       //右微调按钮
  RM_Trim_State_t trimRight_last;

  //电位器 往左推小，值域172-1810 实际可能取不到端点
  uint16_t pot;                   //阴刻有S1的拨盘

  //左微调按钮控制的光标，-9~+9
  int8_t x_cnt;
  int8_t y_cnt;

  //拨盘电位器控制的光标，最左0，中间1，最右2
  int8_t cursor;
} pub_RM_Data;

// IMU姿态传感器数据 —— 无头模式用
typedef struct {
  float yaw_rad;   // 偏航角，单位：弧度，范围 -π ~ +π
  float pitch_rad; // 俯仰角
  float roll_rad;  // 滚转角
} pub_imu_data;

typedef struct{
  float Acc_linear;
  float Dec_linear;
  float v_Max;

  float Acc_omega;
  float Dec_omega;
  float w_Max;
} speed_plan;

typedef struct {
    float_t Acc_linear;
    float_t Dec_linear;
    float_t v_Max;
    float_t w_Max;
} speed_data;

typedef struct {
  bool chassis_motor1;
  bool chassis_motor2;
  bool chassis_motor3;
  bool chassis_motor4;
  bool arm2006_motor;
  bool arm3508_motor;
  bool arm4310_motor;
} pub_motor_status;

// 发布底盘运动指令
typedef struct {
  float linear_x_;
  float linear_y_;
  float omega_;
} pub_chassis_cmd;

// 底盘速度
typedef struct {
  float vx;
  float vy;
  float w;
} chassis_speed;

// 底盘定位
typedef struct {
  float x;
  float y;
  float yaw;
} chassis_position;

//Position模块数据结构体
typedef struct {
  uint8_t frame_id;
  uint8_t payload_length;
  uint32_t frame_count;
  float x;
  float y;
  float yaw;
  float yaw_speed;

} pub_Position_Data;

#pragma pack()