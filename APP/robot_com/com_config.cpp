/**
 * @file com_config.cpp
 * @author Keten (2863861004@qq.com) lxlx
 * @brief 全局通信配置，包含can设备、串口设备、协议解析等
 * @version 0.2
 * @date 2026-04-21 2026-05-19(lxlx)
 *
 * @copyright Copyright (c) 2026
 *
 * @attention :
 * @note :
 * @versioninfo :
 */
#include "com_config.h"
#include "FreeRTOS.h"
#include "cmsis_os2.h"
#include "pid_controller.h"
#include "portmacro.h"
#include "stm32h7xx_hal.h"
#include "stm32h7xx_hal_def.h"
#include "stm32h7xx_hal_uart.h"
#include "task.h"

#include "Canbus.hpp"
#include "Motor.hpp"
#include "Position.hpp"
#include "ROSCom.hpp"
#include "UartPort.hpp"
#include "UsbPort.hpp"
#include "rm_pocket.hpp"
#include "pm20s.hpp"
#include "tim.h"
#include "WitMotionImu.hpp"
#include "topics.hpp"
#include "topic_pool.h"
#include "usart.h"

#include "control_Traject.hpp"

#include "chassis_solution.hpp" //访问底盘控制器用于串口调参

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <stdint.h>
#include <stdio.h>

osThreadId_t CAN1_Send_TaskHandle;
osThreadId_t CAN2_Send_TaskHandle;
osThreadId_t CAN3_Send_TaskHandle;
osThreadId_t uart3SendTaskHandle;
osThreadId_t uart2ProcessTaskHandle;
osThreadId_t uart3ProcessTaskHandle;
osThreadId_t uart4ProcessTaskHandle;
osThreadId_t uart1ProcessTaskHandle;
osThreadId_t uart6ProcessTaskHandle;
osThreadId_t uart8ProcessTaskHandle;
osThreadId_t uart9ProcessTaskHandle;
osThreadId_t uart5ProcessTaskHandle;
osThreadId_t uart10ProcessTaskHandle;
osThreadId_t usbcdcProcessTaskHandle;
osThreadId_t DebugSerialTaskHandle;
osThreadId_t usbcdcSendTaskHandle;
osThreadId_t omniIrSendTaskHandle;
osThreadId_t whisperIrSendTaskHandle;

extern FDCAN_HandleTypeDef hfdcan1;
extern FDCAN_HandleTypeDef hfdcan2;
extern FDCAN_HandleTypeDef hfdcan3;

CanBus fdcan1_bus(hfdcan1);
CanBus fdcan2_bus(hfdcan2);
CanBus fdcan3_bus(hfdcan3);

// can设备

// 底盘电机
C620Motor chassis_motor1(&fdcan3_bus, 0x201, 0, 0x200, 0);
C620Motor chassis_motor2(&fdcan3_bus, 0x202, 0, 0x200, 0);
C620Motor chassis_motor3(&fdcan3_bus, 0x203, 0, 0x200, 0);
C620Motor chassis_motor4(&fdcan3_bus, 0x204, 0, 0x200, 0);

//上层电机
C610Motor arm2006_motor(&fdcan2_bus, 0x205, 0, 0x1FF, 0);
C620Motor arm3508_motor(&fdcan2_bus, 0x206, 0, 0x1FF, 0);
DM4310Motor arm4310_motor(&fdcan2_bus, 0x301, 0, 0x01, 0,
                         DM4310Motor::PosWithSpeed);


// 串口外设（回调+信号量唤醒处理线程进行解包）
void onUsbRxCb(const uint8_t *data, size_t len, void *user);

//----------------------------
// -----rm pocket（USART3）---
//----------------------------
void onUart3RxCb(const uint8_t *data, size_t len, void *user);
extern UART_HandleTypeDef huart3;

DMA_BUFFER_ATTR static uint8_t uart3_rx_dma[128];
DMA_BUFFER_ATTR static uint8_t uart3_tx_dma[128];
UartPort uart3_port(&huart3, DMA_USE::DMA_on, uart3_rx_dma, sizeof(uart3_rx_dma), uart3_tx_dma,
                    sizeof(uart3_tx_dma), onUart3RxCb, nullptr);
osSemaphoreId_t uart3_rx_semphore = NULL;

// ---------------------------
// ---IMU姿态传感器（USART2）--
// ---------------------------
void onUart2RxCb(const uint8_t *data, size_t len, void *user);
extern UART_HandleTypeDef huart2;

DMA_BUFFER_ATTR static uint8_t uart2_rx_dma[128];
DMA_BUFFER_ATTR static uint8_t uart2_tx_dma[64];
UartPort uart2_port(&huart2, DMA_USE::DMA_on, uart2_rx_dma, sizeof(uart2_rx_dma), uart2_tx_dma,
                    sizeof(uart2_tx_dma), onUart2RxCb, nullptr);
osSemaphoreId_t uart2_rx_semphore = NULL;

// ---------------------------
// ------Position（USART4）---
// ---------------------------
void onUart4RxCb(const uint8_t *data, size_t len, void *user);
extern UART_HandleTypeDef huart4;

DMA_BUFFER_ATTR static uint8_t uart4_rx_dma[128];
DMA_BUFFER_ATTR static uint8_t uart4_tx_dma[64];
UartPort uart4_port(&huart4, DMA_USE::DMA_on, uart4_rx_dma, sizeof(uart4_rx_dma), uart4_tx_dma,
                    sizeof(uart4_tx_dma), onUart4RxCb, nullptr);
osSemaphoreId_t uart4_rx_semphore = NULL;

// ---------------------------
// ------debug串口(USART5)----
// ---------------------------
void onUart5RxCb(const uint8_t *data, size_t len, void *user); //仅用于实例化不报错
extern UART_HandleTypeDef huart5;

DMA_BUFFER_ATTR static uint8_t uart5_rx_dma[64];
DMA_BUFFER_ATTR static uint8_t uart5_tx_dma[512];
UartPort uart5_port(&huart5, DMA_USE::DMA_on, uart5_rx_dma, sizeof(uart5_rx_dma), uart5_tx_dma,
                    sizeof(uart5_tx_dma), onUart5RxCb, nullptr);
osSemaphoreId_t uart5_rx_semphore = NULL;


// IMU姿态传感器解析器 及 Topic发布者
WitMotionImu wit_imu;
TypedTopicPublisher<pub_imu_data> imu_data_pub("imu_data");
pub_imu_data imu_msg{};

// rm pocket控制器（基于uart3）
rmPocket rm_pocket;
TypedTopicPublisher<pub_RM_Data> rm_data_pub("rm");
pub_RM_Data rm_msg = {
  .swA_last = RM_2_POS_SW_State_t::UP,
  .swB_last = RM_3_POS_SW_State_t::UP,
  .swC_last = RM_3_POS_SW_State_t::UP,
  .swD_last = RM_2_POS_SW_State_t::UP,
  .swE_last = RM_2_POS_SW_State_t::UP,
  .trimLeft = RM_Trim_State_t::MIDDLE,
  .trimRight = RM_Trim_State_t::MIDDLE
};

// Position模块（基于uart4）
Position position;
TypedTopicPublisher<pub_Position_Data> Position_data_pub("position");
pub_Position_Data Position_msg{};

TypedTopicPublisher<pub_motor_status> motor_status_pub("motor_status");
pub_motor_status motor_status_data{};

// usb
osSemaphoreId_t usbcdc_rx_semphore = NULL;
ROSProtocol ros_protocol(nullptr, &UsbPort::Instance());

uint8_t comServiceInit() {
  // can外设初始化
  canFilterInit(&hfdcan1, FDCAN_STANDARD_ID, FDCAN_FILTER_TO_RXFIFO0, 0, 0);
  canFilterInit(&hfdcan1, FDCAN_STANDARD_ID, FDCAN_FILTER_TO_RXFIFO1, 0, 0);
  bspCanInit(&hfdcan1);
  canFilterInit(&hfdcan2, FDCAN_STANDARD_ID, FDCAN_FILTER_TO_RXFIFO0, 0, 0);
  canFilterInit(&hfdcan2, FDCAN_STANDARD_ID, FDCAN_FILTER_TO_RXFIFO1, 0, 0);
  bspCanInit(&hfdcan2);
  canFilterInit(&hfdcan3, FDCAN_STANDARD_ID, FDCAN_FILTER_TO_RXFIFO0, 0, 0);
  canFilterInit(&hfdcan3, FDCAN_STANDARD_ID, FDCAN_FILTER_TO_RXFIFO1, 0, 0);
  bspCanInit(&hfdcan3);

  // can 总线初始化
  fdcan1_bus.init();
  fdcan2_bus.init();
  fdcan3_bus.init();

  chassis_motor1.init();
  chassis_motor2.init();
  chassis_motor3.init();
  chassis_motor4.init();

  arm2006_motor.init();
  arm3508_motor.init();
  arm4310_motor.init();

  // ---- 底盘电机初始化 ----
  fdcan3_bus.registerDevice(&chassis_motor1);
  fdcan3_bus.registerDevice(&chassis_motor2);
  fdcan3_bus.registerDevice(&chassis_motor3);
  fdcan3_bus.registerDevice(&chassis_motor4);

  // ---- 上层电机初始化 ----
  fdcan2_bus.registerDevice(&arm2006_motor);
  fdcan2_bus.registerDevice(&arm3508_motor);
  fdcan2_bus.registerDevice(&arm4310_motor);

  
  // 串口外设
  uart2_rx_semphore = osSemaphoreNew(1, 0, NULL);
  if (uart2_rx_semphore == NULL || uart2_port.startRx() != HAL_OK) {
    return 1;
  }
  
  uart3_rx_semphore = osSemaphoreNew(1, 0, NULL);
  if (uart3_rx_semphore == NULL || uart3_port.startRx() != HAL_OK) {
    return 1;
  }

  uart4_rx_semphore = osSemaphoreNew(1, 0, NULL);
  if (uart4_rx_semphore == NULL || uart4_port.startRx() != HAL_OK) {
    return 1;
  }

  uart5_rx_semphore = osSemaphoreNew(1, 0, NULL);
  if (uart5_rx_semphore == NULL || uart5_port.startRx() != HAL_OK) {
    return 1;
  }

  // position模块初始化
  position.init();

  // usb 外设
  usbcdc_rx_semphore = osSemaphoreNew(1, 0, NULL);
  if (usbcdc_rx_semphore == NULL) {
    return 1;
  }
  ros_protocol.init();
  UsbPort::Instance().SetRxCallback(onUsbRxCb, NULL);
  return 0;
}

void onUart2RxCb(const uint8_t *data, size_t len, void *user) {
  (void)user;
  if (data != nullptr && len > 0 && uart2_rx_semphore != NULL) {
    (void)osSemaphoreRelease(uart2_rx_semphore);
  }
}

void onUart3RxCb(const uint8_t *data, size_t len, void *user) {
  (void)user;
  if (data != nullptr && len > 0 && uart3_rx_semphore != NULL) {
    (void)osSemaphoreRelease(uart3_rx_semphore);
  }
}

void onUart4RxCb(const uint8_t *data, size_t len, void *user) {
  (void)user;
  if (data != nullptr && len > 0 && uart4_rx_semphore != NULL) {
    (void)osSemaphoreRelease(uart4_rx_semphore);
  }
}

void onUart5RxCb(const uint8_t *data, size_t len, void *user) {
  (void)user;
  if (data != nullptr && len > 0 && uart5_rx_semphore != NULL) {
    (void)osSemaphoreRelease(uart5_rx_semphore);
  }
}

void onUsbRxCb(const uint8_t *data, size_t len, void *user) {
  (void)user;
  if (data != nullptr && len > 0 && usbcdc_rx_semphore != NULL) {
    (void)osSemaphoreRelease(usbcdc_rx_semphore);
  }
}

void can1SendTask(void *argument) {
  TickType_t currentTime = xTaskGetTickCount();

  for (;;) {

    vTaskDelayUntil(&currentTime, 1); // 每1ms执行一次发送任务
  }
}


void can2SendTask(void *argument) {
  TickType_t currentTime = xTaskGetTickCount();
  CanBus::ClassicPack pack;
  pack.type = CanBus::Type::STANDARD;

  uint8_t len = 8;
  const uint32_t arm_motor_ids[4] = {0x205, 0x206, 0x207, 0x208};
  for (;;) {
    pack.id = 0x1FF; // DJI Group 2
    // 当前仅有 0x201(arm2006) 和 0x203(arm3508)，其余槽位置 0
    int16_t commands[4] = {0};

    // arm motor
    commands[0] = static_cast<int16_t>(arm2006_motor.cmdTrans()); // 0x201
    commands[1] = static_cast<int16_t>(arm3508_motor.cmdTrans()); // 0x203
    commands[2] = static_cast<int16_t>(0); // 0x203
    commands[3] = static_cast<int16_t>(0); // 0x204
    packDJIMotorCanMsg(pack.id, arm_motor_ids, commands, 4, pack.data, len);
    // fdcan2_bus.addCanMsg(pack);

    vTaskDelayUntil(&currentTime, 1); // 每1ms执行一次发送任务
  }
}

void can3SendTask(void *argument) {
  TickType_t currentTime = xTaskGetTickCount();
  CanBus::ClassicPack pack;
  pack.type = CanBus::Type::STANDARD;

  uint8_t len = 8;  
  const uint32_t chassis_motor_ids[4] = {0x201, 0x202, 0x203, 0x204};

  for (;;) {
    // 一帧固定打包 4 个槽位：0x201~0x204
    pack.id = 0x200; // DJI Group 2

    // 当前仅有 0x201(arm2006) 和 0x203(arm3508)，其余槽位置 0
    int16_t commands[4] = {0};
    commands[0] = static_cast<int16_t>(chassis_motor1.cmdTrans()); // 0x201
    commands[1] = static_cast<int16_t>(chassis_motor2.cmdTrans()); // 0x202
    commands[2] = static_cast<int16_t>(chassis_motor3.cmdTrans()); // 0x203   
    commands[3] = static_cast<int16_t>(chassis_motor4.cmdTrans()); // 0x204
    packDJIMotorCanMsg(pack.id, chassis_motor_ids, commands, 4, pack.data, len);
    // arm3508_motor.manager_->addCanMsg(pack);
    fdcan3_bus.addCanMsg(pack);
    vTaskDelayUntil(&currentTime, 1); // 每1ms执行一次发送任务
  }
}

void uart3RxProcessTask(void *argument) {
  (void)argument;
  if(!rm_data_pub.IsValid()) return;

  for(;;) {
    (void)osSemaphoreAcquire(uart3_rx_semphore, osWaitForever);
    UartPort::Packet packet{};
    while(uart3_port.Read(packet)) {
      for(uint16_t i = 0; i < packet.len; ++i) {
        uint8_t frame_id = rm_pocket.processByte(packet.data[i]);
        if(frame_id != 0) {
          const auto &rm_data = rm_pocket.getRCState();
          
          rm_msg.swA_last = rm_msg.swA;
          rm_msg.swB_last = rm_msg.swB;
          rm_msg.swC_last = rm_msg.swC;
          rm_msg.swD_last = rm_msg.swD;
          rm_msg.swE_last = rm_msg.swE;

          rm_msg.trimLeft_last = rm_msg.trimLeft;
          rm_msg.trimRight_last = rm_msg.trimRight;
          
          rm_msg.joyLHori = rm_data.joyLHori;
          rm_msg.joyLVert = rm_data.joyLVert;
          rm_msg.joyRHori = rm_data.joyRHori;
          rm_msg.joyRVert = rm_data.joyRVert;
          rm_msg.swA = rm_data.swA;
          rm_msg.swB = rm_data.swB;
          rm_msg.swC = rm_data.swC;
          rm_msg.swD = rm_data.swD;
          rm_msg.swE = rm_data.swE;
          rm_msg.pot = rm_data.pot;
          rm_msg.x_cnt = rm_data.x_cnt;
          rm_msg.y_cnt = rm_data.y_cnt;
          rm_msg.cursor = rm_data.cursor;
          rm_msg.trimLeft = rm_data.trimLeft;
          rm_msg.trimRight = rm_data.trimRight;
          

          rm_data_pub.Publish(rm_msg);
        }
      }
    }
  }
}
//航模回传
void uart3SendTask(void *argument) {
  (void)argument;
  for(;;)
  {
    osDelay(osWaitForever);
  }
}

//debug串口暂时先只做发送
void uart5RxProcessTask(void *argument) {
  (void)argument;
  for(;;)
  {
    osDelay(osWaitForever);
  }
}

void DebugSerialTask(void *argument) {
  (void)argument;
  static char debug_buffer[256];
  static char title[12];


  // const PID_t& pid_LU = Omnichassis_solver.pid(OmniChassis::kLeftUp);
  // const PID_t& pid_RU = Omnichassis_solver.pid(OmniChassis::kRightUp);
  // const PID_t& pid_LD = Omnichassis_solver.pid(OmniChassis::kLeftDown);
  // const PID_t& pid_RD = Omnichassis_solver.pid(OmniChassis::kRightDown);

  TickType_t currentTime = xTaskGetTickCount();

  // extern Traject_chassis.watch_2;
  // extern Traject_chassis.watch_3;
  // extern Traject_chassis.watch_4;
  // extern Traject_chassis.watch_5;

  for(;;)
  {
    //电机在线检测
    title[0] =  chassis_motor1.isOffline() ? 'X' : 'O';
    title[1] =  chassis_motor2.isOffline() ? 'X' : 'O';
    title[2] =  chassis_motor3.isOffline() ? 'X' : 'O';
    title[3] =  chassis_motor4.isOffline() ? 'X' : 'O';
    title[4] =  arm2006_motor.isOffline() ? 'X' : 'O';
    title[5] =  arm3508_motor.isOffline() ? 'X' : 'O';
    title[6] =  arm4310_motor.isOffline() ? 'X' : 'O';


    motor_status_data.chassis_motor1 = chassis_motor1.isOffline();
    motor_status_data.chassis_motor2 = chassis_motor2.isOffline();
    motor_status_data.chassis_motor3 = chassis_motor3.isOffline();
    motor_status_data.chassis_motor4 = chassis_motor4.isOffline();
    motor_status_data.arm2006_motor = arm2006_motor.isOffline();
    motor_status_data.arm3508_motor = arm3508_motor.isOffline();
    motor_status_data.arm4310_motor = arm4310_motor.isOffline();

    motor_status_pub.Publish(motor_status_data);

    // int len = snprintf(debug_buffer, sizeof(debug_buffer), "%d.%02d,%d.%02d,%d.%02d,%d.%02d,%d.%02d,%d.%02d,%d.%02d,%d.%02d,%d.%03d,%d.%03d,%d.%02d\n",
    //                                           static_cast<int>(pid_LU.Ref), (static_cast<int>(abs(pid_LU.Ref * 100)))%100,
    //                                           static_cast<int>(pid_LU.Measure), (static_cast<int>(abs(pid_LU.Measure * 100)))%100,
    //                                           static_cast<int>(pid_RU.Ref), (static_cast<int>(abs(pid_RU.Ref * 100)))%100,
    //                                           static_cast<int>(pid_RU.Measure), (static_cast<int>(abs(pid_RU.Measure * 100)))%100,
    //                                           static_cast<int>(pid_LD.Ref), (static_cast<int>(abs(pid_LD.Ref * 100)))%100,
    //                                           static_cast<int>(pid_LD.Measure), (static_cast<int>(abs(pid_LD.Measure * 100)))%100,
    //                                           static_cast<int>(pid_RD.Ref), (static_cast<int>(abs(pid_RD.Ref * 100)))%100,
    //                                           static_cast<int>(pid_RD.Measure), (static_cast<int>(abs(pid_RD.Measure * 100)))%100,
    //                                           static_cast<int>(control_position.x), (static_cast<int>(abs(control_position.x * 1000)))%1000,
    //                                           static_cast<int>(control_position.y), (static_cast<int>(abs(control_position.y * 1000)))%1000,
    //                                           static_cast<int>(lateral.Ref), (static_cast<int>(abs(lateral.Ref * 100)))%100

    // int len = snprintf(debug_buffer, sizeof(debug_buffer), "%d.%02d,%d.%02d,%d.%02d,%d.%02d,%d.%02d\n",
    //                                               static_cast<int>(Traject_chassis.watch_1), (static_cast<int>(abs(Traject_chassis.watch_1 * 100)))%100,
    //                                               static_cast<int>(Traject_chassis.watch_2), (static_cast<int>(abs(Traject_chassis.watch_2 * 100)))%100,
    //                                               static_cast<int>(Traject_chassis.watch_3), (static_cast<int>(abs(Traject_chassis.watch_3 * 100)))%100,
    //                                               static_cast<int>(Traject_chassis.watch_4), (static_cast<int>(abs(Traject_chassis.watch_4 * 100)))%100,
    //                                               static_cast<int>(Traject_chassis.watch_5), (static_cast<int>(abs(Traject_chassis.watch_5 * 100)))%100
    // int len = snprintf(debug_buffer, sizeof(debug_buffer), "%d.%02d,%d.%02d,%d.%02d\n",
    //                                           static_cast<int>(robot_v_aim_cmd.linear_x_), (static_cast<int>(abs(robot_v_aim_cmd.linear_x_ * 100)))%100,
    //                                           static_cast<int>(robot_v_aim_cmd.linear_y_), (static_cast<int>(abs(robot_v_aim_cmd.linear_y_ * 100)))%100,
    //                                           static_cast<int>(robot_v_aim_cmd.omega_), (static_cast<int>(abs(robot_v_aim_cmd.omega_ * 100)))%100


    // // );
    // int len = snprintf(debug_buffer, sizeof(debug_buffer), "%splatform: %d.%02d,%d.%02d,%d.%02d,%d.%02d\n",
    //                                           title,
    //                                           static_cast<int>(lift.platfrom_pos_pid_.Ref), (static_cast<int>(abs(lift.platfrom_pos_pid_.Ref * 100)))%100,
    //                                           static_cast<int>(lift.platfrom_pos_pid_.Measure), (static_cast<int>(abs(lift.platfrom_pos_pid_.Measure * 100)))%100,
    //                                           static_cast<int>(lift.left_v_pid_.Ref), (static_cast<int>(abs(lift.left_v_pid_.Ref * 100)))%100,
    //                                           static_cast<int>(lift.left_v_pid_.Measure), (static_cast<int>(abs(lift.left_v_pid_.Measure * 100)))%100,
    //                                           static_cast<int>(lift.right_v_pid_.Ref), (static_cast<int>(abs(lift.right_v_pid_.Ref * 100)))%100,
    //                                           static_cast<int>(lift.right_v_pid_.Measure), (static_cast<int>(abs(lift.right_v_pid_.Measure * 100)))%100
    // );
      int len = snprintf(debug_buffer, sizeof(debug_buffer), "%splatform: %d.%02d\n",
                                              title,
                                              static_cast<int>(0), (static_cast<int>(abs(0 * 100)))%100
    );
    // int len = snprintf(debug_buffer, sizeof(debug_buffer), "platform: %d.%02d,%d.%02d,%d.%02d,%d.%02d\n",
    //                                           static_cast<int>(lift.platfrom_pos_pid_.Ref), (static_cast<int>(abs(lift.platfrom_pos_pid_.Ref * 100)))%100,
    //                                           static_cast<int>(lift.platfrom_pos_pid_.Measure), (static_cast<int>(abs(lift.platfrom_pos_pid_.Measure * 100)))%100,
    //                                           static_cast<int>(lift.left_v_pid_.Ref), (static_cast<int>(abs(lift.left_v_pid_.Ref * 100)))%100,
    //                                           static_cast<int>(lift.left_v_pid_.Measure), (static_cast<int>(abs(lift.left_v_pid_.Measure * 100)))%100,
    //                                           static_cast<int>(lift.right_v_pid_.Ref), (static_cast<int>(abs(lift.right_v_pid_.Ref * 100)))%100,
    //                                           static_cast<int>(lift.right_v_pid_.Measure), (static_cast<int>(abs(lift.right_v_pid_.Measure * 100)))%100
    // );
    // HAL_UART_Transmit_DMA(&huart5, (const uint8_t *)debug_buffer, sizeof(debug_buffer));
    uart5_port.writeDma(reinterpret_cast<const uint8_t*>(debug_buffer), len);
    vTaskDelayUntil(&currentTime, 10);//10ms发送一次
  }
}

void usbCdcProcessTask(void *argument) {

  (void)argument;

  for (;;) {
    // (void)osSemaphoreAcquire(usbcdc_rx_semphore, osWaitForever);

    // UsbPort::Packet packet{};
    // while (UsbPort::Instance().Read(packet)) {
    //   // 逐个字节解析
    //   for (uint16_t i = 0; i < packet.len; ++i) {
    //     uint8_t frame_id = ros_protocol.processData(packet.data[i]);
    //     if (frame_id != 0) {
    //       switch (frame_id) {
    //         case static_cast<uint8_t>(ROSProtocol::package_id::QR_CODE_BAG): {
    //           // 发布二维码类型
    //           const auto &qr_types = ros_protocol.getQRCodeBagData().QR_type;
    //           qr_code_data.QR_type = qr_types;
    //           qr_code_data_pub.Publish(qr_code_data);
    //           //重复应答
    //           // uint8_t rev[64] = {0};
    //           // memcpy(rev, &ros_protocol.getQRCodeBagData(), sizeof(ros_protocol.getQRCodeBagData()));
    //           // UsbPort::Instance().WriteAsync(rev, sizeof(ros_protocol.getQRCodeBagData()));
    //           break;
    //         }
    //         default:
    //           break;
    //       }

    //     }
    //   }
    // }
    //原用于二维码显示相关，现屏蔽
    osDelay(osWaitForever);
  }
}

void usbCdcSendTask(void *argument) {
  (void)argument;

  // TickType_t currentTime = xTaskGetTickCount();

  // if(!qr_code_cmd_sub.IsValid()) {
  //   return;
  // }

  // if(!qr_code_data_pub.IsValid()) {
  //   return;
  // }

  // for (;;) {
  //   if(qr_code_cmd_sub.TryGet(&qr_code_cmd)) {
  //     uint8_t tx[64] = {0};
  //     uint8_t frame_length = ros_protocol.packQRMsg(tx, qr_code_cmd.QR_type);
  //     UsbPort::Instance().WriteAsync(tx, frame_length);
  //   }
  //   vTaskDelayUntil(&currentTime, 10); // 每10ms发送一次
  // }
  //原用于二维码显示相关，现屏蔽
  for(;;)
  {
    osDelay(osWaitForever);
  }
}

void uart2RxProcessTask(void *argument) {
  (void)argument;

  for (;;) {
    (void)osSemaphoreAcquire(uart2_rx_semphore, osWaitForever);

    UartPort::Packet packet{};
    while (uart2_port.Read(packet)) {
      // 逐字节喂给IMU协议解析器
      for (uint16_t i = 0; i < packet.len; ++i) {
        uint8_t frame_type = wit_imu.processByte(packet.data[i]);
        if (frame_type == 0x53) {
          // 一帧角度数据解析完毕，发布到Topic总线
          const auto &data = wit_imu.getImuData();
          imu_msg.yaw_rad = data.yaw_rad;
          imu_msg.pitch_rad = data.pitch_rad;
          imu_msg.roll_rad = data.roll_rad;
          imu_data_pub.Publish(imu_msg);
        }
      }
    }

  }
}

void uart4RxProcessTask(void *argument) {
  (void)argument;
  if (!Position_data_pub.IsValid()) {
    return;
  }

  for (;;) {
    (void)osSemaphoreAcquire(uart4_rx_semphore, osWaitForever);

    UartPort::Packet packet{};
    while (uart4_port.Read(packet)) {
      for (uint16_t i = 0; i < packet.len; ++i) {
        uint8_t frame_id = position.processByte(packet.data[i]);
        if (frame_id != 0) {
          const auto &pos_data = position.getData();
          Position_msg.frame_id = pos_data.frame_id;
          Position_msg.payload_length = pos_data.payload_length;
          Position_msg.frame_count = pos_data.frame_count;
          Position_msg.x = pos_data.x;
          Position_msg.y = pos_data.y;
          Position_msg.yaw = pos_data.yaw;
          Position_msg.yaw_speed = pos_data.yaw_speed;
          Position_data_pub.Publish(Position_msg);
        }
      }
    }
  }
}
