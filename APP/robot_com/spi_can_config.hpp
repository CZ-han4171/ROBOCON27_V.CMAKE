/**
 * @file spi_can_config.hpp
 * @brief 第 4 路 CAN FD：STM32H723 SPI4 + MCP2518FD + TCAN1044V
 * @version 0.1
 * @date 2026-11-07
 *
 * @copyright Copyright (c) 2026
 *
 * @attention : 芯片初始化放在任务里做，不要放 Robot_Init()。那里是关中断状态，
 *              HAL 阻塞式 SPI 靠 SysTick 计时超时，关中断时 HAL_GetTick() 不前进，
 *              芯片不应答就会死等
 * @note : 1. 帧长上限 64 字节，发送是"写 RAM + 置 TXREQ"，没有 Mailbox
 *         2. 接收靠 INT# 下降沿唤醒 + 任务里主动读 SPI，中断里不搬数据
 *         3. 目前是独立模块，还没接 CanBus / com_config 的路由
 * @versioninfo :
 */
#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include "cmsis_os.h"

// 第 4 路 CAN 任务，5ms 周期，同时被 INT# 唤醒
void can4SendTask(void *argument);

#ifdef __cplusplus
}
#endif

#ifdef __cplusplus

#include "SpiCanTransport.hpp"

namespace SpiCanApp {

// 初始化进度，失败时用来区分"SPI 不通"和"SPI 通了但配置失败"
enum class Phase : uint8_t {
  Init = 0,        // 还没尝试
  SpiDead = 1,     // 读不到 DEVID：接线、片选、供电、晶振
  ConfigFail = 2,  // SPI 通但配置失败：波特率组合、ISO 位、模式切换
  Running = 3,
};

// 调试快照，调试器里直接看变量，不占 CPU
struct Stats {
  Phase phase = Phase::Init;
  uint32_t rxFrames = 0;  // 累计收到帧数
  uint32_t txFrames = 0;  // 累计发出帧数
  uint32_t rxLost = 0;    // RX FIFO 溢出次数
  uint32_t irqCount = 0;  // INT# 次数
  uint32_t lastRxId = 0;  // 最后一帧的 ID
  uint8_t lastRxLen = 0;  // 最后一帧的字节数
  uint8_t devId = 0;      // DEVID，高位 ID 低位 REV
  uint8_t tec = 0;        // 发送错误计数
  uint8_t rec = 0;        // 接收错误计数
};

extern Stats g_stats;

// 唯一实例，中断回调和任务共用
BspCan::SpiCanTransport &transport();

/**
 * @brief 发送一帧，非阻塞
 *
 * @param id   11 位标准 ID 或 29 位扩展 ID
 * @param data 数据指针
 * @param len  字节数 0 ~ 64
 * @param ext  true = 扩展帧
 * @param fd   true = CAN FD 帧（默认），false = 经典帧
 * @return false = 发送队列满或参数非法
 */
bool sendFrame(uint32_t id, const uint8_t *data, uint8_t len, bool ext = false,
               bool fd = true);

Stats snapshot();

} // namespace SpiCanApp
#endif
