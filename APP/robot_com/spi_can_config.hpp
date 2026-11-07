/**
 * @file spi_can_config.hpp
 * @brief 第 4 路 CAN FD：STM32H723 SPI4 + MCP2518FD + TCAN1044V
 * @version 0.2
 * @date 2026-11-07
 *
 * @copyright Copyright (c) 2026
 *
 * @attention : 芯片初始化放在任务里做，不要放 Robot_Init()。那里是关中断状态，
 *              HAL 阻塞式 SPI 靠 SysTick 计时超时，关中断时 HAL_GetTick() 不前进，
 *              芯片不应答就会死等
 * @note : 1. 帧长上限 64 字节，发送是"写 RAM + 置 TXREQ"，没有 Mailbox
 *         2. 接收靠 INT# 下降沿唤醒 + 任务里主动读 SPI，中断里不搬数据
 *         3. 上层不分家：CAN4 复用 CanBus / CanDevice，只是把 CanBus 的后端
 *            从片上 FDCAN 换成 BspCan::SpiCanTransport，所以设备类、
 *            matchRx、onRx、send、registerDevice 的用法和三路 FDCAN 一模一样
 *         4. 只做经典帧 8 字节的上层语义，和 CanDevice::onRx 保持一致
 * @versioninfo : 0.2 去掉独立的 Can4Bus/Can4Device，改为复用 CanBus
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

#include "Canbus.hpp"
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
  uint32_t txFrames = 0;  // 累计成功推给芯片的帧数
  uint32_t rxLost = 0;    // RX FIFO 溢出次数
  uint32_t irqCount = 0;  // INT# 次数
  uint32_t lastRxId = 0;  // 最后一帧的 ID
  uint8_t lastRxLen = 0;  // 最后一帧的字节数
  uint8_t devId = 0;      // DEVID，高位 ID 低位 REV
  uint8_t tec = 0;        // 发送错误计数
  uint8_t rec = 0;        // 接收错误计数
  uint32_t rxNoMatch = 0;  // 收到但没有任何设备认领的帧
  uint32_t txLost = 0;     // 出队后推给芯片失败（FIFO 满 / SPI 出错）
};

extern Stats g_stats;

/**
 * @brief 第 4 路 CAN 总线实例
 *
 * 和其它三路一样是 CanBus，区别只在内部后端：这里挂的是 SPI 外挂的
 * MCP2518FD，不是片上 FDCAN。所以：
 *   - 收帧：can4SendTask 里调 can4_bus.pollRx()，按 ID 分发给 registerDevice 的设备
 *   - 发帧：设备内部走 CanBus::addCanMsg() 入队，can4_bus.txService() 推给芯片
 *   - 入队成功会通过 setTxNotify() 立刻唤醒 can4SendTask，不用等 5ms 周期
 */
extern CanBus can4_bus;

/*
 * 用法（和 fdcanX_bus 完全一样）：
 *
 *   // 1) 设备类一行都不用改，构造时把 &can4_bus 当总线传进去
 *   C610Motor can4_motor1(&can4_bus, 0x211, false, 0x210, false);
 *
 *   // 2) 在 comServiceInit() 里注册，和三路 FDCAN 并排写
 *   can4_bus.registerDevice(&can4_motor1);
 *
 *   // 3) 发帧：设备自己的 send() 内部就是 addCanMsg + 唤醒 can4SendTask
 *   can4_motor1.setCurrent(1000);
 *
 * 不建类直接发一帧也行：
 *   CanBus::ClassicPack pack{};
 *   pack.id = 0x123;
 *   pack.type = CanBus::Type::STANDARD;
 *   std::memcpy(pack.data, raw, 8);
 *   can4_bus.addCanMsg(pack);
 *
 * 注意：只有 g_stats.phase == Phase::Running 之后，队列里的帧才会被推出去。
 */

// 唯一实例，中断回调和任务共用
BspCan::SpiCanTransport &transport();

/**
 * @brief 绕过队列直接发一帧，非阻塞（自测/回环用）
 *
 * @param id   11 位标准 ID 或 29 位扩展 ID
 * @param data 数据指针
 * @param len  字节数 0 ~ 64
 * @param ext  true = 扩展帧
 * @param fd   true = CAN FD 帧（默认），false = 经典帧
 * @return false = 芯片发送队列满或参数非法
 */
bool sendFrame(uint32_t id, const uint8_t *data, uint8_t len, bool ext = false,
               bool fd = true);

Stats snapshot();

} // namespace SpiCanApp
#endif
