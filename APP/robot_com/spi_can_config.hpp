/**
 * @file spi_can_config.hpp
 * @brief 第 4 路 CAN：STM32H723 SPI4 + MCP2518FD（与三路片上 FDCAN 隔离）
 *
 * 调用方法见本文件末尾 "如何调用"
 * @attention 所有 SPI 事务只在 can4SendTask 里发起，芯片初始化也在该任务内完成
 */
#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include "cmsis_os.h"

// 第 4 路 CAN 任务：5 ms 周期，同时被 INT# 和"有帧要发"唤醒
void can4SendTask(void *argument);

#ifdef __cplusplus
}
#endif

#ifdef __cplusplus

#include "Canbus.hpp"
#include "SpiCanTransport.hpp"
#include "lockfree_queue.hpp"

namespace SpiCanApp {

// 初始化进度：失败时用来区分"SPI 不通"和"SPI 通了但配置失败"
enum class Phase : uint8_t {
  Init = 0,
  SpiDead = 1,     // SPI / 报文 RAM 往返失败：接线、片选、供电、晶振
  ConfigFail = 2,  // SPI 通但配置失败：波特率组合、模式切换
  Running = 3,
};

// 调试快照，Ozone / J-Link 直接看
struct Stats {
  Phase phase = Phase::Init;
  uint32_t rxFrames = 0;
  uint32_t txFrames = 0;  // 成功推给芯片的帧数
  uint32_t txLost = 0;    // 推给芯片失败（涨但 tec 不动 = 帧没上总线）
  uint32_t rxLost = 0;    // RX FIFO 溢出次数
  uint32_t irqCount = 0;
  uint32_t spiFail = 0;  // 非 0 会触发重新初始化
  uint32_t lastRxId = 0;
  uint8_t tec = 0;  // 恒 0 = 总线上有节点在 ACK
  uint8_t rec = 0;
};

extern Stats g_stats;

// 初始化停在第几步（0xFF = 全部成功）、RX 自愈次数、已静默时长
extern volatile uint8_t g_initStep;
extern volatile uint32_t g_rxRecover;
extern volatile uint32_t g_rxSilentMs;

// 设备表 + 发送队列（消费者只有 can4SendTask）
class Can4Bus {
public:
  static constexpr uint8_t kMaxDevices = 12;
  static constexpr uint8_t kTxDepth = 8;

  struct Pack {
    uint32_t id = 0;
    uint8_t len = 0;
    bool ext = false;
    bool fd = false;
    bool brs = false;
    uint8_t data[8] = {0};
  };

  // 0 = 成功，1 = 参数非法，2 = 设备表已满
  uint8_t registerDevice(CanDevice *device);

  bool addCanMsg(const Pack &pack);
  bool addCanMsg(uint32_t id, const uint8_t data[8], uint8_t len,
                 bool ext = false, bool fd = false, bool brs = false);

  void setTxNotify(void (*fn)(void)) { tx_notify_ = fn; }

  // 只能在 can4SendTask 里调用
  void txService();
  void dispatch(const BspCan::FdFrame &frame);

private:
  CanDevice *device_[kMaxDevices] = {};
  Algorithm::MpscQueue<Pack, kTxDepth> txQueue_;
  void (*tx_notify_)(void) = nullptr;
};

extern Can4Bus g_can4Bus;

// 绕过队列直发一帧，只能在 can4SendTask 里调
bool sendFrame(uint32_t id, const uint8_t *data, uint8_t len, bool ext = false,
               bool fd = false);

// 发一帧：任何任务都能调。本任务直发，别的任务入队 + 唤醒（最多晚 5 ms）
bool can4Send(uint32_t id, const uint8_t *data, uint8_t len, bool ext = false,
              bool fd = false);

uint8_t can4Attach(CanDevice *dev);

Stats snapshot();

/*
 * ============================== 如何调用 ==============================
 * 1) 任务：已经在 APP/robot_task/robot_task.cpp 里建好了（CAN4_Send_TaskHandle，
 *    osPriorityNormal，栈 1536 B），不需要自己再建。
 *
 * 2) 设备注册：已经在软件里接好，顺序不能反 ——
 *      RobotDevices_Init() 里 can4_motor1.init();      // robot_setup.cpp
 *      然后 comServiceInit() 里 can4Attach(&can4_motor1);  // com_config.cpp
 *
 * 3) 发帧（任何任务，但必须在调度器启动之后）：
 *      uint8_t data[8]; uint8_t len;
 *      const uint32_t ids[4] = {0x205, 0, 0, 0};    // 必须是本帧覆盖的反馈 ID
 *      const int16_t cmds[4] = {600, 0, 0, 0};      // 600 raw ≈ 0.73 A(C620)
 *      packDJIMotorCanMsg(0x1FF, ids, cmds, 4, data, len);   // ID5~8 用 0x1FF
 *      SpiCanApp::can4Send(0x1FF, data, len);                // ID1~4 用 0x200
 *    ⚠ tx_id 与 ids 必须同组：0x200 只认 0x201~0x204，0x1FF 只认 0x205~0x208，
 *      传错组 packDJIMotorCanMsg 会把整帧填 0（电调什么都不做）。
 *    ⚠ 跨任务最多 8 字节，满了返回 false（队列 8 帧），不阻塞。
 *    ⚠ 只在任务里调用，别放 Robot_Init（那里关中断，阻塞式 SPI 会死等）。
 *
 * 4) 收帧：注册过的设备按 matchRx(IdType + Identifier) 自动收到 onRx；
 *    调试看 g_stats.rxFrames / lastRxId / tec。
 *
 * ⚠ 不要调 motor.send() / speedControl() 那套：manager_ 传的是 nullptr 时它们
 *   直接 return，既不报错也不发帧（CAN4 没有 CanBus manager），必须用 can4Send。
 * =====================================================================
 */

} // namespace SpiCanApp

#endif
