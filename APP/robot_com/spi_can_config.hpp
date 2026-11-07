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
 *         3. 上层接入用本文件的 Can4Device / Can4Bus，用法和 CanDevice /
 *            CanBus 一一对应（registerDevice + onRx + send），但不需要
 *            FDCAN 句柄，也不用改 Canbus.hpp
 *        4. 只做经典帧 8 字节的上层语义，和 CanDevice::onRx 保持一致
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

#include "lockfree_queue.hpp"

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
  uint32_t rxNoMatch = 0;  // 收到但没有任何 Can4Device 认领的帧
  uint32_t txLost = 0;     // 队列里的帧推给芯片时失败（FIFO 满 / SPI 出错）
};

extern Stats g_stats;

/**
 * @brief CAN4 上的设备基类
 *
 * 和 CanDevice 的用法一一对应，差别只有三点：
 *   1. matchRx 收到的是 FdFrame，不是 FDCAN_RxHeaderTypeDef
 *   2. 不需要在构造函数里传总线指针，发送走 send()，内部走 Can4Bus
 *   3. 发送先进队列，由 can4SendTask 统一推给芯片（SPI 只能在任务里跑）
 */
class Can4Device {
public:
  Can4Device(uint32_t rxId, bool rxExt = false, uint32_t txId = 0,
             bool txExt = false);
  virtual ~Can4Device() = default;

  uint32_t id() const { return id_; }
  bool isExtId() const { return rxExt_; }
  void setCanId(uint32_t id, bool ext) {
    id_ = id;
    rxExt_ = ext;
  }

  uint32_t txId() const { return txId_; }
  bool txIsExtId() const { return txExt_; }
  void setTxCanId(uint32_t id, bool ext) {
    txId_ = id;
    txExt_ = ext;
  }

  // 默认按 rxId + 标准/扩展匹配，要按 ID 段收就重写它
  virtual bool matchRx(const BspCan::FdFrame &frame) const;

  // 收到一帧，len 已截到 8 以内，语义同 CanDevice::onRx
  virtual void onRx(const uint8_t data[8], uint8_t len) {
    (void)data;
    (void)len;
  }

  // 组装本周期要发的数据，返回 false = 这一轮不发
  virtual bool buildTx(uint8_t data[8], uint8_t &len) {
    (void)data;
    len = 0;
    return false;
  }

  // 用 tx 侧 ID 发一帧，进队列并唤醒任务
  bool send(const uint8_t data[8], uint8_t len, bool fd = false, bool brs = false);

private:
  uint32_t id_;    // 接收 ID
  bool rxExt_;     // 接收 ID 是否是扩展帧
  uint32_t txId_;  // 发送 ID
  bool txExt_;     // 发送 ID 是否是扩展帧
};

/**
 * @brief CAN4 总线管理：设备表 + 发送队列
 *
 * 生产者（各任务的 Can4Device::send / addCanMsg）只入队，
 * 消费者只有 can4SendTask 一个，所以用 MPSC 无锁队列。
 */
class Can4Bus {
public:
  static constexpr uint8_t kMaxDevices = 6;
  static constexpr uint8_t kTxDepth = 8;

  struct Pack {
    uint32_t id = 0;
    uint8_t len = 0;
    bool ext = false;
    bool fd = false;
    bool brs = false;
    uint8_t data[8] = {0};
  };

  // 返回 0 = 成功，1 = 参数非法，2 = 设备表已满
  uint8_t registerDevice(Can4Device *device);
  uint8_t deviceCount() const;

  // 入队一帧，返回 false = 队列满
  bool addCanMsg(const Pack &pack);
  bool addCanMsg(uint32_t id, const uint8_t data[8], uint8_t len,
                 bool ext = false, bool fd = false, bool brs = false);

  // 把队列里的帧推给芯片，只能在 can4SendTask 里调用
  void txService();
  // 收到一帧后分发给设备，只能在 can4SendTask 里调用
  void dispatch(const BspCan::FdFrame &frame);

private:
  Can4Device *device_[kMaxDevices] = {};
  Algorithm::MpscQueue<Pack, kTxDepth> txQueue_;
};

// 全局唯一实例，和 CanBus 的三路总线并列
extern Can4Bus g_can4Bus;

/*
 * 用法（和 CanBus 那一套一一对应，不用传总线指针）：
 *
 *   // 1) 写设备：继承 Can4Device，只实现 onRx，要发就调 send()
 *   class Can4Motor : public SpiCanApp::Can4Device {
 *   public:
 *     explicit Can4Motor(uint32_t rxId) : Can4Device(rxId, false, rxId - 1) {}
 *     void onRx(const uint8_t data[8], uint8_t len) override {
 *       if (len < 8) return;
 *       // 解析 data -> speed_ / pos_
 *     }
 *     void setCurrent(int16_t cur) {
 *       uint8_t buf[8] = {0};
 *       buf[0] = (uint8_t)(cur >> 8);
 *       buf[1] = (uint8_t)(cur & 0xFF);
 *       (void)send(buf, 8);       // 入队 + 唤醒 can4SendTask
 *     }
 *   };
 *   Can4Motor can4_motor1(0x211);   // 收 0x211，发 0x210
 *
 *   // 2) 挂总线：写在 comServiceInit() 里，和 fdcanX_bus.registerDevice() 并排
 *   SpiCanApp::g_can4Bus.registerDevice(&can4_motor1);
 *
 *   // 3) 发帧：在任意任务里调，只入队不阻塞，真正推给芯片只在 can4SendTask 里做
 *   can4_motor1.setCurrent(1000);
 *
 * 一次性发帧不想建类，也可以直接：
 *   const uint8_t raw[8] = {1, 2, 3, 4, 5, 6, 7, 8};
 *   SpiCanApp::g_can4Bus.addCanMsg(0x123, raw, 8);
 *
 * 注意：只有 g_stats.phase == Phase::Running 之后队列里的帧才会被推出去。
 */

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
