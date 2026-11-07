/**
 * @file Canbus.hpp
 * @author Keten (2863861004@qq.com)
 * @brief fdcan 总线抽象
 * @version 0.1
 * @date 2026-04-18
 *
 * @copyright Copyright (c) 2026
 *
 * @attention :
 * @note :
 * @versioninfo :
 */
#pragma once

#include "stm32h7xx_hal.h"

#include "fdcan.h"

#include "FdcanTransport.hpp"
#include "ICanTransport.hpp"
#include "lockfree_queue.hpp"
#include "stm32h7xx_hal_fdcan.h"
#include <atomic>
#include <cstdint>

void canFilterInit(FDCAN_HandleTypeDef *hcan, uint32_t idType, uint32_t fifo,
                   uint32_t id, uint32_t maskId);

void bspCanInit(FDCAN_HandleTypeDef *hcan);

class CanBus;

/**
 * @brief Can设备基类
 *
 */
class CanDevice {
public:
  CanDevice(CanBus *manager, uint32_t id = 0, bool is_extid = false,
            uint32_t tx_id = 0, bool tx_is_extid = false)
      : is_extid_(is_extid), id_(id), tx_id_(tx_id), tx_is_extid_(tx_is_extid),
        manager_(manager) {}

  virtual ~CanDevice() = default;

  uint32_t id() const { return id_; }

  bool isExtId() const { return is_extid_; }

  void setCanId(uint32_t id, bool is_extid) {
    id_ = id;
    is_extid_ = is_extid;
  }

  virtual bool matchRx(const FDCAN_RxHeaderTypeDef &rx_header) const {
    const bool frame_is_ext = (rx_header.IdType == FDCAN_EXTENDED_ID);
    if (frame_is_ext != is_extid_) {
      return false;
    }
    return rx_header.Identifier == id_;
  }

  virtual void onRx(const uint8_t data[8], uint8_t len) {
    (void)data;
    (void)len;
  }

  virtual bool buildTx(uint8_t data[8], uint8_t &len) {
    (void)data;
    len = 0;
    return false;
  }

  bool is_extid_;
  uint32_t id_;

  // 发送id
  uint32_t tx_id_;
  bool tx_is_extid_;

  // 保存一个Manager引用
  CanBus *manager_{nullptr};
};

/**
 * @brief Can 总线Manager类
 *
 */
class CanBus {
public:
  enum class Type : uint8_t { STANDARD = 0, EXTENDED = 1 };

  typedef struct __attribute__((packed)) {
    uint32_t id;
    Type type;
    uint8_t data[8];
    uint32_t DLC{8};
  } ClassicPack;

  // 片上 FDCAN：内部包一层 FdcanTransport，外部写法不变
  explicit CanBus(FDCAN_HandleTypeDef &impl)
      : fdcan_(&impl), backend_(&fdcan_), impl_(&impl) {}

  // 外挂控制器（SPI 等）：后端由调用方提供，上层用法和片上一模一样
  explicit CanBus(BspCan::ICanTransport &backend)
      : backend_(&backend), impl_(nullptr) {}

  static CanBus *instanceByIndex(uint8_t idx);

  uint8_t init(void);

  uint8_t registerDevice(CanDevice *device);

  uint8_t addCanMsg(const ClassicPack &pack);

  // 处理接收中断（FDCAN 用，中断上下文）
  void processRxInterrupt(uint32_t fifo);

  // 主动搬一轮接收：SPI 后端只能在任务里轮询，FDCAN 也可以拿来兜底
  void pollRx();

  // 处理发送中断
  void processTxInterrupt();

  uint32_t hardwareTxQueueEmptySize() { return backend_->txFreeLevel(); }

  // 保证操作串行化
  void txService();

  // 入队成功后的回调。SPI 外挂控制器只能在它自己的任务里发帧，
  // 装上回调后 addCanMsg() 只入队 + 唤醒任务，不在这里碰硬件；
  // 片上 FDCAN 不用装，保持"入队后顺手试发一次"的老行为。
  void setTxNotify(void (*fn)(void)) { tx_notify_ = fn; }

  // 调试计数，四条总线通用
  uint32_t rxFrameCount() const { return rx_frames_; }
  uint32_t rxNoMatchCount() const { return rx_no_match_; }
  uint32_t txFrameCount() const { return tx_frames_; }
  uint32_t txFailCount() const { return tx_fail_; }
  uint32_t lastRxId() const { return last_rx_id_; }
  uint8_t lastRxLen() const { return last_rx_len_; }

private:
  // 一帧收到后按 ID 分发给设备，FDCAN 和 SPI 共用同一套
  void dispatch(const BspCan::FdFrame &frame);

  // 片上 FDCAN 后端，走 SPI 总线时不会被用到
  BspCan::FdcanTransport fdcan_;
  // 实际使用的后端
  BspCan::ICanTransport *backend_;
  // hal 实例，只给中断路由用，SPI 总线时为 nullptr
  FDCAN_HandleTypeDef *impl_;
  // 维护发送队列
  Algorithm::MpscQueue<ClassicPack, 8> tx_queue_;

  std::atomic<uint32_t> tx_lock_{0};
  std::atomic<uint32_t> tx_pend_{0};
  void (*tx_notify_)(void) = nullptr;

  // 总共有3个fdcan
  static CanBus *map[3];

  // can总线最多挂12个设备
  static constexpr uint8_t kMaxDevices = 12;
  CanDevice *device_[kMaxDevices];

  // 一轮轮询最多搬多少帧，剩下的下一轮继续
  static constexpr uint8_t kMaxRxPerPoll = 8;

  // 调试计数，中断里也会写，所以加 volatile
  volatile uint32_t rx_frames_ = 0;
  volatile uint32_t rx_no_match_ = 0;
  volatile uint32_t tx_frames_ = 0;
  volatile uint32_t tx_fail_ = 0;
  volatile uint32_t last_rx_id_ = 0;
  volatile uint8_t last_rx_len_ = 0;

  uint32_t txMailbox;
};
