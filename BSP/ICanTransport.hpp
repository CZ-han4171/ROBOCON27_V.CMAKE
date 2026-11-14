/**
 * @file ICanTransport.hpp
 * @brief CAN 传输层接口，片上 FDCAN 和 SPI 外挂控制器共用
 * @version 0.1
 * @date 2026-11-07
 *
 * @copyright Copyright (c) 2026
 *
 * @attention :
 * @note : 只依赖 cstdint，不引 HAL，可以在纯算法环境里单独编译
 *         实现方只负责"把一帧塞进硬件 / 从硬件取一帧"，
 *         设备匹配、发送队列、ID 分发统一留在 CanBus 层
 * @versioninfo :
 */
#pragma once

#include <cstdint>

namespace BspCan {

// CAN FD 单帧最大载荷
static constexpr uint8_t kFdMaxPayload = 64;

/**
 * @brief 统一帧结构，经典帧和 FD 帧共用
 *
 * data 固定 64 字节，经典帧只用前 8 个，栈上开销 72 字节。
 */
struct FdFrame {
  uint32_t id = 0;    // 11 位标准 ID 或 29 位扩展 ID
  uint8_t len = 0;    // 实际字节数 0 ~ 64
  bool ext = false;   // true = 扩展帧
  bool brs = false;   // true = 数据段变速，仅 FD 帧有效
  bool fdf = false;   // true = CAN FD 帧，false = 经典帧
  bool rtr = false;   // 远程帧，FD 里不允许，留字段备用
  uint8_t data[kFdMaxPayload] = {0};
};

// 字节数转 DLC，9 ~ 15 分别对应 12/16/20/24/32/48/64
inline uint8_t lenToDlc(uint8_t len) {
  if (len <= 8) {
    return len;
  }
  if (len <= 12) {
    return 9;
  }
  if (len <= 16) {
    return 10;
  }
  if (len <= 20) {
    return 11;
  }
  if (len <= 24) {
    return 12;
  }
  if (len <= 32) {
    return 13;
  }
  if (len <= 48) {
    return 14;
  }
  return 15;
}

// DLC 转字节数
inline uint8_t dlcToLen(uint8_t dlc) {
  static const uint8_t kDlcTable[16] = {0, 1, 2, 3, 4, 5, 6, 7,
                                        8, 12, 16, 20, 24, 32, 48, 64};
  return kDlcTable[dlc & 0x0F];
}

/**
 * @brief 传输层接口
 *
 * 实现方只管把一帧塞进硬件、从硬件取一帧，
 * 设备匹配、发送队列、路由留在 CanBus 层。
 */
class ICanTransport {
public:
  virtual ~ICanTransport() = default;

  // 初始化并进入正常工作模式，返回 false 表示芯片没应答
  virtual bool start() = 0;

  // 非阻塞发送，返回 false = 硬件发送队列满或帧非法
  virtual bool send(const FdFrame &frame) = 0;

  // 非阻塞接收，返回 false = 当前没有帧
  virtual bool recv(FdFrame &frame) = 0;

  // 硬件还能塞几帧，0 = 这一轮别从队列出队（出队了也塞不进去）
  virtual uint32_t txFreeLevel() = 0;

  // 带通道号的接收：片上 FDCAN 分 FIFO0/FIFO1，SPI 只有一个 RX FIFO
  virtual bool recvFrom(uint8_t channel, FdFrame &frame) {
    (void)channel;
    return recv(frame);
  }

  // 中断上下文调用，只允许置标志这类极短动作
  virtual void onInterrupt() = 0;

  // false = bus-off，需要重新初始化
  virtual bool busHealthy() const = 0;

  // 错误计数快照，调试用
  virtual void errorCounters(uint8_t &tec, uint8_t &rec) const = 0;
};

} // namespace BspCan
