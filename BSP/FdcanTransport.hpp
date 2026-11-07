/**
 * @file FdcanTransport.hpp
 * @brief ICanTransport 的片上 FDCAN 实现
 * @version 0.1
 * @date 2026-11-07
 *
 * @copyright Copyright (c) 2026
 *
 * @attention : 时基、过滤器、NVIC 仍由 CubeMX + bspCanInit() 负责，
 *              这里只做"帧 <-> 硬件寄存器"的搬运
 * @note : 经典帧最多 8 字节，FD 帧在 CanBus 上层还没放开
 * @versioninfo :
 */
#pragma once

#include "ICanTransport.hpp"
// 必须先起 HAL 总头，直接引外设头会踩 hal_def.h 的循环包含
#include "stm32h7xx_hal.h"

namespace BspCan {

/**
 * @brief 片上 FDCAN 传输后端
 *
 * 一个实例绑一个 FDCAN_HandleTypeDef，CanBus 内部持有它，
 * 所以 com_config.cpp 里 `CanBus fdcanX_bus(hfdcanX);` 的写法不用改。
 */
class FdcanTransport final : public ICanTransport {
public:
  FdcanTransport() = default;
  explicit FdcanTransport(FDCAN_HandleTypeDef *h) : h_(h) {}

  void bind(FDCAN_HandleTypeDef *h) { h_ = h; }
  FDCAN_HandleTypeDef *handle() const { return h_; }

  // FDCAN 的时钟/过滤器/启动由 bspCanInit() 做完，这里没有额外动作
  bool start() override { return h_ != nullptr; }

  bool send(const FdFrame &frame) override;
  bool recv(FdFrame &frame) override { return recvFrom(0, frame); }
  bool recvFrom(uint8_t channel, FdFrame &frame) override;
  uint32_t txFreeLevel() override;

  void onInterrupt() override {}

  bool busHealthy() const override;
  void errorCounters(uint8_t &tec, uint8_t &rec) const override;

private:
  bool readFifo(uint32_t fifo, FdFrame &frame);

  FDCAN_HandleTypeDef *h_ = nullptr;
};

} // namespace BspCan
