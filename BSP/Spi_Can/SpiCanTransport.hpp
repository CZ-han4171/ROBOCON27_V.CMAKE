/**
 * @file SpiCanTransport.hpp
 * @brief ICanTransport 的 SPI 实现：STM32 SPI4 + MCP2518FD
 * @version 0.1
 * @date 2026-11-07
 *
 * @copyright Copyright (c) 2026
 *
 * @attention : 所有 SPI 事务必须在同一个任务里发起，片选、总线、芯片内部状态都不允许并发访问
 * @note : 1. 发送是"写 RAM + 置 TXREQ"，由芯片自己发，没有返回值可查
 *         2. 接收要先轮询 FIFO 非空，INT 引脚只是提示
 * @versioninfo :
 */
#pragma once

#include "ICanTransport.hpp"
#include "Mcp2518fd.hpp"

#include "stm32h7xx_hal.h"

namespace BspCan {

// 引脚定义，改硬件只改这里
struct SpiCanPins {
  GPIO_TypeDef *csPort = nullptr;
  uint16_t csPin = 0;
  GPIO_TypeDef *intPort = nullptr;  // INT#，低有效
  uint16_t intPin = 0;
  GPIO_TypeDef *rstPort = nullptr;  // 没引出就留空，靠 SPI 复位指令
  uint16_t rstPin = 0;
};

class SpiCanTransport : public ICanTransport {
public:
  struct Options {
    uint32_t sysclkHz = 20000000;  // 板上晶振 20MHz
    bool usePll = false;           // 4MHz 晶振 + PLL 时置 true
    uint32_t nominalBitrate = 1000000;
    // 20MHz / 2Mbps 只有 10 tq，再高会低于 calcTiming 的 8 tq 下限
    uint32_t dataBitrate = 2000000;
    uint8_t nominalSamplePoint = 80;
    uint8_t dataSamplePoint = 75;
    bool isoCanFd = true;  // 与对端节点必须一致
  };

  SpiCanTransport(SPI_HandleTypeDef &hspi, const SpiCanPins &pins,
                  const Options &opt);

  bool start() override;
  bool send(const FdFrame &frame) override;
  bool recv(FdFrame &frame) override;
  void onInterrupt() override;
  bool busHealthy() const override;
  void errorCounters(uint8_t &tec, uint8_t &rec) const override;

  // INT# 累计次数，用来看中断到底有没有来
  uint32_t irqCount() const { return irqCount_; }

  Mcp2518fd &chip() { return chip_; }

private:
  static bool spiXfer(void *ctx, const uint8_t *tx, uint8_t *rx, uint16_t len);
  void select(bool active);

  SPI_HandleTypeDef &hspi_;
  SpiCanPins pins_;
  Options opt_;
  Mcp2518fd chip_;
  volatile uint32_t irqCount_ = 0;
};

} // namespace BspCan
