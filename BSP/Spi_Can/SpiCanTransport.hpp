/**
 * @file SpiCanTransport.hpp
 * @brief ICanTransport 的 SPI 实现：STM32 SPI4 + MCP2518FD
 *
 * @attention 所有 SPI 事务必须在同一个任务里发起（片选和芯片状态不能并发访问）
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
  GPIO_TypeDef *rstPort = nullptr;  // 14 脚 MCP2518FD 无 nRESET 引脚，留空即可
  uint16_t rstPin = 0;
};

class SpiCanTransport : public ICanTransport {
public:
  struct Options {
    uint32_t sysclkHz = 20000000;  // 板上晶振 20MHz
    bool usePll = false;           // 4MHz 晶振 + PLL 时置 true
    uint32_t nominalBitrate = 1000000;
    uint32_t dataBitrate = 2000000;  // 20 MHz 下只有 10 tq，再高会低于 calcTiming 下限
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
