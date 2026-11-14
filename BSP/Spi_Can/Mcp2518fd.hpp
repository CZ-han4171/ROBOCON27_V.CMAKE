/**
 * @file Mcp2518fd.hpp
 * @brief MCP2518FD CAN FD 控制器寄存器级驱动
 * @version 0.1
 * @date 2026-11-07
 *
 * @copyright Copyright (c) 2026
 *
 * @attention : 本驱动不依赖 HAL 和 RTOS，SPI 收发由外部注入
 * @note : 1. 所有等待都用循环计数，初始化阶段可以在关中断环境下调用
 *         2. 位时序由外部给出的 SYSCLK 和目标波特率换算，见 calcTiming
 *         3. 未实现：TEF、时间戳、TDC、ECC、低功耗唤醒
 * @versioninfo :
 */
#pragma once

#include "ICanTransport.hpp"
#include "mcp2518fd_regs.hpp"

#include <cstdint>

namespace BspCan {

/**
 * @brief SPI 通道，MCU 侧实现
 *
 * 一次调用完成一次全双工收发，片选由实现内部控制。
 */
struct SpiPort {
  void *ctx = nullptr;
  // 返回 false 表示 HAL 层出错（超时 / 忙）
  bool (*xfer)(void *ctx, const uint8_t *tx, uint8_t *rx, uint16_t len) = nullptr;
};

// 单个位时序段，编码值 = 实际长度 - 1（手册：0000 0000 表示 1 个 TQ）
struct BitTiming {
  uint8_t brp = 0;    // 波特率预分频 - 1
  uint8_t tseg1 = 0;  // PropSeg + PhaseSeg1 - 1
  uint8_t tseg2 = 0;  // PhaseSeg2 - 1
  uint8_t sjw = 0;    // 同步跳宽 - 1
};

struct TimingConfig {
  BitTiming nominal;  // 仲裁段
  BitTiming data;     // 数据段，只对 BRS=1 的帧生效
};

class Mcp2518fd {
public:
  // 接收用 C1FIFO1，改这里要同步改 CanBus 层
  static constexpr uint8_t kRxFifoIndex = 1;
  static constexpr uint8_t kTxDepth = 4;
  static constexpr uint8_t kRxDepth = 8;

  struct Config {
    uint32_t sysclkHz = 40000000;  // 40MHz 直供或 4MHz 晶振开 PLL 后都是 40M
    bool usePll = false;           // 4MHz 晶振 + PLL x10 时置 true
    bool isoCanFd = true;          // 与对端节点必须一致，配错 CRC 算法不同
    TimingConfig timing;
    uint32_t modeTimeoutLoops = 20000;  // 模式切换的循环上限，一次循环约一次 SFR 读
  };

  // C1TREC 快照
  struct Status {
    uint8_t tec = 0;     // 发送错误计数
    uint8_t rec = 0;     // 接收错误计数
    bool busOff = false;
  };

  explicit Mcp2518fd(SpiPort spi) : spi_(spi) {}

  // 复位 -> 时钟 -> 位时序 -> FIFO/过滤器 -> 正常模式
  bool init(const Config &cfg);

  // 读芯片 ID，[3:0]=REV [7:4]=ID，用来确认 SPI 链路是通的
  bool readDevId(uint8_t &id, uint8_t &rev);

  // ---- 发送 / 接收 ----
  bool pushTx(const FdFrame &frame);
  bool popRx(FdFrame &frame);
  bool txQueueFull() const;

  // ---- 中断与状态 ----
  uint32_t readInterrupt();
  void clearInterrupt(uint32_t flags);
  uint32_t readFifoStatus(uint8_t fifo) const;
  bool busHealthy() const;
  void errorCounters(uint8_t &tec, uint8_t &rec) const;
  Status status() const;

  /**
   * @brief 由 SYSCLK 和目标波特率算出一组位时序
   *
   * @param sysclkHz        芯片 SYSCLK
   * @param bitrate         目标波特率
   * @param samplePointPct  采样点百分比
   * @param out             计算结果，失败时不改写
   * @return true = 有解，false = 这个晶振跑不了这个波特率
   */
  static bool calcTiming(uint32_t sysclkHz, uint32_t bitrate,
                         uint8_t samplePointPct, BitTiming &out);

private:
  uint32_t readSfr(uint16_t addr) const;
  void writeSfr(uint16_t addr, uint32_t val) const;
  void readRam(uint16_t addr, uint32_t *words, uint8_t count) const;
  void writeRam(uint16_t addr, const uint32_t *words, uint8_t count) const;

  // 轮询某个寄存器直到 mask 位置起
  bool waitFlag(uint16_t addr, uint32_t mask) const;
  // 轮询某个寄存器直到 mask 位清掉（FRESET 这类自清零位用）
  bool waitClear(uint16_t addr, uint32_t mask) const;

  bool setMode(uint32_t reqOp);
  bool configTxQueue();
  bool configRxFifo();
  bool configFilter(uint8_t n, bool ext, uint32_t id, uint32_t mask);

  SpiPort spi_;
  Config cfg_;
  // 已使能的中断位，写 C1INT 清标志时必须一起写回，否则 IE 会被清零
  uint32_t irqEn_ = 0;
};

} // namespace BspCan
