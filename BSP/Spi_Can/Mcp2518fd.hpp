/**
 * @file Mcp2518fd.hpp
 * @brief MCP2518FD 寄存器级驱动（SPI 由外部注入，不依赖 HAL / RTOS）
 *
 * 报文 RAM 布局（本片实测）：0x400~0x407 留空 | TXQ @0x408 | FIFO1 @0x418（每条 16 B）
 * CiTXQUA / CiFIFOUA1 只读，回读值是"相对 0x400 的偏移"，不能当地址写
 */
#pragma once

#include "ICanTransport.hpp"
#include "mcp2518fd_regs.hpp"

#include <cstdint>

namespace BspCan {

// SPI 通道，由 MCU 侧实现；一次调用完成一次全双工收发
struct SpiPort {
  void *ctx = nullptr;
  bool (*xfer)(void *ctx, const uint8_t *tx, uint8_t *rx, uint16_t len) = nullptr;
};

// 编码值 = 实际长度 - 1
struct BitTiming {
  uint8_t brp = 0;
  uint8_t tseg1 = 0;
  uint8_t tseg2 = 0;
  uint8_t sjw = 0;
};

struct TimingConfig {
  BitTiming nominal;
  BitTiming data;  // 仅 BRS=1 的 FD 帧
};

class Mcp2518fd {
public:
  static constexpr uint8_t kRxFifoIndex = 1;
  static constexpr uint8_t kTxDepth = 1;     // 本片只验证过 TXQ 单槽位
  static constexpr uint8_t kRxDepth = 16;    // 1 kHz 反馈下留 3 倍余量
  static constexpr uint8_t kMaxPayload = 8;  // PLSIZE=8，对象 = 8 头 + 8 数据

  struct Config {
    uint32_t sysclkHz = 20000000;
    bool usePll = false;   // 4 MHz 晶振开 x10 PLL 时置 true
    bool isoCanFd = true;  // 必须与对端一致
    TimingConfig timing;
    uint32_t modeTimeoutLoops = 2000;
  };

  struct Status {
    uint8_t tec = 0;
    uint8_t rec = 0;
    bool busOff = false;
  };

  explicit Mcp2518fd(SpiPort spi) : spi_(spi) {}

  bool init(const Config &cfg);

  // 本片 DEVID 恒回 0，判活请用 linkSelfTest()
  bool readDevId(uint8_t &id, uint8_t &rev);
  bool spiReset() const;
  bool linkSelfTest();

  bool pushTx(const FdFrame &frame);
  bool popRx(FdFrame &frame);
  bool txQueueFull() const;

  uint32_t readInterrupt();
  void clearInterrupt();

  // bit16=NBIT0ERR 收发器没驱动总线；bit18=NACKERR 没人应答
  uint32_t readBDiag1() const { return readSfr(mcp2518fd::kRegC1BDiag1); }

  bool busHealthy() const;
  Status status() const;

  bool spiHealthy() const { return xferFail_ == 0U; }
  uint32_t spiFailCount() const { return xferFail_; }
  uint32_t rxOverflowCount() const { return rxOvCount_; }

  // 0xFF = 全部成功；1=SPI复位 2=OSCRDY 3=OSC/PLL 4=配置模式 5=CICON
  // 6=位时序 7=TXQ 8=RX FIFO 9=中断 10=Normal
  uint8_t initStep() const { return initStep_; }

  static bool calcTiming(uint32_t sysclkHz, uint32_t bitrate,
                         uint8_t samplePointPct, BitTiming &out,
                         bool dataPhase = false);

private:
  bool xfer(const uint8_t *tx, uint8_t *rx, uint16_t len) const;
  uint32_t readSfr(uint16_t addr) const;
  void writeSfr(uint16_t addr, uint32_t val) const;
  void readRam(uint16_t addr, uint32_t *words, uint8_t count) const;
  void writeRam(uint16_t addr, const uint32_t *words, uint8_t count) const;
  bool ramProbe(uint16_t addr, uint32_t value, uint32_t &readBack);
  bool waitFlag(uint16_t addr, uint32_t mask) const;

  bool setMode(uint32_t reqOp);
  bool configTxQueue();
  bool configRxFifo();
  bool verifyFifoConfig(uint16_t addr, uint32_t expect) const;
  bool configFilter(uint8_t n, bool ext, uint32_t id, uint32_t mask);

  SpiPort spi_;
  Config cfg_;
  uint32_t irqEn_ = 0;  // 已使能的中断位，清标志时原样写回
  mutable uint32_t xferFail_ = 0;
  mutable uint32_t rxOvCount_ = 0;
  mutable uint8_t initStep_ = 0;
  mutable uint32_t txqBusyStreak_ = 0;

  static constexpr uint8_t kObjWords = 4;  // 8 头 + 8 数据
  static constexpr uint16_t kObjBytes = kObjWords * 4;
  static constexpr uint16_t kTxRamBase = 0x408;
  static constexpr uint16_t kRxRamBase = 0x418;
  static constexpr uint32_t kTxqBusyLimit = 60;  // 60 x 5 ms = 300 ms 后强制放行

  mutable uint8_t rxIdx_ = 0;  // UA 回读不合法时的兜底索引
};

} // namespace BspCan
