/**
 * @file Mcp2518fd.cpp
 * @brief MCP2518FD 寄存器级驱动实现
 */
#include "Mcp2518fd.hpp"

#include <cstring>

namespace BspCan {

using namespace mcp2518fd;

namespace {

// 一个报文对象：2 个 32 位头 + 16 个 32 位数据（PLSIZE = 64 字节）
constexpr uint8_t kObjWords = 2 + 16;
constexpr uint8_t kMaxObjWords = 18;
constexpr uint8_t kBufBytes = 2 + 4 * kMaxObjWords;

inline uint32_t min32(uint32_t a, uint32_t b) { return a < b ? a : b; }

} // namespace

// ---------------------------------------------------------------- SFR 访问
uint32_t Mcp2518fd::readSfr(uint16_t addr) const {
  uint8_t tx[6] = {0};
  uint8_t rx[6] = {0};
  tx[0] = static_cast<uint8_t>((kSpiCmdRead << 4) | ((addr >> 8) & 0x0F));
  tx[1] = static_cast<uint8_t>(addr & 0xFF);
  spi_.xfer(spi_.ctx, tx, rx, sizeof(tx));
  return static_cast<uint32_t>(rx[2]) | (static_cast<uint32_t>(rx[3]) << 8) |
         (static_cast<uint32_t>(rx[4]) << 16) |
         (static_cast<uint32_t>(rx[5]) << 24);
}

void Mcp2518fd::writeSfr(uint16_t addr, uint32_t val) const {
  uint8_t tx[6];
  uint8_t rx[6] = {0};
  tx[0] = static_cast<uint8_t>((kSpiCmdWrite << 4) | ((addr >> 8) & 0x0F));
  tx[1] = static_cast<uint8_t>(addr & 0xFF);
  tx[2] = static_cast<uint8_t>(val & 0xFF);
  tx[3] = static_cast<uint8_t>((val >> 8) & 0xFF);
  tx[4] = static_cast<uint8_t>((val >> 16) & 0xFF);
  tx[5] = static_cast<uint8_t>((val >> 24) & 0xFF);
  spi_.xfer(spi_.ctx, tx, rx, sizeof(tx));
}

// ---------------------------------------------------------------- RAM 访问
void Mcp2518fd::readRam(uint16_t addr, uint32_t *words, uint8_t count) const {
  if (count > kMaxObjWords) {
    count = kMaxObjWords;
  }
  const uint16_t total = static_cast<uint16_t>(2 + 4 * count);
  uint8_t tx[kBufBytes] = {0};
  uint8_t rx[kBufBytes] = {0};
  tx[0] = static_cast<uint8_t>((kSpiCmdRead << 4) | ((addr >> 8) & 0x0F));
  tx[1] = static_cast<uint8_t>(addr & 0xFF);
  spi_.xfer(spi_.ctx, tx, rx, total);

  for (uint8_t i = 0; i < count; ++i) {
    const uint8_t *p = &rx[2 + 4 * i];
    words[i] = static_cast<uint32_t>(p[0]) | (static_cast<uint32_t>(p[1]) << 8) |
               (static_cast<uint32_t>(p[2]) << 16) |
               (static_cast<uint32_t>(p[3]) << 24);
  }
}

void Mcp2518fd::writeRam(uint16_t addr, const uint32_t *words,
                         uint8_t count) const {
  if (count > kMaxObjWords) {
    count = kMaxObjWords;
  }
  const uint16_t total = static_cast<uint16_t>(2 + 4 * count);
  uint8_t tx[kBufBytes] = {0};
  uint8_t rx[kBufBytes] = {0};
  tx[0] = static_cast<uint8_t>((kSpiCmdWrite << 4) | ((addr >> 8) & 0x0F));
  tx[1] = static_cast<uint8_t>(addr & 0xFF);
  for (uint8_t i = 0; i < count; ++i) {
    tx[2 + 4 * i + 0] = static_cast<uint8_t>(words[i] & 0xFF);
    tx[2 + 4 * i + 1] = static_cast<uint8_t>((words[i] >> 8) & 0xFF);
    tx[2 + 4 * i + 2] = static_cast<uint8_t>((words[i] >> 16) & 0xFF);
    tx[2 + 4 * i + 3] = static_cast<uint8_t>((words[i] >> 24) & 0xFF);
  }
  spi_.xfer(spi_.ctx, tx, rx, total);
}

// ---------------------------------------------------------------- 模式控制
bool Mcp2518fd::waitFlag(uint16_t addr, uint32_t mask) const {
  for (uint32_t i = 0; i < cfg_.modeTimeoutLoops; ++i) {
    if ((readSfr(addr) & mask) != 0) {
      return true;
    }
  }
  return false;
}

bool Mcp2518fd::waitClear(uint16_t addr, uint32_t mask) const {
  for (uint32_t i = 0; i < cfg_.modeTimeoutLoops; ++i) {
    if ((readSfr(addr) & mask) == 0) {
      return true;
    }
  }
  return false;
}

bool Mcp2518fd::setMode(uint32_t reqOp) {
  uint32_t con = readSfr(kRegC1Con);
  con = (con & ~kConReqOpMask) | ((reqOp & 0x7u) << 24);
  writeSfr(kRegC1Con, con);

  for (uint32_t i = 0; i < cfg_.modeTimeoutLoops; ++i) {
    const uint32_t cur = readSfr(kRegC1Con);
    if (((cur & kConOpModMask) >> 21) == (reqOp & 0x7u)) {
      return true;
    }
  }
  return false;
}

// ------------------------------------------------------------------ 初始化
bool Mcp2518fd::readDevId(uint8_t &id, uint8_t &rev) {
  const uint32_t v = readSfr(kRegDevId);
  id = static_cast<uint8_t>((v >> 4) & 0x0F);
  rev = static_cast<uint8_t>(v & 0x0F);
  // 全 0 或全 F 基本都是 SPI 没通：MISO 悬空、片选不对、SCK 没出来
  return (v != 0x00000000u) && (v != 0xFFFFFFFFu);
}

bool Mcp2518fd::init(const Config &cfg) {
  cfg_ = cfg;

  // SPI 复位指令，上电默认就在配置模式
  {
    uint8_t tx[2] = {static_cast<uint8_t>(kSpiCmdReset << 4), 0x00};
    uint8_t rx[2] = {0};
    spi_.xfer(spi_.ctx, tx, rx, 2);
  }

  // 等晶振起振，OSCRDY 起来才说明 20MHz 那个晶振真在振
  if (!waitFlag(kRegOsc, kOscOscReady)) {
    return false;
  }

  // 时钟源：20MHz / 40MHz 直供都关 PLL，4MHz 晶振才开 x10
  {
    uint32_t osc = readSfr(kRegOsc);
    osc &= ~(kOscPllEn | kOscOscDis | kOscSclkDiv | kOscLpmEn);
    if (cfg_.usePll) {
      osc |= kOscPllEn;
    }
    writeSfr(kRegOsc, osc);
    // PLLEN 写进去之后才等 PLLREADY，顺序反了会白等一轮超时
    if (cfg_.usePll && !waitFlag(kRegOsc, kOscPllReady)) {
      return false;
    }
  }

  // FIFO 和过滤器只有配置模式下才能改
  if (!setMode(kModeConfig)) {
    return false;
  }

  // ISO 与非 ISO 的 CRC 算法不同，配错百分之百通信不上
  {
    uint32_t con = readSfr(kRegC1Con);
    if (cfg_.isoCanFd) {
      con |= kConIsoCrcEn;
    } else {
      con &= ~kConIsoCrcEn;
    }
    con |= kConTxQEn;  // 打开发送队列
    writeSfr(kRegC1Con, con);
  }

  // 位时序：NBTCFG / DBTCFG 低 24 位都是 BRP | TSEG1 | TSEG2 | SJW
  {
    const BitTiming &n = cfg_.timing.nominal;
    writeSfr(kRegC1NBtCfg, (static_cast<uint32_t>(n.brp) << 24) |
                               (static_cast<uint32_t>(n.tseg1) << 16) |
                               (static_cast<uint32_t>(n.tseg2) << 8) |
                               static_cast<uint32_t>(n.sjw));
    const BitTiming &d = cfg_.timing.data;
    writeSfr(kRegC1DBtCfg, (static_cast<uint32_t>(d.brp) << 24) |
                               (static_cast<uint32_t>(d.tseg1 & 0x1F) << 16) |
                               (static_cast<uint32_t>(d.tseg2 & 0x0F) << 8) |
                               static_cast<uint32_t>(d.sjw & 0x0F));
  }

  if (!configTxQueue() || !configRxFifo()) {
    return false;
  }

  // 先写 1 清掉所有标志，再按需打开中断使能
  writeSfr(kRegC1Int, kIntAll);
  irqEn_ = kIntRxIe | kIntRxOvIe | kIntCerIe | kIntSerIe;
  writeSfr(kRegC1Int, irqEn_);

  if (!setMode(kModeNormalFd)) {
    return false;
  }
  return true;
}

bool Mcp2518fd::configTxQueue() {
  // PLSIZE=64B，FSIZE=深度-1，优先级取中间值
  uint32_t con = (static_cast<uint32_t>(kPlSize64) << kFifoConPldSizeShift) |
                 (static_cast<uint32_t>(kTxDepth - 1) << kFifoConFSizeShift) |
                 (0x10u << kFifoConTxPriShift) | kFifoConTxEn;
  writeSfr(kRegC1TxQCon, con);
  // 再置一次 FRESET，把 head/tail 归零，等它自己清掉
  writeSfr(kRegC1TxQCon, con | kFifoConFrEsEt);
  return waitClear(kRegC1TxQCon, kFifoConFrEsEt);
}

bool Mcp2518fd::configRxFifo() {
  uint32_t con = (static_cast<uint32_t>(kPlSize64) << kFifoConPldSizeShift) |
                 (static_cast<uint32_t>(kRxDepth - 1) << kFifoConFSizeShift) |
                 kFifoConNotEmptyIe;
  // TXEN = 0 就是接收 FIFO
  writeSfr(regFifoCon(kRxFifoIndex), con);
  writeSfr(regFifoCon(kRxFifoIndex), con | kFifoConFrEsEt);
  if (!waitClear(regFifoCon(kRxFifoIndex), kFifoConFrEsEt)) {
    return false;
  }

  // 过滤器 0 掩码全 0 = 全部接收，具体 ID 分发留到 com_config 层做
  return configFilter(0, false, 0, 0);
}

bool Mcp2518fd::configFilter(uint8_t n, bool ext, uint32_t id, uint32_t mask) {
  uint32_t obj = id & (0x7FFu | (0x3FFFFu << 11));
  uint32_t msk = mask & (0x7FFu | (0x3FFFFu << 11));
  if (ext) {
    obj |= kFltExIde;
    msk |= kFltExIde;
  }
  writeSfr(regFltObj(n), obj);
  writeSfr(regFltMask(n), msk);
  // FLTCONn：bit7 = FLTENn，bit4:0 = 目标 FIFO 编号
  writeSfr(regFltCon(n), (0x1u << 7) | (kRxFifoIndex & 0x1F));
  return true;
}


// ---------------------------------------------------------------- 发送 / 接收
bool Mcp2518fd::txQueueFull() const {
  return (readSfr(kRegC1TxQSta) & kTxqStaNotFull) == 0;
}

bool Mcp2518fd::pushTx(const FdFrame &frame) {
  if (frame.len > kFdMaxPayload) {
    return false;
  }
  // 队列满就直接放弃，交给上层下一轮重发，不要在这里死等
  if (txQueueFull()) {
    return false;
  }

  const uint16_t ua =
      static_cast<uint16_t>(readSfr(kRegC1TxQUa) & 0xFFFF);
  if (ua < kRamBase || ua > kRamEnd) {
    return false;
  }

  uint32_t words[kObjWords] = {0};
  const uint32_t sid = frame.ext ? ((frame.id >> 18) & 0x7FF) : (frame.id & 0x7FF);
  const uint32_t eid = frame.ext ? (frame.id & 0x3FFFF) : 0;
  words[0] = sid | (eid << 11);
  words[1] = (static_cast<uint32_t>(lenToDlc(frame.len)) & kObjDlcMask) |
             (frame.ext ? kObjIde : 0) | (frame.rtr ? kObjRtr : 0) |
             (frame.brs ? kObjBrs : 0) | (frame.fdf ? kObjFdf : 0);

  // 数据区固定 64 字节，未用到的补 0
  std::memcpy(&words[2], frame.data, kFdMaxPayload);

  writeRam(ua, words, kObjWords);

  // 先 UINC 推进 head，再置 TXREQ 交给芯片自己发
  const uint32_t con = readSfr(kRegC1TxQCon);
  writeSfr(kRegC1TxQCon, con | kFifoConUInc);
  writeSfr(kRegC1TxQCon, con | kFifoConTxReq);
  return true;
}

bool Mcp2518fd::popRx(FdFrame &frame) {
  const uint32_t sta = readSfr(regFifoSta(kRxFifoIndex));
  if ((sta & kFifoStaNotEmptyIf) == 0) {
    return false;
  }

  const uint16_t ua =
      static_cast<uint16_t>(readSfr(regFifoUa(kRxFifoIndex)) & 0xFFFF);
  if (ua < kRamBase || ua > kRamEnd) {
    return false;
  }

  uint32_t words[kObjWords] = {0};
  readRam(ua, words, kObjWords);

  const uint32_t r0 = words[0];
  const uint32_t r1 = words[1];
  frame.ext = (r1 & kObjIde) != 0;
  frame.rtr = (r1 & kObjRtr) != 0;
  frame.brs = (r1 & kObjBrs) != 0;
  frame.fdf = (r1 & kObjFdf) != 0;
  frame.len = dlcToLen(static_cast<uint8_t>(r1 & kObjDlcMask));
  if (frame.len > kFdMaxPayload) {
    frame.len = kFdMaxPayload;
  }

  const uint32_t sid = r0 & 0x7FF;
  const uint32_t eid = (r0 >> 11) & 0x3FFFF;
  frame.id = frame.ext ? ((sid << 18) | eid) : sid;

  std::memcpy(frame.data, &words[2], kFdMaxPayload);

  // 读完再 UINC，把这个槽位还给 FIFO
  const uint32_t con = readSfr(regFifoCon(kRxFifoIndex));
  writeSfr(regFifoCon(kRxFifoIndex), con | kFifoConUInc);
  return true;
}

// ---------------------------------------------------------------- 中断与状态
uint32_t Mcp2518fd::readInterrupt() { return readSfr(kRegC1Int); }

void Mcp2518fd::clearInterrupt(uint32_t flags) {
  // IF 位写 1 清零；IE 位写 0 会关掉使能，所以要把 irqEn_ 一起写回去
  writeSfr(kRegC1Int, (flags & 0xFFFFu) | irqEn_);
}

uint32_t Mcp2518fd::readFifoStatus(uint8_t fifo) const {
  return readSfr(regFifoSta(fifo));
}

Mcp2518fd::Status Mcp2518fd::status() const {
  const uint32_t trec = readSfr(kRegC1Trec);
  Status s;
  s.tec = static_cast<uint8_t>(trec & 0xFF);
  s.rec = static_cast<uint8_t>((trec >> 8) & 0xFF);
  s.busOff = (trec & kTrecTxBo) != 0;
  return s;
}

// 只有 bus-off 才算总线不可用；错误警告阶段继续跑，计数在 status() 里看
bool Mcp2518fd::busHealthy() const { return !status().busOff; }

void Mcp2518fd::errorCounters(uint8_t &tec, uint8_t &rec) const {
  const Status s = status();
  tec = s.tec;
  rec = s.rec;
}

// ------------------------------------------------------------------ 位时序
bool Mcp2518fd::calcTiming(uint32_t sysclkHz, uint32_t bitrate,
                           uint8_t samplePointPct, BitTiming &out) {
  if (sysclkHz == 0 || bitrate == 0 || samplePointPct == 0 ||
      samplePointPct > 100) {
    return false;
  }

  // 从小往大找分频，tq 数越多采样点越细
  for (uint32_t brp = 1; brp <= 256; ++brp) {
    const uint32_t div = brp * bitrate;
    // 除不尽就换下一个，否则实际波特率和设定值对不上
    if ((sysclkHz % div) != 0) {
      continue;
    }
    const uint32_t tqTotal = sysclkHz / div;
    if (tqTotal < 8 || tqTotal > 256) {
      continue;
    }

    // 采样点位置 = 1(同步段) + TSEG1，所以 TSEG1 长度要减 1
    const uint32_t want = (tqTotal * samplePointPct + 50) / 100;
    uint32_t tseg1Len = (want <= 1) ? 1 : (want - 1);
    if (tseg1Len > tqTotal - 2) {
      tseg1Len = tqTotal - 2;  // 至少给 PhaseSeg2 留 1 个 tq
    }
    const uint32_t tseg2Len = tqTotal - 1 - tseg1Len;
    if (tseg2Len < 1) {
      continue;
    }

    out.brp = static_cast<uint8_t>(brp - 1);
    out.tseg1 = static_cast<uint8_t>(tseg1Len - 1);
    out.tseg2 = static_cast<uint8_t>(tseg2Len - 1);
    out.sjw = static_cast<uint8_t>(min32(tseg2Len, 4) - 1);
    return true;
  }
  return false;
}

} // namespace BspCan

