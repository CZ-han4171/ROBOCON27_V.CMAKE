/**
 * @file Mcp2518fd.cpp
 * @brief MCP2518FD 寄存器级驱动实现
 */
#include "Mcp2518fd.hpp"

#include <cstring>

namespace BspCan {

using namespace mcp2518fd;

namespace {

constexpr uint8_t kMaxObjWords = 4;  // PLSIZE=8：T0/T1/D0/D1
constexpr uint8_t kBufBytes = 2 + 4 * kMaxObjWords;

inline uint32_t min32(uint32_t a, uint32_t b) { return a < b ? a : b; }

// 自检特征值：SFR 必须写"每字节最高位可写"的值，否则会被字段宽度截断
constexpr uint32_t kSelfTestSfrPattern = 0x00123456u;
constexpr uint32_t kSelfTestRamPattern = 0x5AA5C33Cu;
constexpr uint16_t kSelfTestRamAddr = 0x600;

} // namespace

// ---- SPI 事务 ----
bool Mcp2518fd::xfer(const uint8_t *tx, uint8_t *rx, uint16_t len) const {
  if (spi_.xfer == nullptr || tx == nullptr || rx == nullptr || len == 0) {
    ++xferFail_;
    return false;
  }
  if (!spi_.xfer(spi_.ctx, tx, rx, len)) {
    ++xferFail_;
    return false;
  }
  return true;
}

// ---- SFR / RAM ----
uint32_t Mcp2518fd::readSfr(uint16_t addr) const {
  uint8_t tx[6] = {0};
  uint8_t rx[6] = {0};
  tx[0] = static_cast<uint8_t>((kSpiCmdRead << 4) | ((addr >> 8) & 0x0F));
  tx[1] = static_cast<uint8_t>(addr & 0xFF);
  if (!xfer(tx, rx, sizeof(tx))) {
    return 0;  // 读失败返回 0：等标志的循环会因此超时退出
  }
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
  (void)xfer(tx, rx, sizeof(tx));
}

void Mcp2518fd::readRam(uint16_t addr, uint32_t *words, uint8_t count) const {
  if (count > kMaxObjWords) {
    count = kMaxObjWords;
  }
  const uint16_t total = static_cast<uint16_t>(2 + 4 * count);
  uint8_t tx[kBufBytes] = {0};
  uint8_t rx[kBufBytes] = {0};
  tx[0] = static_cast<uint8_t>((kSpiCmdRead << 4) | ((addr >> 8) & 0x0F));
  tx[1] = static_cast<uint8_t>(addr & 0xFF);
  if (!xfer(tx, rx, total)) {
    for (uint8_t i = 0; i < count; ++i) {
      words[i] = 0;
    }
    return;
  }
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
  (void)xfer(tx, rx, total);
}

// ---- 模式控制 ----
bool Mcp2518fd::waitFlag(uint16_t addr, uint32_t mask) const {
  const uint32_t fail_before = xferFail_;
  for (uint32_t i = 0; i < cfg_.modeTimeoutLoops; ++i) {
    if ((readSfr(addr) & mask) != 0) {
      return true;
    }
    if (xferFail_ != fail_before) {
      return false;  // SPI 已出错，别再空转等超时
    }
  }
  return false;
}

bool Mcp2518fd::setMode(uint32_t reqOp) {
  const uint32_t fail_before = xferFail_;
  uint32_t con = readSfr(kRegC1Con);
  // 读失败时 con 是 0，写回去会把 TXQEN/ISOCRCEN 全清掉
  if (xferFail_ != fail_before) {
    return false;
  }
  con = (con & ~kConReqOpMask) | ((reqOp & 0x7u) << 24);
  writeSfr(kRegC1Con, con);

  for (uint32_t i = 0; i < cfg_.modeTimeoutLoops; ++i) {
    const uint32_t cur = readSfr(kRegC1Con);
    if (((cur & kConOpModMask) >> 21) == (reqOp & 0x7u)) {
      return true;
    }
    if (xferFail_ != fail_before) {
      return false;
    }
  }
  return false;
}

// ---- 自检 ----
bool Mcp2518fd::readDevId(uint8_t &id, uint8_t &rev) {
  const uint32_t v = readSfr(kRegDevId);
  id = static_cast<uint8_t>((v >> 4) & 0xFF);
  rev = static_cast<uint8_t>(v & 0x0F);
  return (v != 0x00000000u) && (v != 0xFFFFFFFFu);
}

bool Mcp2518fd::spiReset() const {
  uint8_t tx[2] = {static_cast<uint8_t>(kSpiCmdReset << 4), 0x00};
  uint8_t rx[2] = {0};
  return xfer(tx, rx, sizeof(tx));
}

bool Mcp2518fd::ramProbe(uint16_t addr, uint32_t value, uint32_t &readBack) {
  // 报文 RAM 只接受 32 位字访问，地址必须 4 字节对齐且在 0x400~0xBFC
  if (static_cast<uint32_t>(addr) < kRamBase ||
      static_cast<uint32_t>(addr) > kRamEnd || (addr & 0x3u) != 0u) {
    return false;
  }
  const uint32_t failBefore = xferFail_;
  writeRam(addr, &value, 1);
  uint32_t got = 0;
  readRam(addr, &got, 1);
  if (xferFail_ != failBefore) {
    return false;  // 出错时读回值不可信，不能当成"芯片回了 0"
  }
  readBack = got;
  return got == value;
}

bool Mcp2518fd::linkSelfTest() {
  const uint32_t failBefore = xferFail_;

  const uint32_t nbtOld = readSfr(kRegC1NBtCfg);
  writeSfr(kRegC1NBtCfg, kSelfTestSfrPattern);
  const bool sfrOk = (readSfr(kRegC1NBtCfg) == kSelfTestSfrPattern);
  writeSfr(kRegC1NBtCfg, nbtOld);

  uint32_t ramBack = 0;
  const bool ramOk = ramProbe(kSelfTestRamAddr, kSelfTestRamPattern, ramBack);

  if (xferFail_ != failBefore) {
    return false;
  }
  return sfrOk && ramOk;
}

// ---- 初始化 ----
bool Mcp2518fd::init(const Config &cfg) {
  cfg_ = cfg;
  irqEn_ = 0;
  xferFail_ = 0;
  rxIdx_ = 0;

  // 1) 复位指令只在配置模式生效，先请求 Config
  initStep_ = 1;
  (void)setMode(kModeConfig);
  if (!spiReset()) {
    return false;
  }

  // 2) 等晶振起振，只认 OSCRDY
  initStep_ = 2;
  if (!waitFlag(kRegOsc, kOscOscReady)) {
    return false;
  }

  // 3) 时钟源：20/40 MHz 直供都关 PLL，4 MHz 晶振才开 x10
  initStep_ = 3;
  {
    uint32_t osc = readSfr(kRegOsc);
    osc &= ~(kOscPllEn | kOscOscDis | kOscSclkDiv | kOscLpmEn);
    if (cfg_.usePll) {
      osc |= kOscPllEn;
    }
    writeSfr(kRegOsc, osc);
    if (cfg_.usePll && !waitFlag(kRegOsc, kOscPllReady)) {
      return false;
    }
  }

  // 4) FIFO / 滤波器 / 位时序都只有配置模式下能改
  initStep_ = 4;
  if (!setMode(kModeConfig)) {
    return false;
  }

  // 5) C1CON：ISO CRC、TXQEN、RTXAT（让 TXQCON.TXAT 生效）
  initStep_ = 5;
  {
    uint32_t con = readSfr(kRegC1Con);
    if (cfg_.isoCanFd) {
      con |= kConIsoCrcEn;
    } else {
      con &= ~kConIsoCrcEn;
    }
    con |= kConTxQEn | kConRtxAt;
    writeSfr(kRegC1Con, con);

    // 只校验 TXQEN/ISOCRCEN，RTXAT 在克隆片上可能写不进去
    const uint32_t back = readSfr(kRegC1Con);
    constexpr uint32_t kCheck = kConTxQEn | kConIsoCrcEn;
    if ((back & kCheck) != (con & kCheck)) {
      return false;
    }
  }

  // 6) 位时序；TDC 关闭（只对 BRS=1 生效，本片上曾导致 bus-off）
  initStep_ = 6;
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
    writeSfr(kRegC1Tdc, 0u);
  }

  initStep_ = 7;
  if (!configTxQueue()) {
    return false;
  }
  initStep_ = 8;
  if (!configRxFifo()) {
    return false;
  }

  // 9) C1INT 写 0 清可清标志 + IE 段开中断
  initStep_ = 9;
  irqEn_ = kIntRxIe | kIntRxOvIe | kIntCerIe | kIntSerIe;
  writeSfr(kRegC1Int, irqEn_);

  initStep_ = 10;
  if (!setMode(kModeNormalFd)) {
    return false;
  }
  initStep_ = 0xFF;
  return true;
}

bool Mcp2518fd::configTxQueue() {
  // 1 槽 8 字节 + TXAT=0：没人 ACK 时一帧立刻结束，不会把槽位占死
  const uint32_t con =
      (static_cast<uint32_t>(kPlSize8) << kFifoConPldSizeShift) |
      (static_cast<uint32_t>(kTxDepth - 1) << kFifoConFSizeShift) |
      kFifoConTxEn;
  writeSfr(kRegC1TxQCon, con);
  // 不写 CiTXQUA（只读）：TXQ 对象固定在 0x408 = 0x400 + 回读偏移
  return verifyFifoConfig(kRegC1TxQCon, con);
}

bool Mcp2518fd::configRxFifo() {
  const uint32_t con =
      (static_cast<uint32_t>(kPlSize8) << kFifoConPldSizeShift) |
      (static_cast<uint32_t>(kRxDepth - 1) << kFifoConFSizeShift) |
      kFifoConNotEmptyIe | kFifoConRxOvIe;
  const uint16_t addr = regFifoCon(kRxFifoIndex);
  writeSfr(addr, con);  // TXEN=0 就是接收 FIFO
  if (!verifyFifoConfig(addr, con)) {
    return false;
  }
  // 不写 CiFIFOUA1（只读）：对象 0 在 0x418，popRx 按 0x400 + 回读值定位
  rxIdx_ = 0;
  // 滤波器 0：对象/掩码全 0 = 全部接收，ID 分发留到应用层
  return configFilter(0, false, 0, 0);
}

bool Mcp2518fd::verifyFifoConfig(uint16_t addr, uint32_t expect) const {
  constexpr uint32_t kMask = (0x7u << kFifoConPldSizeShift) |
                             (0x1Fu << kFifoConFSizeShift) | kFifoConTxEn;
  return (readSfr(addr) & kMask) == (expect & kMask);
}

bool Mcp2518fd::configFilter(uint8_t n, bool ext, uint32_t id, uint32_t mask) {
  // 标准帧不能置 SID11/EXIDE：掩码位=1 表示必须匹配，置错会挡掉所有帧
  constexpr uint32_t eidMask = 0x3FFFFu;
  uint32_t obj = 0;
  uint32_t msk = 0;
  if (ext) {
    obj = ((id >> 18) & 0x7FFu) | ((id & eidMask) << 11) | kFltExIde;
    msk = (((mask >> 18) & 0x7FFu) | ((mask & eidMask) << 11)) | kFltExIde;
  } else {
    obj = id & 0x7FFu;
    msk = mask & 0x7FFu;
  }
  writeSfr(regFltObj(n), obj);
  writeSfr(regFltMask(n), msk);
  // FLTCON 每 4 个过滤器共用一个字：每字节 bit7=FLTENn、bit4:0=FIFO 号
  writeSfr(regFltCon(n),
           ((0x1u << 7) | (kRxFifoIndex & 0x1F)) << ((n % 4u) * 8u));
  return true;
}

// ---- 发送 ----
bool Mcp2518fd::txQueueFull() const {
  // 只看 TXREQ；别加 TXQSTA 的位（本片不可靠，会长期判忙把帧堵在队列里）
  const uint32_t con = readSfr(kRegC1TxQCon);
  if ((con & kFifoConTxReq) == 0u) {
    txqBusyStreak_ = 0;
    return false;
  }
  // 连续判忙 300 ms 强制放行一次，最多丢 1 帧
  if (++txqBusyStreak_ >= kTxqBusyLimit) {
    txqBusyStreak_ = 0;
    return false;
  }
  return true;
}

bool Mcp2518fd::pushTx(const FdFrame &frame) {
  if (frame.len > kMaxPayload) {
    return false;
  }
  const uint32_t fail_before = xferFail_;
  if (txQueueFull()) {
    return false;  // 交给上层下一轮重发，不在这里死等
  }

  uint32_t words[kObjWords] = {0};
  const uint32_t sid =
      frame.ext ? ((frame.id >> 18) & 0x7FF) : (frame.id & 0x7FF);
  const uint32_t eid = frame.ext ? (frame.id & 0x3FFFF) : 0;
  words[0] = sid | (eid << 11);
  words[1] = (static_cast<uint32_t>(lenToDlc(frame.len)) & kObjDlcMask) |
             (frame.ext ? kObjIde : 0) | (frame.rtr ? kObjRtr : 0) |
             (frame.brs ? kObjBrs : 0) | (frame.fdf ? kObjFdf : 0);
  std::memcpy(&words[2], frame.data, kMaxPayload);

  // ⚠️ 对象写在 0x408，不是 0x400（差 8 字节会让芯片把 D0/D1 当成 ID/DLC）
  writeRam(kTxRamBase, words, kObjWords);

  const uint32_t con = readSfr(kRegC1TxQCon);
  // 读失败写回 0 会把 TXEN/PLSIZE/FSIZE 清掉，宁可这帧不发
  if (xferFail_ != fail_before) {
    return false;
  }
  writeSfr(kRegC1TxQCon, con | kFifoConUInc | kFifoConTxReq);

  return xferFail_ == fail_before;
}

// ---- 接收 ----
bool Mcp2518fd::popRx(FdFrame &frame) {
  const uint32_t fail_before = xferFail_;
  uint32_t sta = readSfr(regFifoSta(kRxFifoIndex));

  // 溢出标志必须写 1 清，不清则该 FIFO 永远拒收
  if ((sta & kFifoStaRxOvIf) != 0u) {
    ++rxOvCount_;
    writeSfr(kRegC1RxOvIf, rxOvIfBit(kRxFifoIndex));
    sta = readSfr(regFifoSta(kRxFifoIndex));
  }

  // 唯一可靠的"有帧"判据是 bit0（FIFOCI 是消息索引，不是帧数）
  if ((sta & kFifoStaNotEmptyOrNotFull) == 0u) {
    return false;
  }

  // 槽位按 CiFIFOUA1 回读偏移定位（对象 0 -> 0x018），不合法才退回软件索引
  uint16_t ua;
  {
    const uint32_t uaRaw = readSfr(regFifoUa(kRxFifoIndex)) & 0x7FFu;
    const uint16_t cand = static_cast<uint16_t>(kRamBase + uaRaw);
    const uint16_t lastSlot =
        static_cast<uint16_t>(kRxRamBase + (kRxDepth - 1u) * kObjBytes);
    if ((cand >= kRxRamBase) && (cand <= lastSlot) && ((cand & 0x3u) == 0u)) {
      ua = cand;
      rxIdx_ = static_cast<uint8_t>((cand - kRxRamBase) / kObjBytes);
    } else {
      ua = static_cast<uint16_t>(kRxRamBase + rxIdx_ * kObjBytes);
    }
  }

  uint32_t words[kObjWords] = {0};
  readRam(ua, words, kObjWords);
  if (xferFail_ != fail_before) {
    return false;
  }
  rxIdx_ = static_cast<uint8_t>((rxIdx_ + 1u) % kRxDepth);

  const uint32_t r0 = words[0];
  const uint32_t r1 = words[1];
  frame.ext = (r1 & kObjIde) != 0;
  frame.rtr = (r1 & kObjRtr) != 0;
  frame.brs = (r1 & kObjBrs) != 0;
  frame.fdf = (r1 & kObjFdf) != 0;
  frame.len = dlcToLen(static_cast<uint8_t>(r1 & kObjDlcMask));
  if (frame.len > kMaxPayload) {
    frame.len = kMaxPayload;
  }

  const uint32_t sid = r0 & 0x7FF;
  const uint32_t eid = (r0 >> 11) & 0x3FFFF;
  frame.id = frame.ext ? ((sid << 18) | eid) : sid;

  std::memset(frame.data, 0, kFdMaxPayload);
  std::memcpy(frame.data, &words[2], kMaxPayload);

  const uint32_t con = readSfr(regFifoCon(kRxFifoIndex));
  if (xferFail_ != fail_before) {
    return false;  // 不 UINC 只是下轮重读同一帧，不会丢数据
  }
  writeSfr(regFifoCon(kRxFifoIndex), con | kFifoConUInc);

  return xferFail_ == fail_before;
}

// ---- 中断 / 状态 ----
uint32_t Mcp2518fd::readInterrupt() { return readSfr(kRegC1Int); }

void Mcp2518fd::clearInterrupt() { writeSfr(kRegC1Int, irqEn_); }

Mcp2518fd::Status Mcp2518fd::status() const {
  const uint32_t trec = readSfr(kRegC1Trec);
  Status s;
  s.tec = static_cast<uint8_t>((trec >> kTrecTecShift) & 0xFF);
  s.rec = static_cast<uint8_t>((trec >> kTrecRecShift) & 0xFF);
  s.busOff = (trec & kTrecTxBo) != 0;
  return s;
}

bool Mcp2518fd::busHealthy() const { return !status().busOff; }

// ---- 位时序 ----
bool Mcp2518fd::calcTiming(uint32_t sysclkHz, uint32_t bitrate,
                           uint8_t samplePointPct, BitTiming &out,
                           bool dataPhase) {
  if (sysclkHz == 0 || bitrate == 0 || samplePointPct == 0 ||
      samplePointPct > 100) {
    return false;
  }

  const uint32_t maxTseg1Len = dataPhase ? 32u : 256u;
  const uint32_t maxTseg2Len = dataPhase ? 16u : 128u;

  for (uint32_t brp = 1; brp <= 256; ++brp) {
    const uint32_t div = brp * bitrate;
    if ((sysclkHz % div) != 0) {
      continue;  // 除不尽就换下一个，否则实际波特率和设定值对不上
    }
    const uint32_t tqTotal = sysclkHz / div;
    if (tqTotal < 8 || tqTotal > 256) {
      continue;
    }

    const uint32_t want = (tqTotal * samplePointPct + 50) / 100;
    uint32_t tseg1Len = (want <= 1) ? 1 : (want - 1);
    if (tseg1Len > tqTotal - 2) {
      tseg1Len = tqTotal - 2;  // 至少给 PhaseSeg2 留 1 个 tq
    }
    const uint32_t tseg2Len = tqTotal - 1 - tseg1Len;
    if (tseg2Len < 1) {
      continue;
    }
    if (tseg1Len > maxTseg1Len || tseg2Len > maxTseg2Len) {
      continue;  // 装不进字段，换更大的 brp 再试
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
