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

// 芯片给出的 FIFO 用户地址（CiTXQUA / CiFIFOUAn）是 0x400 基址的绝对地址，
// 报文 RAM 占 0x400~0xBFF，不是从 0 开始的偏移，所以直接拿它当 SPI 访问地址
inline bool uaValid(uint16_t ua) {
  return static_cast<uint32_t>(ua) >= kRamBase &&
         static_cast<uint32_t>(ua) <= kRamEnd;
}

} // namespace

// ---------------------------------------------------------------- SPI 事务
// 所有 SPI 都从这里走，失败会计数，上层能查 spiHealthy()，不会"静默失败"
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

// ---------------------------------------------------------------- SFR 访问
uint32_t Mcp2518fd::readSfr(uint16_t addr) const {
  uint8_t tx[6] = {0};
  uint8_t rx[6] = {0};
  tx[0] = static_cast<uint8_t>((kSpiCmdRead << 4) | ((addr >> 8) & 0x0F));
  tx[1] = static_cast<uint8_t>(addr & 0xFF);
  if (!xfer(tx, rx, sizeof(tx))) {
    return 0;  // 读失败返回 0：等标志的循环会因为等不到而超时退出
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

// ---------------------------------------------------------------- 模式控制
bool Mcp2518fd::waitFlag(uint16_t addr, uint32_t mask) const {
  const uint32_t fail_before = xferFail_;
  for (uint32_t i = 0; i < cfg_.modeTimeoutLoops; ++i) {
    if ((readSfr(addr) & mask) != 0) {
      return true;
    }
    // SPI 已经出错就别再空转：每轮都要白等满 kSpiTimeoutMs
    if (xferFail_ != fail_before) {
      return false;
    }
  }
  return false;
}

bool Mcp2518fd::setMode(uint32_t reqOp) {
  const uint32_t fail_before = xferFail_;
  uint32_t con = readSfr(kRegC1Con);
  // 读失败 con 是 0，写回去会把 TXQEN/ISOCRCEN 等配置位全清掉
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
    // 等模式切换期间 SPI 出错：再转满 modeTimeoutLoops 轮就是白等几十秒
    if (xferFail_ != fail_before) {
      return false;
    }
  }
  return false;
}

// ------------------------------------------------------------------ 初始化
bool Mcp2518fd::readDevId(uint8_t &id, uint8_t &rev) {
  const uint32_t v = readSfr(kRegDevId);
  id = static_cast<uint8_t>((v >> 4) & 0xFF);
  rev = static_cast<uint8_t>(v & 0x0F);
  // 全 0 或全 F 基本都是 SPI 没通：MISO 悬空、片选不对、SCK 没出来
  return (v != 0x00000000u) && (v != 0xFFFFFFFFu);
}

bool Mcp2518fd::init(const Config &cfg) {
  cfg_ = cfg;
  irqEn_ = 0;
  // 每轮初始化重新计数：上一次的 SPI 故障不该把这次直接判死
  xferFail_ = 0;

  // 1) SPI 复位指令：芯片进配置模式，FIFO 指针和报文 RAM 分配一起回到上电值。
  //    不用 CiTXQCON / CiFIFOCON 的 FRESET 位 —— 官方 Linux 驱动同样只发这条指令，
  //    免得依赖"FRESET 到底会不会自己清零"这种没把握的行为
  {
    uint8_t tx[2] = {static_cast<uint8_t>(kSpiCmdReset << 4), 0x00};
    uint8_t rx[2] = {0};
    if (!xfer(tx, rx, sizeof(tx))) {
      return false;
    }
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

    // 读回校验：配置位写不进去，说明没停在配置模式或者 SPI 只有写方向通
    const uint32_t back = readSfr(kRegC1Con);
    constexpr uint32_t kCheck = kConTxQEn | kConIsoCrcEn;
    if ((back & kCheck) != (con & kCheck)) {
      return false;
    }
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
  // TDC：Auto 模式让芯片自己测量收发器环路延迟。TDCO 填手册上限 63 作补偿
  // 上限、TDCV 留 0 作最小值，这是 Linux mcp251xfd 驱动的推荐配置——手册给的
  // [-64,63] 是相对值语义，实测填相对值反而会导致 bus-off。经典帧不走数据段，
  // TDC 对它无影响，只有 BRS=1 的帧才生效。
  writeSfr(kRegC1Tdc, (kTdcModAuto << kTdcModShift) | (kTdcCoMax << kTdcCoShift));

  if (!configTxQueue() || !configRxFifo()) {
    return false;
  }

  // C1INT 是"写 0 清零"：IF 段写 0 把挂起的可清标志清掉，
  // IE 段写 irqEn_ 打开需要的中断。RXIF/TXIF/RXOVIF 是只读汇总位，写不写都没用。
  irqEn_ = kIntRxIe | kIntRxOvIe | kIntCerIe | kIntSerIe;
  writeSfr(kRegC1Int, irqEn_);

  if (!setMode(kModeNormalFd)) {
    return false;
  }
  return true;
}

bool Mcp2518fd::configTxQueue() {
  // PLSIZE=64B，FSIZE=深度-1，TXAT=3（不限重发次数），优先级取中间值
  const uint32_t con =
      (static_cast<uint32_t>(kPlSize64) << kFifoConPldSizeShift) |
      (static_cast<uint32_t>(kTxDepth - 1) << kFifoConFSizeShift) |
      (static_cast<uint32_t>(kFifoConTxAtUnlimited) << kFifoConTxAtShift) |
      (0x10u << kFifoConTxPriShift) | kFifoConTxEn;
  writeSfr(kRegC1TxQCon, con);
  if (!verifyFifoConfig(kRegC1TxQCon, con)) {
    return false;
  }

  // 芯片按 FSIZE 重新分配报文 RAM 后会把地址写进 CiTXQUA，
  // 读回来必须落在 0x400~0xBFF，否则后面按 UA 写 RAM 全写到别处去了
  return uaValid(static_cast<uint16_t>(readSfr(kRegC1TxQUa) & 0xFFFFu));
}

bool Mcp2518fd::configRxFifo() {
  const uint32_t con = (static_cast<uint32_t>(kPlSize64) << kFifoConPldSizeShift) |
                       (static_cast<uint32_t>(kRxDepth - 1) << kFifoConFSizeShift) |
                       kFifoConNotEmptyIe | kFifoConRxOvIe;
  const uint16_t addr = regFifoCon(kRxFifoIndex);
  // TXEN = 0 就是接收 FIFO
  writeSfr(addr, con);
  if (!verifyFifoConfig(addr, con)) {
    return false;
  }
  if (!uaValid(static_cast<uint16_t>(readSfr(regFifoUa(kRxFifoIndex)) & 0xFFFFu))) {
    return false;
  }

  // 过滤器 0 掩码全 0 = 全部接收，具体 ID 分发留到 com_config 层做
  return configFilter(0, false, 0, 0);
}

// 配置完读回 FIFOCON。PLSIZE/FSIZE/TXEN 只在配置模式下可改，
// 读回来和写下去不一致，就说明芯片没停在配置模式或者 SPI 有问题
bool Mcp2518fd::verifyFifoConfig(uint16_t addr, uint32_t expect) const {
  constexpr uint32_t kMask = (0x7u << kFifoConPldSizeShift) |
                             (0x1Fu << kFifoConFSizeShift) | kFifoConTxEn;
  return (readSfr(addr) & kMask) == (expect & kMask);
}

bool Mcp2518fd::configFilter(uint8_t n, bool ext, uint32_t id, uint32_t mask) {
  uint32_t obj = id & (0x7FFu | (0x3FFFFu << 11));
  uint32_t msk = mask & (0x7FFu | (0x3FFFFu << 11));
  if (ext) {
    obj |= kFltExIde;
    msk |= kFltExIde;
  } else {
    obj |= kFltSid11;
  }
  writeSfr(regFltObj(n), obj);
  writeSfr(regFltMask(n), msk);
  // FLTCONn：每 4 个过滤器共用一个 32 位寄存器，每个过滤器占 1 个字节
  // （FLT0 在 [7:0]、FLT1 在 [15:8]……），所以按 n%4 左移，不能都写低字节
  // bit7 = FLTENn，bit4:0 = 目标 FIFO 编号
  writeSfr(regFltCon(n),
           ((0x1u << 7) | (kRxFifoIndex & 0x1F)) << ((n % 4u) * 8u));
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
  const uint32_t fail_before = xferFail_;
  // 队列满就直接放弃，交给上层下一轮重发，不要在这里死等
  if (txQueueFull()) {
    return false;
  }

  const uint16_t ua =
      static_cast<uint16_t>(readSfr(kRegC1TxQUa) & 0xFFFFu);
  if (!uaValid(ua)) {
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

  // UINC 推进 head、TXREQ 交给芯片自己发。两个都是自清零的控制位，
  // 带上读回来的配置位一起写，省一次 SPI 事务也不会破坏 FIFO 配置
  const uint32_t con = readSfr(kRegC1TxQCon);
  // 读失败时 readSfr 返回 0，写回 0|UINC|TXREQ 会把 TXEN/PLSIZE/FSIZE 全清掉，
  // TXQ 直接被禁用。宁可这一帧不发，也不能写坏 FIFO 配置
  if (xferFail_ != fail_before) {
    return false;
  }
  writeSfr(kRegC1TxQCon, con | kFifoConUInc | kFifoConTxReq);

  // 中途只要有一次 SPI 失败，这帧可能根本没进芯片，必须让上层知道（计入 txFail）
  return xferFail_ == fail_before;
}

bool Mcp2518fd::popRx(FdFrame &frame) {
  const uint32_t fail_before = xferFail_;
  const uint32_t sta = readSfr(regFifoSta(kRxFifoIndex));
  if ((sta & kFifoStaNotEmptyIf) == 0) {
    return false;
  }

  const uint16_t ua =
      static_cast<uint16_t>(readSfr(regFifoUa(kRxFifoIndex)) & 0xFFFFu);
  if (!uaValid(ua)) {
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
  // 同上：读失败写回 0 会把 RX FIFO 的配置清掉。不 UINC 只是下一轮重读同一帧，
  // 不会丢数据，比清掉配置安全
  if (xferFail_ != fail_before) {
    return false;
  }
  writeSfr(regFifoCon(kRxFifoIndex), con | kFifoConUInc);

  // 读 RAM 时就失败了，帧内容不可信，返回 false 让上层丢掉
  return xferFail_ == fail_before;
}

// ---------------------------------------------------------------- 中断与状态
uint32_t Mcp2518fd::readInterrupt() { return readSfr(kRegC1Int); }

// C1INT 的标志位是"写 0 清零"，而且只有 kIntIfClearable 里那 5 个能软件清。
// IF 段全写 0 就是把所有可清标志清掉，IE 段写回 irqEn_ 保住中断使能。
void Mcp2518fd::clearInterrupt() { writeSfr(kRegC1Int, irqEn_); }

uint32_t Mcp2518fd::readFifoStatus(uint8_t fifo) const {
  return readSfr(regFifoSta(fifo));
}

Mcp2518fd::Status Mcp2518fd::status() const {
  const uint32_t trec = readSfr(kRegC1Trec);
  Status s;
  // C1TREC: TEC 在高字节 [15:8]，REC 在低字节 [7:0]
  s.tec = static_cast<uint8_t>((trec >> kTrecTecShift) & 0xFF);
  s.rec = static_cast<uint8_t>((trec >> kTrecRecShift) & 0xFF);
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
// 仲裁段 TSEG1 是 8 位、TSEG2 是 7 位，数据段分别只有 5 位和 4 位。
// 数据段 tq 总数一大（低波特率）就会超出字段宽度，写进芯片被静默截断，
// 实际波特率和采样点全变，比配置失败更难查，所以这里按段判上限。
bool Mcp2518fd::calcTiming(uint32_t sysclkHz, uint32_t bitrate,
                           uint8_t samplePointPct, BitTiming &out,
                           bool dataPhase) {
  if (sysclkHz == 0 || bitrate == 0 || samplePointPct == 0 ||
      samplePointPct > 100) {
    return false;
  }

  // 字段存的是"长度-1"，所以字段宽度 +1 就是长度上限
  // DBTCFG[20:16] = 5 位 -> 32；NBTCFG[23:16] = 8 位 -> 256
  const uint32_t maxTseg1Len = dataPhase ? 32u : 256u;
  // DBTCFG[11:8] = 4 位 -> 16；NBTCFG[14:8] = 7 位 -> 128
  const uint32_t maxTseg2Len = dataPhase ? 16u : 128u;

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
    // 这个分频下装不进寄存器字段，换更大的 brp（tq 总数变小）再试
    if (tseg1Len > maxTseg1Len || tseg2Len > maxTseg2Len) {
      continue;
    }

    out.brp = static_cast<uint8_t>(brp - 1);
    out.tseg1 = static_cast<uint8_t>(tseg1Len - 1);
    out.tseg2 = static_cast<uint8_t>(tseg2Len - 1);
    // SJW 最多取 4 个 tq，天然装得下两段的 4 位下限字段，不用另判
    out.sjw = static_cast<uint8_t>(min32(tseg2Len, 4) - 1);
    return true;
  }
  return false;
}

} // namespace BspCan

