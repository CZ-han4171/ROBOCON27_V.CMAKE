/**
 * @file mcp2518fd_regs.hpp
 * @brief MCP2518FD 寄存器与内存布局（只保留驱动用到的部分）
 */
#pragma once

#include <cstdint>

namespace BspCan {
namespace mcp2518fd {

// SPI 指令：4 bit 命令 + 12 bit 地址
static constexpr uint8_t kSpiCmdReset = 0x0;
static constexpr uint8_t kSpiCmdRead = 0x3;
static constexpr uint8_t kSpiCmdWrite = 0x2;

static constexpr uint16_t kRegOsc = 0xE00;
static constexpr uint16_t kRegDevId = 0xE14;  // 本片恒回 0

static constexpr uint16_t kRegC1Con = 0x000;
static constexpr uint16_t kRegC1NBtCfg = 0x004;
static constexpr uint16_t kRegC1DBtCfg = 0x008;
static constexpr uint16_t kRegC1Tdc = 0x00C;
static constexpr uint16_t kRegC1Int = 0x01C;
static constexpr uint16_t kRegC1RxOvIf = 0x028;
static constexpr uint16_t kRegC1Trec = 0x034;
static constexpr uint16_t kRegC1BDiag1 = 0x03C;
static constexpr uint16_t kRegC1TxQCon = 0x050;

// FIFO m 从 1 开始，每 3 个字一组，步长 0x0C
inline uint16_t regFifoCon(uint8_t m) { return 0x05C + (m - 1) * 0x0C; }
inline uint16_t regFifoSta(uint8_t m) { return 0x060 + (m - 1) * 0x0C; }
inline uint16_t regFifoUa(uint8_t m) { return 0x064 + (m - 1) * 0x0C; }

// 过滤器：FLTCON 每 4 个共用一个字，OBJ/MASK 每对 8 字节
inline uint16_t regFltCon(uint8_t n) { return 0x1D0 + (n / 4) * 0x04; }
inline uint16_t regFltObj(uint8_t n) { return 0x1F0 + n * 0x08; }
inline uint16_t regFltMask(uint8_t n) { return 0x1F4 + n * 0x08; }

// C1CON
static constexpr uint32_t kConReqOpMask = 0x7u << 24;
static constexpr uint32_t kConOpModMask = 0x7u << 21;
static constexpr uint32_t kConTxQEn = 0x1u << 20;
static constexpr uint32_t kConRtxAt = 0x1u << 16;    // 1 = 让 TXQCON.TXAT 生效
static constexpr uint32_t kConIsoCrcEn = 0x1u << 5;  // 1 = ISO, 0 = non-ISO

static constexpr uint32_t kModeNormalFd = 0x0;
static constexpr uint32_t kModeInternalLoopback = 0x2;
static constexpr uint32_t kModeConfig = 0x4;

// C1INT
static constexpr uint32_t kIntRxIe = 0x1u << 17;
static constexpr uint32_t kIntRxOvIe = 0x1u << 27;
static constexpr uint32_t kIntSerIe = 0x1u << 28;
static constexpr uint32_t kIntCerIe = 0x1u << 29;

// 报文 RAM：0x400 ~ 0xBFF（2 KB）
static constexpr uint16_t kRamBase = 0x400;
static constexpr uint16_t kRamEnd = 0xBFF;

static constexpr uint32_t kFifoConPldSizeShift = 29;  // PLSIZE[2:0]
static constexpr uint32_t kFifoConFSizeShift = 24;    // FSIZE[4:0] = 深度 - 1
static constexpr uint32_t kFifoConTxReq = 0x1u << 9;
static constexpr uint32_t kFifoConUInc = 0x1u << 8;
static constexpr uint32_t kFifoConTxEn = 0x1u << 7;
static constexpr uint32_t kFifoConRxOvIe = 0x1u << 3;
static constexpr uint32_t kFifoConNotEmptyIe = 0x1u << 0;

// FIFOSTA bit0 是唯一可靠的"有帧/有空位"判据；bits12:8 是消息索引，不是帧数
static constexpr uint32_t kFifoStaNotEmptyOrNotFull = 0x1u << 0;
static constexpr uint32_t kFifoStaRxOvIf = 0x1u << 3;

// C1RXOVIF：写 1 清，不清则该 FIFO 永远拒收新帧
inline uint32_t rxOvIfBit(uint8_t fifo) { return 0x1u << fifo; }

// C1BDIAG1：低 16 位无错报文计数，高 16 位错误标志（写 1 清）
static constexpr uint32_t kBDiag1EfMsgCntMask = 0xFFFFu;
static constexpr uint32_t kBDiag1DlcMismatch = 0x1u << 31;
static constexpr uint32_t kBDiag1DCrcErr = 0x1u << 29;
static constexpr uint32_t kBDiag1DBit0Err = 0x1u << 24;
static constexpr uint32_t kBDiag1TxBoErr = 0x1u << 23;
static constexpr uint32_t kBDiag1NFormErr = 0x1u << 19;
static constexpr uint32_t kBDiag1NAckErr = 0x1u << 18;   // 没人 ACK
static constexpr uint32_t kBDiag1NBit1Err = 0x1u << 17;
static constexpr uint32_t kBDiag1NBit0Err = 0x1u << 16;  // 收发器没驱动总线
static constexpr uint32_t kBDiag1ErrFlagMask = 0xFFFF0000u;

// C1TREC：TEC 在高字节，REC 在低字节，bit21 = bus-off
static constexpr uint32_t kTrecTecShift = 8;
static constexpr uint32_t kTrecRecShift = 0;
static constexpr uint32_t kTrecTxBo = 0x1u << 21;

static constexpr uint8_t kPlSize8 = 0;  // 只有 8 字节载荷在用

// 报文对象 T1/R1
static constexpr uint32_t kObjFdf = 0x1u << 7;
static constexpr uint32_t kObjBrs = 0x1u << 6;
static constexpr uint32_t kObjRtr = 0x1u << 5;
static constexpr uint32_t kObjIde = 0x1u << 4;
static constexpr uint32_t kObjDlcMask = 0x0Fu;

// 过滤器：bit31=SID11、bit30=EXIDE、bit29:11=EID[17:0]、bit10:0=SID[10:0]
// ⚠ 标准帧这两位都不能置：掩码位=1 表示必须匹配，置错会挡掉所有帧
static constexpr uint32_t kFltExIde = 0x1u << 30;

// OSC
static constexpr uint32_t kOscPllEn = 0x1u << 0;
static constexpr uint32_t kOscOscDis = 0x1u << 2;
static constexpr uint32_t kOscLpmEn = 0x1u << 3;
static constexpr uint32_t kOscSclkDiv = 0x1u << 4;
static constexpr uint32_t kOscPllReady = 0x1u << 8;
static constexpr uint32_t kOscOscReady = 0x1u << 10;

} // namespace mcp2518fd
} // namespace BspCan
