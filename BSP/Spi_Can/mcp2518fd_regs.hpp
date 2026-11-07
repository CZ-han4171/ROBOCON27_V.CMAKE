/**
 * @file mcp2518fd_regs.hpp
 * @brief MCP2518FD 寄存器与内存布局定义
 *
 * @note : SFR 为 32 位宽，低字节在低地址（小端），下面直接按 32 位字访问
 */
#pragma once

#include <cstdint>

namespace BspCan {
namespace mcp2518fd {

// ---------------------------------------------------------------- SPI 指令
// 4 bit 命令 + 12 bit 地址，紧跟数据；nCS 低有效，SCK 空闲低（mode 0,0）
static constexpr uint8_t kSpiCmdReset = 0x0;      ///< A=0x000，复位并进配置模式
static constexpr uint8_t kSpiCmdRead = 0x3;
static constexpr uint8_t kSpiCmdWrite = 0x2;
static constexpr uint8_t kSpiCmdReadCrc = 0xB;
static constexpr uint8_t kSpiCmdWriteCrc = 0xA;
static constexpr uint8_t kSpiCmdWriteSafe = 0xC;

// ------------------------------------------------------------ MCP2518FD SFR
static constexpr uint16_t kRegOsc = 0xE00;      ///< 振荡器控制
static constexpr uint16_t kRegIoCon = 0xE04;    ///< 引脚复用 / 中断引脚配置
static constexpr uint16_t kRegCrc = 0xE08;
static constexpr uint16_t kRegEccCon = 0xE0C;
static constexpr uint16_t kRegEccStat = 0xE10;
static constexpr uint16_t kRegDevId = 0xE14;    ///< [3:0]=REV [7:4]=ID

// ----------------------------------------------------------- CAN 模块寄存器
static constexpr uint16_t kRegC1Con = 0x000;
static constexpr uint16_t kRegC1NBtCfg = 0x004;
static constexpr uint16_t kRegC1DBtCfg = 0x008;
static constexpr uint16_t kRegC1Tdc = 0x00C;
static constexpr uint16_t kRegC1Tbc = 0x010;
static constexpr uint16_t kRegC1TsCon = 0x014;
static constexpr uint16_t kRegC1Vec = 0x018;
static constexpr uint16_t kRegC1Int = 0x01C;
static constexpr uint16_t kRegC1RxIf = 0x020;
static constexpr uint16_t kRegC1TxIf = 0x024;
static constexpr uint16_t kRegC1RxOvIf = 0x028;
static constexpr uint16_t kRegC1TxAtIf = 0x02C;
static constexpr uint16_t kRegC1TxReq = 0x030;
static constexpr uint16_t kRegC1Trec = 0x034;
static constexpr uint16_t kRegC1BDiag0 = 0x038;
static constexpr uint16_t kRegC1BDiag1 = 0x03C;

static constexpr uint16_t kRegC1TxQCon = 0x050;
static constexpr uint16_t kRegC1TxQSta = 0x054;
static constexpr uint16_t kRegC1TxQUa = 0x058;

// FIFO m 从 1 开始，每 3 个 32 位字一组，步长 0x0C
inline uint16_t regFifoCon(uint8_t m) { return 0x05C + (m - 1) * 0x0C; }
inline uint16_t regFifoSta(uint8_t m) { return 0x060 + (m - 1) * 0x0C; }
inline uint16_t regFifoUa(uint8_t m) { return 0x064 + (m - 1) * 0x0C; }

// 过滤器：FLTCONn 每 4 个过滤器一组，FLTOBJn/MASKn 每对 8 字节
inline uint16_t regFltCon(uint8_t n) { return 0x1D0 + n * 0x04; }
inline uint16_t regFltObj(uint8_t n) { return 0x1F0 + n * 0x08; }
inline uint16_t regFltMask(uint8_t n) { return 0x1F4 + n * 0x08; }

// ------------------------------------------------------------- C1CON 位域
static constexpr uint32_t kConReqOpMask = 0x7u << 24;   ///< REQOP[2:0]
static constexpr uint32_t kConOpModMask = 0x7u << 21;   ///< OPMOD[2:0]
static constexpr uint32_t kConTxQEn = 0x1u << 20;
static constexpr uint32_t kConStEf = 0x1u << 19;
static constexpr uint32_t kConBrsDis = 0x1u << 12;
static constexpr uint32_t kConIsoCrcEn = 0x1u << 5;     ///< 1=ISO, 0=non-ISO

// REQOP / OPMOD 取值
static constexpr uint32_t kModeNormalFd = 0x0;
static constexpr uint32_t kModeSleep = 0x1;
static constexpr uint32_t kModeInternalLoopback = 0x2;
static constexpr uint32_t kModeListenOnly = 0x3;
static constexpr uint32_t kModeConfig = 0x4;
static constexpr uint32_t kModeExternalLoopback = 0x5;
static constexpr uint32_t kModeNormal20 = 0x6;
static constexpr uint32_t kModeRestricted = 0x7;

// -------------------------------------------------------- C1INT / 中断位
static constexpr uint32_t kIntRxIe = 0x1u << 17;
static constexpr uint32_t kIntTxIe = 0x1u << 16;
static constexpr uint32_t kIntRxOvIe = 0x1u << 27;
static constexpr uint32_t kIntSerIe = 0x1u << 28;
static constexpr uint32_t kIntCerIe = 0x1u << 29;
static constexpr uint32_t kIntModIe = 0x1u << 19;

static constexpr uint32_t kIntRxIf = 0x1u << 1;
static constexpr uint32_t kIntTxIf = 0x1u << 0;
static constexpr uint32_t kIntCerIf = 0x1u << 13;
static constexpr uint32_t kIntSerIf = 0x1u << 12;
static constexpr uint32_t kIntRxOvIf = 0x1u << 11;
static constexpr uint32_t kIntModIf = 0x1u << 3;
static constexpr uint32_t kIntAll = 0xFFFFu;  ///< 写 1 清零

// ---------------------------------------------------- 报文 RAM 与 FIFO 控制
static constexpr uint16_t kRamBase = 0x400;   ///< 消息 RAM 起始，到 0xBFF（2KB）
static constexpr uint16_t kRamEnd = 0xBFF;

// FIFOCON / TXQCON 公共位
static constexpr uint32_t kFifoConPldSizeShift = 29;  ///< PLSIZE[2:0]
static constexpr uint32_t kFifoConFSizeShift = 24;    ///< FSIZE[4:0] = 深度-1
static constexpr uint32_t kFifoConTxPriShift = 14;    ///< TXPRI[4:0]
static constexpr uint32_t kFifoConTxAtShift = 19;     ///< TXAT[1:0]
static constexpr uint32_t kFifoConFrEsEt = 0x1u << 10;
static constexpr uint32_t kFifoConTxReq = 0x1u << 9;
static constexpr uint32_t kFifoConUInc = 0x1u << 8;
static constexpr uint32_t kFifoConTxEn = 0x1u << 7;
static constexpr uint32_t kFifoConRxTsEn = 0x1u << 5;
static constexpr uint32_t kFifoConTxAtIe = 0x1u << 4;
static constexpr uint32_t kFifoConRxOvIe = 0x1u << 3;
static constexpr uint32_t kFifoConEmptyFullIe = 0x1u << 2;
static constexpr uint32_t kFifoConHalfIe = 0x1u << 1;
static constexpr uint32_t kFifoConNotEmptyIe = 0x1u << 0;

// FIFOSTA 位：TXQ 用 bit0 表示"队列未满"
static constexpr uint32_t kFifoStaNotEmptyIf = 0x1u << 0;
static constexpr uint32_t kTxqStaNotFull = 0x1u << 0;

// ------------------------------------------------------------- TREC 状态位
static constexpr uint32_t kTrecEwarn = 0x1u << 16;  // 错误警告（TEC 或 REC ≥ 96）
static constexpr uint32_t kTrecRxWarn = 0x1u << 17;
static constexpr uint32_t kTrecTxWarn = 0x1u << 18;
static constexpr uint32_t kTrecRxBp = 0x1u << 19;   // 接收 error-passive
static constexpr uint32_t kTrecTxBp = 0x1u << 20;   // 发送 error-passive
static constexpr uint32_t kTrecTxBo = 0x1u << 21;   // bus-off

// PLSIZE 编码（表 3-5 注 1）
static constexpr uint8_t kPlSize8 = 0;
static constexpr uint8_t kPlSize12 = 1;
static constexpr uint8_t kPlSize16 = 2;
static constexpr uint8_t kPlSize20 = 3;
static constexpr uint8_t kPlSize24 = 4;
static constexpr uint8_t kPlSize32 = 5;
static constexpr uint8_t kPlSize48 = 6;
static constexpr uint8_t kPlSize64 = 7;

// 报文对象 T1/R1 位（表 3-5 / 3-6）
static constexpr uint32_t kObjEsi = 0x1u << 8;
static constexpr uint32_t kObjFdf = 0x1u << 7;
static constexpr uint32_t kObjBrs = 0x1u << 6;
static constexpr uint32_t kObjRtr = 0x1u << 5;
static constexpr uint32_t kObjIde = 0x1u << 4;
static constexpr uint32_t kObjDlcMask = 0x0Fu;

// 过滤器对象位（FLTOBJ0）
static constexpr uint32_t kFltExIde = 0x1u << 30;
static constexpr uint32_t kFltSid11 = 0x1u << 29;

// --------------------------------------------------------- OSC 寄存器位域
// bit7:0 = [7]- CLKODIV[1:0] SCLKDIV LPMEN OSCDIS [1]- PLLEN
static constexpr uint32_t kOscPllEn = 0x1u << 0;      ///< 1 = SYSCLK 取自 10x PLL（4MHz 晶振时用）
static constexpr uint32_t kOscOscDis = 0x1u << 2;     ///< 1 = 停振进 Sleep
static constexpr uint32_t kOscLpmEn = 0x1u << 3;
static constexpr uint32_t kOscSclkDiv = 0x1u << 4;    ///< 1 = SYSCLK 二分频
static constexpr uint32_t kOscClkoDivShift = 5;       ///< CLKODIV[1:0]
static constexpr uint32_t kOscPllReady = 0x1u << 8;
static constexpr uint32_t kOscOscReady = 0x1u << 10;
static constexpr uint32_t kOscSclkReady = 0x1u << 12;

} // namespace mcp2518fd
} // namespace BspCan
