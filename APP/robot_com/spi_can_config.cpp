/**
 * @file spi_can_config.cpp
 * @brief 第 4 路 CAN（MCP2518FD + SPI4）：板级配置 + 收发任务
 *
 * 与片上 FDCAN 完全隔离：只复用 CanDevice 基类（matchRx / onRx），
 * 总线、队列、任务、统计都是 CAN4 自己的
 */
#include "spi_can_config.hpp"

#include "FreeRTOS.h"
#include "task.h"

#include "main.h"
#include "spi.h"

#include <cstring>

namespace SpiCanApp {
namespace {

// 接线：SCK PE12 / MOSI PE14 / MISO PE13（AF5）、nCS PE11、INT# PE10；晶振 20 MHz
const BspCan::SpiCanPins kPins = {
    CS_CAN_GPIO_Port, CS_CAN_Pin, INT_CAN_GPIO_Port, INT_CAN_Pin, nullptr, 0,
};

constexpr uint32_t kTaskPeriodMs = 5;  // 空闲时 5 ms 醒一次
constexpr uint32_t kMaxRxPerCycle = BspCan::Mcp2518fd::kRxDepth;
constexpr uint32_t kFlagInt = 0x01u;  // INT# 拉低
constexpr uint32_t kFlagTx = 0x02u;   // 有新帧进发送队列
constexpr uint32_t kFlagWake = kFlagInt | kFlagTx;
constexpr uint32_t kRetryDelayMs = 1000;

// RX 假死自愈：tec/rec 都为 0（线是通的）却长时间收不到帧，整片重建一次
constexpr uint32_t kRxWatchdogSilentMs = 1500;
constexpr uint32_t kRxWatchdogRetryMs = 2000;

// 20 MHz 晶振、1 Mbps、采样点 80% -> NBTCFG = 0x000E0303
BspCan::SpiCanTransport::Options makeOptions() {
  BspCan::SpiCanTransport::Options o;
  o.sysclkHz = 20000000;
  o.usePll = false;
  o.nominalBitrate = 1000000;
  o.dataBitrate = 1000000;
  o.nominalSamplePoint = 80;
  o.dataSamplePoint = 80;
  o.isoCanFd = true;
  return o;
}

BspCan::SpiCanTransport g_transport(hspi4, kPins, makeOptions());

osThreadId_t g_taskId = nullptr;

void notifyTxPending() {
  if (g_taskId != nullptr && osThreadGetId() != g_taskId) {
    (void)osThreadFlagsSet(g_taskId, kFlagTx);
  }
}

// EXTI 上下文：只置标志、唤醒任务，中断里不碰 SPI
void onExti() {
  g_transport.onInterrupt();
  if (g_taskId != nullptr) {
    osThreadFlagsSet(g_taskId, kFlagInt);
  }
}

} // namespace

// 初始化进度 / 自愈计数（调试器看这几个）
volatile uint8_t g_initStep = 0;
volatile uint32_t g_rxRecover = 0;
volatile uint32_t g_rxSilentMs = 0;

Stats g_stats;

Stats snapshot() { return g_stats; }

Can4Bus g_can4Bus;

// ---- Can4Bus：设备表 + 发送队列 ----
uint8_t Can4Bus::registerDevice(CanDevice *device) {
  if (device == nullptr) {
    return 1;
  }
  for (uint8_t i = 0; i < kMaxDevices; ++i) {
    if (device_[i] == device) {
      return 0;  // 重复注册当成功
    }
  }
  for (uint8_t i = 0; i < kMaxDevices; ++i) {
    if (device_[i] == nullptr) {
      device_[i] = device;
      return 0;
    }
  }
  return 2;  // 设备表满
}

bool Can4Bus::addCanMsg(const Pack &pack) {
  if (static_cast<size_t>(pack.len) > sizeof(pack.data)) {
    return false;
  }
  if (txQueue_.TryPush(pack) != Algorithm::QueueError::OK) {
    return false;
  }
  if (tx_notify_ != nullptr) {
    tx_notify_();
  }
  return true;
}

bool Can4Bus::addCanMsg(uint32_t id, const uint8_t data[8], uint8_t len,
                        bool ext, bool fd, bool brs) {
  if (data == nullptr || len > 8) {
    return false;
  }
  Pack pack;
  pack.id = id;
  pack.len = len;
  pack.ext = ext;
  pack.fd = fd;
  pack.brs = fd && brs;
  std::memcpy(pack.data, data, len);
  return addCanMsg(pack);
}

void Can4Bus::dispatch(const BspCan::FdFrame &frame) {
  const uint8_t len = (frame.len > 8) ? 8 : frame.len;  // 上层接口固定 8 字节
  g_stats.lastRxId = frame.id;

  if (frame.rtr) {  // 远程帧没有数据段
    return;
  }

  FDCAN_RxHeaderTypeDef rx_header = {0};
  rx_header.Identifier = frame.id;
  rx_header.IdType = frame.ext ? FDCAN_EXTENDED_ID : FDCAN_STANDARD_ID;
  rx_header.RxFrameType = FDCAN_DATA_FRAME;
  rx_header.DataLength = BspCan::lenToDlc(len);
  rx_header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
  rx_header.BitRateSwitch = frame.brs ? FDCAN_BRS_ON : FDCAN_BRS_OFF;
  rx_header.FDFormat = frame.fdf ? FDCAN_FD_CAN : FDCAN_CLASSIC_CAN;

  for (uint8_t i = 0; i < kMaxDevices; ++i) {
    if (device_[i] != nullptr && device_[i]->matchRx(rx_header)) {
      device_[i]->onRx(frame.data, len);
    }
  }
}

void Can4Bus::txService() {
  Pack pack;
  while (!g_transport.chip().txQueueFull() &&
         txQueue_.TryPop(pack) == Algorithm::QueueError::OK) {
    BspCan::FdFrame frame;
    frame.id = pack.id;
    frame.len = pack.len;
    frame.ext = pack.ext;
    frame.fdf = pack.fd;
    frame.brs = pack.brs;
    std::memcpy(frame.data, pack.data, pack.len);
    if (!g_transport.send(frame)) {
      ++g_stats.txLost;
    } else {
      ++g_stats.txFrames;
    }
  }
}

// ---- 对外发送接口 ----
bool sendFrame(uint32_t id, const uint8_t *data, uint8_t len, bool ext,
               bool fd) {
  // 片选和芯片内部状态不允许并发访问：这个口只给 can4SendTask
  if (osThreadGetId() != g_taskId) {
    return false;
  }
  if (data == nullptr || len > BspCan::Mcp2518fd::kMaxPayload) {
    return false;
  }
  BspCan::FdFrame f;
  f.id = id;
  f.len = len;
  f.ext = ext;
  f.fdf = fd;
  f.brs = fd;
  std::memcpy(f.data, data, len);
  return g_transport.send(f);
}

uint8_t can4Attach(CanDevice *dev) { return g_can4Bus.registerDevice(dev); }

bool can4Send(uint32_t id, const uint8_t *data, uint8_t len, bool ext, bool fd) {
  if (data == nullptr) {
    return false;
  }
  if (osThreadGetId() == g_taskId) {  // 本任务直发，省掉入队 + 唤醒
    return sendFrame(id, data, len, ext, fd);
  }
  if (len > sizeof(Can4Bus::Pack::data)) {
    return false;  // 跨任务只能发 8 字节
  }
  Can4Bus::Pack pack;
  pack.id = id;
  pack.len = len;
  pack.ext = ext;
  pack.fd = fd;
  pack.brs = fd;
  std::memcpy(pack.data, data, len);
  return g_can4Bus.addCanMsg(pack);
}

} // namespace SpiCanApp

extern "C" void can4SendTask(void *argument) {
  (void)argument;
  using namespace SpiCanApp;

  g_taskId = osThreadGetId();
  g_can4Bus.setTxNotify(&notifyTxPending);

  for (;;) {
    // 5 ms 超时，或被 INT# / 新帧立即唤醒
    (void)osThreadFlagsWait(kFlagWake, osFlagsWaitAny, kTaskPeriodMs);
    g_stats.irqCount = g_transport.irqCount();
    g_stats.rxLost = g_transport.chip().rxOverflowCount();

    // ---------------- 初始化 / 失败重试 ----------------
    if (g_stats.phase != Phase::Running) {
      uint8_t id = 0;
      uint8_t rev = 0;
      if (!g_transport.chip().readDevId(id, rev)) {
        // 本片 DEVID 恒回 0：先发复位指令，再用 SFR + RAM 往返判活
        (void)g_transport.chip().spiReset();
        osDelay(5);
        if (!g_transport.chip().linkSelfTest()) {
          g_stats.phase = Phase::SpiDead;
          g_stats.spiFail = g_transport.chip().spiFailCount();
          osDelay(kRetryDelayMs);
          continue;
        }
      }
      g_stats.phase = g_transport.start() ? Phase::Running : Phase::ConfigFail;
      g_initStep = g_transport.chip().initStep();
      if (g_stats.phase != Phase::Running) {
        osDelay(kRetryDelayMs);
      }
      continue;
    }

    // ---------------- 收帧 ----------------
    BspCan::FdFrame frame;
    for (uint32_t i = 0; i < kMaxRxPerCycle; ++i) {
      if (!g_transport.recv(frame)) {
        break;
      }
      ++g_stats.rxFrames;
      g_can4Bus.dispatch(frame);
    }

    // ---------------- 发帧 ----------------
    g_can4Bus.txService();

    // ---------------- 清中断标志 ----------------
    // INT# 是电平信号，不清标志就不会回到高电平
    if (g_transport.chip().readInterrupt() != 0) {
      g_transport.chip().clearInterrupt();
    }

    // ---------------- 总线状态 ----------------
    const BspCan::Mcp2518fd::Status st = g_transport.chip().status();
    g_stats.tec = st.tec;
    g_stats.rec = st.rec;
    g_stats.spiFail = g_transport.chip().spiFailCount();

    // SPI 出过错，芯片内部状态就不可信：停止出队，延时后重走初始化
    if (!g_transport.chip().spiHealthy() || st.busOff) {
      g_stats.phase = Phase::ConfigFail;
      osDelay(kRetryDelayMs);
      continue;
    }

    // ---------------- RX 假死自愈 ----------------
    {
      static uint32_t wdLastRxFrames = 0;
      static uint32_t wdLastTxFrames = 0;
      static uint32_t wdLastRxMs = 0;
      static uint32_t wdLastRecoverMs = 0;
      const uint32_t nowMs = osKernelGetTickCount();

      if (g_stats.rxFrames != wdLastRxFrames) {
        wdLastRxFrames = g_stats.rxFrames;
        wdLastRxMs = nowMs;
        g_rxSilentMs = 0;
      } else {
        g_rxSilentMs = nowMs - wdLastRxMs;
      }
      const bool txAlive = (g_stats.txFrames != wdLastTxFrames);
      wdLastTxFrames = g_stats.txFrames;
      const bool busAcked = (st.tec == 0) && (st.rec == 0);
      if (busAcked && txAlive && (g_rxSilentMs >= kRxWatchdogSilentMs) &&
          ((nowMs - wdLastRecoverMs) >= kRxWatchdogRetryMs)) {
        wdLastRecoverMs = nowMs;
        ++g_rxRecover;
        if (!g_transport.start()) {
          g_stats.phase = Phase::ConfigFail;
        }
        g_initStep = g_transport.chip().initStep();
        wdLastRxMs = osKernelGetTickCount();
        g_rxSilentMs = 0;
      }
    }
  }
}

extern "C" void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
  if (GPIO_Pin == INT_CAN_Pin) {
    SpiCanApp::onExti();
  }
}
