/**
 * @file spi_can_config.cpp
 * @brief 第 4 路 CAN FD（MCP2518FD + SPI4）的板级配置与收发任务
 *
 * 和三路片上 FDCAN 完全隔离
 * 只复用 CanDevice 基类（matchRx / onRx），总线、队列、任务、统计全是 CAN4 自己的。
 */
#include "spi_can_config.hpp"

#include "FreeRTOS.h"
#include "task.h"

#include "main.h"
#include "spi.h"

#include <cstring>

namespace SpiCanApp {
namespace {

// 板级接线
// SCK  PE12 -> R36 22R -> U24 pin10 SCK
// MOSI PE14 -> R35 22R -> U24 pin11 SDI
// MISO PE13 -> R34 22R -> U24 pin12 SDO
// CS   PE11 -> R33 22R -> U24 pin13 nCS（低有效）
// INT  PE10 <- R24 22R -  U24 pin4  INT#（低有效）
// 晶振 X2 = 20MHz，收发器 TCAN1044V（VCC 5V / VIO 3V3）
const BspCan::SpiCanPins kPins = {
    CS_CAN_GPIO_Port, CS_CAN_Pin,
    INT_CAN_GPIO_Port, INT_CAN_Pin,
    nullptr, 0,  // 没引出 nRESET，只能走 SPI 软复位
};

constexpr uint32_t kTaskPeriodMs = 5;     // 空闲时 5ms 醒一次
constexpr uint32_t kMaxRxPerCycle = 16;   // 一轮最多搬 16 帧
constexpr uint32_t kFlagInt = 0x01u;      // 任务通知标志：INT# 拉低
constexpr uint32_t kFlagTx = 0x02u;       // 任务通知标志：有新帧进发送队列
constexpr uint32_t kFlagWake = kFlagInt | kFlagTx;
constexpr uint32_t kRetryDelayMs = 1000;  // 初始化失败后的重试间隔

// 自测心跳：1s 发一帧经典帧 0x123，普通 USB-CAN 分析仪就能看到。
// 上车之前保持 false
constexpr bool kHeartbeat = false;
constexpr uint32_t kHeartbeatId = 0x123;
constexpr uint32_t kHeartbeatPeriodMs = 1000;

// 业务发送示例开关：默认关闭。改成 true 就实测
//   addCanMsg() -> txQueue_ -> 本任务 txService() -> g_transport.send()
// 这条链。业务代码请直接用 can4Send()——它内部自动选"本任务直发"还是"入队唤醒"；
// 这里绕开 can4Send 直接调 addCanMsg，只是为了单独验证队列这一段。
constexpr bool kDemoTx = false;
constexpr uint32_t kDemoTxId = 0x124;         // 换一个不和心跳重复的 ID
constexpr uint32_t kDemoTxPeriodMs = 100;

BspCan::SpiCanTransport::Options makeOptions() {
  BspCan::SpiCanTransport::Options o;
  o.sysclkHz = 20000000;       // 晶振20MHz
  o.usePll = false;
  o.nominalBitrate = 1000000;  // 20MHz / 1Mbps = 20 tq，采样点 65%
  o.dataBitrate = 1000000;     // 20MHz / 1Mbps = 20 tq
  o.nominalSamplePoint = 65;
  o.dataSamplePoint = 70;
  o.isoCanFd = true;
  return o;
}

BspCan::SpiCanTransport g_transport(hspi4, kPins, makeOptions());

osThreadId_t g_taskId = nullptr;

// 发送队列入队成功后由 Can4Bus 回调：不等 5ms 周期，直接叫醒本任务把帧推出去。
// 已经在 can4SendTask 里（比如设备在 onRx 里回帧）就不用自己叫醒自己，
// 同一轮后面的 txService() 会把它推出去。
void notifyTxPending() {
  if (g_taskId != nullptr && osThreadGetId() != g_taskId) {
    (void)osThreadFlagsSet(g_taskId, kFlagTx);
  }
}

// EXTI 上下文：只置标志、唤醒任务
void onExti() {
  g_transport.onInterrupt();
  if (g_taskId != nullptr) {
    osThreadFlagsSet(g_taskId, kFlagInt);
  }
}

} // namespace

Stats g_stats;

BspCan::SpiCanTransport &transport() { return g_transport; }

Stats snapshot() { return g_stats; }

// 全局唯一实例，和 CanBus 的三路总线并列
Can4Bus g_can4Bus;

// ---------------------------------------------------------------------------
// CAN4 总线：设备表 + 发送队列
// ---------------------------------------------------------------------------

uint8_t Can4Bus::registerDevice(CanDevice *device) {
  if (device == nullptr) {
    return 1;
  }
  for (uint8_t i = 0; i < kMaxDevices; ++i) {
    if (device_[i] == device) {
      return 0;  // 重复注册直接当成功
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

uint8_t Can4Bus::deviceCount() const {
  uint8_t n = 0;
  for (uint8_t i = 0; i < kMaxDevices; ++i) {
    if (device_[i] != nullptr) {
      ++n;
    }
  }
  return n;
}

bool Can4Bus::addCanMsg(const Pack &pack) {
  // Pack 只有 8 字节载荷，超了会在 txService() 里按 pack.len 越界读
  if (static_cast<size_t>(pack.len) > sizeof(pack.data)) {
    return false;
  }
  if (txQueue_.TryPush(pack) != Algorithm::QueueError::OK) {
    return false;
  }
  // 唤醒 can4SendTask：不然要等 5ms 周期才轮到这一帧
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
  // 上层设备接口固定 8 字节，FD 长帧只取前 8 个
  const uint8_t len = (frame.len > 8) ? 8 : frame.len;

  g_stats.lastRxId = frame.id;
  g_stats.lastRxLen = len;

  // 远程帧没有数据段，CanDevice::onRx 的语义就是"数据帧到了"，直接丢掉
  if (frame.rtr) {
    return;
  }

  // 三路 FDCAN 上写的设备类和挂到这里的完全相同
  FDCAN_RxHeaderTypeDef rx_header = {0};
  rx_header.Identifier = frame.id;
  rx_header.IdType = frame.ext ? FDCAN_EXTENDED_ID : FDCAN_STANDARD_ID;
  rx_header.RxFrameType = FDCAN_DATA_FRAME;
  rx_header.DataLength = BspCan::lenToDlc(len);
  rx_header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
  // 按真实帧填：matchRx 目前只看 Identifier / IdType，但以后有设备想按
  // FD / BRS 区分时（同一 ID 的经典帧和 FD 帧语义不同）拿到的就是真值
  rx_header.BitRateSwitch = frame.brs ? FDCAN_BRS_ON : FDCAN_BRS_OFF;
  rx_header.FDFormat = frame.fdf ? FDCAN_FD_CAN : FDCAN_CLASSIC_CAN;

  bool matched = false;
  for (uint8_t i = 0; i < kMaxDevices; ++i) {
    if (device_[i] == nullptr) {
      continue;
    }
    if (!device_[i]->matchRx(rx_header)) {
      continue;
    }
    device_[i]->onRx(frame.data, len);
    matched = true;
  }
  if (!matched) {
    ++g_stats.rxNoMatch;
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

bool sendFrame(uint32_t id, const uint8_t *data, uint8_t len, bool ext,
               bool fd) {
  //片选和芯片内部状态都不允许并发访问：这个口只给 can4SendTask
  if (osThreadGetId() != g_taskId) {
    return false;
  }
  if (data == nullptr || len > BspCan::kFdMaxPayload) {
    return false;
  }
  BspCan::FdFrame f;
  f.id = id;
  f.len = len;
  f.ext = ext;
  f.fdf = fd;
  f.brs = fd;  // FD 帧默认开数据段变速
  std::memcpy(f.data, data, len);
  return g_transport.send(f);
}

uint8_t can4Attach(CanDevice *dev) { return g_can4Bus.registerDevice(dev); }

bool can4Send(uint32_t id, const uint8_t *data, uint8_t len, bool ext, bool fd) {
  if (data == nullptr) {
    return false;
  }
  // 本任务：直接写芯片，省掉入队 + 唤醒的一次往返
  if (osThreadGetId() == g_taskId) {
    return sendFrame(id, data, len, ext, fd);
  }
  // 别的任务：入队（MPSC 无锁），由 can4SendTask 出队后推给芯片。
  // Pack 只有 8 字节载荷，FD 长帧只能在本任务里用 sendFrame 直发
  if (len > sizeof(Can4Bus::Pack::data)) {
    return false;
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
  // 入队成功立刻唤醒本任务，不用等 5ms 周期
  g_can4Bus.setTxNotify(&notifyTxPending);

  uint32_t hbTick = osKernelGetTickCount();
  uint8_t seq = 0;

  for (;;) {
    // 5ms 超时，或 INT# 拉低 / 有帧要发时被立即唤醒，先到先算
    (void)osThreadFlagsWait(kFlagWake, osFlagsWaitAny, kTaskPeriodMs);
    g_stats.irqCount = g_transport.irqCount();

    // ---------------- 初始化 / 失败重试 ----------------
    if (g_stats.phase != Phase::Running) {
      uint8_t id = 0;
      uint8_t rev = 0;
      if (!g_transport.chip().readDevId(id, rev)) {
        // 读不到 DEVID：查 SPI 接线、片选、3.3V/5V、晶振起振
        g_stats.phase = Phase::SpiDead;
        osDelay(kRetryDelayMs);
        continue;
      }
      // 保留 12 位原始值：ID 在 [11:4]、REV 在 [3:0]，截成 uint8_t 会丢掉 ID 高 4 位
      g_stats.devId = static_cast<uint16_t>((id << 4) | (rev & 0x0F));

      g_stats.phase = g_transport.start() ? Phase::Running : Phase::ConfigFail;
      if (g_stats.phase != Phase::Running) {
        osDelay(kRetryDelayMs);
        continue;
      }
      hbTick = osKernelGetTickCount();
    }

    // ---------------- 收帧 ----------------
    // 一轮最多搬 kMaxRxPerCycle 帧（= RX FIFO 深度），按 ID 分发给注册的设备
    BspCan::FdFrame frame;
    for (uint32_t i = 0; i < kMaxRxPerCycle; ++i) {
      if (!g_transport.recv(frame)) {
        break;
      }
      ++g_stats.rxFrames;
      g_can4Bus.dispatch(frame);
    }

    // ---------------- 业务发送示例（默认关闭）----------------
    // 队友该抄的发送姿势就这三段：组帧 -> addCanMsg 入队 -> 本任务 txService 推出。
    // 别的任务里也是同一行 addCanMsg：队列是 MPSC 无锁的，入队会唤醒本任务，
    // 既不用自己等 5ms 周期，也不用碰 SPI。想挂到真实电机上，把 payload 换成
    // packDJIMotorCanMsg() 组出来的 pack 即可（见 spi_can_config.hpp 头注释）。
    if constexpr (kDemoTx) {
      static uint32_t demoTick = 0;
      static uint8_t demoSeq = 0;
      const uint32_t now = osKernelGetTickCount();
      if ((now - demoTick) >= kDemoTxPeriodMs) {
        demoTick = now;
        uint8_t payload[8];
        for (uint8_t i = 0; i < 8; ++i) {
          payload[i] = demoSeq++;
        }
        // false = 队列满（kTxDepth=8）；发没发出去由下面的 txService() 统计
        (void)g_can4Bus.addCanMsg(kDemoTxId, payload, 8, false, false);
      }
    }

    // ---------------- 发帧 ----------------
    // 各任务里的 addCanMsg() 只是入队，真正推给芯片只能在任务上下文做
    g_can4Bus.txService();

    // ---------------- 清芯片中断标志 ----------------
    // INT# 是电平信号，芯片内部 IF 不清就不会回到高电平，
    // 下一个事件也就没有下降沿，只能靠 5ms 轮询兜底
    const uint32_t irq = g_transport.chip().readInterrupt();
    if (irq != 0) {
      using namespace BspCan::mcp2518fd;
      if ((irq & kIntRxOvIf) != 0) {
        ++g_stats.rxLost;  // RX FIFO 溢出，说明搬得不够快或队列太小
      }
      // C1INT 是"写 0 清零"：一次把可清标志清掉，中断使能位保持不变。
      // RXIF/TXIF/RXOVIF 是 FIFO 的只读汇总，UINC 完自己会落，不用软件清
      g_transport.chip().clearInterrupt();
    }

    // ---------------- 总线状态 ----------------
    const BspCan::Mcp2518fd::Status st = g_transport.chip().status();
    g_stats.tec = st.tec;
    g_stats.rec = st.rec;
    g_stats.spiFail = g_transport.chip().spiFailCount();
    // SPI 一旦出过错，芯片内部状态就不可信了：停止出队，延时后重新走一遍初始化
    if (!g_transport.chip().spiHealthy()) {
      g_stats.phase = Phase::ConfigFail;
      osDelay(kRetryDelayMs);
      continue;
    }
    if (st.busOff) {
      // bus-off 只能重新初始化才能恢复，下一轮走重试分支
      g_stats.phase = Phase::ConfigFail;
      osDelay(kRetryDelayMs);
      continue;
    }

    // ---------------- 自测心跳 ----------------
    if constexpr (kHeartbeat) {
      const uint32_t now = osKernelGetTickCount();
      if ((now - hbTick) >= kHeartbeatPeriodMs) {
        hbTick = now;
        uint8_t payload[8];
        for (uint8_t i = 0; i < 8; ++i) {
          payload[i] = seq++;
        }
        if (sendFrame(kHeartbeatId, payload, 8, false, false)) {
          ++g_stats.txFrames;
        }
      }
    }
  }
}

extern "C" void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
  if (GPIO_Pin == INT_CAN_Pin) {
    SpiCanApp::onExti();
  }
}
