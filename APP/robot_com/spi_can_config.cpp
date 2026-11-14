/**
 * @file spi_can_config.cpp
 * @brief 第 4 路 CAN FD（MCP2518FD + SPI4）的板级配置与收发任务
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
constexpr uint32_t kFlagInt = 0x01u;      // 任务通知标志：INT# 拉低
constexpr uint32_t kFlagTx = 0x02u;       // 任务通知标志：有新帧进发送队列
constexpr uint32_t kFlagWake = kFlagInt | kFlagTx;
constexpr uint32_t kRetryDelayMs = 1000;  // 初始化失败后的重试间隔

// 自测心跳：1s 发一帧经典帧 0x123，普通 USB-CAN 分析仪就能看到。
// 上真实总线之前把它改成 false。
constexpr bool kHeartbeat = true;
constexpr uint32_t kHeartbeatId = 0x123;
constexpr uint32_t kHeartbeatPeriodMs = 1000;

BspCan::SpiCanTransport::Options makeOptions() {
  BspCan::SpiCanTransport::Options o;
  o.sysclkHz = 20000000;       // 板上晶振 20MHz
  o.usePll = false;
  o.nominalBitrate = 1000000;  // 20MHz / 1Mbps = 20 tq，采样点 80%
  o.dataBitrate = 2000000;     // 20MHz / 2Mbps = 10 tq
  o.nominalSamplePoint = 80;
  o.dataSamplePoint = 75;
  o.isoCanFd = true;
  return o;
}

BspCan::SpiCanTransport g_transport(hspi4, kPins, makeOptions());

osThreadId_t g_taskId = nullptr;

// 发送队列入队成功后由 CanBus 回调：不等 5ms 周期，直接叫醒本任务把帧推出去。
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

// 第 4 路总线：上层是同一个 CanBus，后端换成 SPI 外挂的 MCP2518FD。
// 从此 CAN4 上的设备就是普通的 CanDevice，四路总线用法完全一致。
CanBus can4_bus{transport()};

Stats snapshot() { return g_stats; }

bool sendFrame(uint32_t id, const uint8_t *data, uint8_t len, bool ext,
               bool fd) {
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

} // namespace SpiCanApp

extern "C" void can4SendTask(void *argument) {
  (void)argument;
  using namespace SpiCanApp;

  g_taskId = osThreadGetId();
  can4_bus.setTxNotify(&notifyTxPending);

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
      g_stats.devId = static_cast<uint8_t>((id << 4) | (rev & 0x0F));

      g_stats.phase = g_transport.start() ? Phase::Running : Phase::ConfigFail;
      if (g_stats.phase != Phase::Running) {
        // 不要 while(1) 也不要 Error_Handler，让其它任务继续跑
        osDelay(kRetryDelayMs);
        continue;
      }
      hbTick = osKernelGetTickCount();
    }

    // ---------------- 收帧 ----------------
    // 一轮最多搬 8 帧，按 ID 分发给 registerDevice() 注册的设备
    can4_bus.pollRx();

    // ---------------- 发帧 ----------------
    // 各任务里的 addCanMsg() 只是入队，真正推给芯片只能在任务上下文做
    can4_bus.txService();

    // 总线内部计数同步到调试快照
    g_stats.rxFrames = can4_bus.rxFrameCount();
    g_stats.txFrames = can4_bus.txFrameCount();
    g_stats.rxNoMatch = can4_bus.rxNoMatchCount();
    g_stats.txLost = can4_bus.txFailCount();
    g_stats.lastRxId = can4_bus.lastRxId();
    g_stats.lastRxLen = can4_bus.lastRxLen();

    // ---------------- 清芯片中断标志 ----------------
    // INT# 是电平信号，芯片内部 IF 不清就不会回到高电平，
    // 下一个事件也就没有下降沿，只能靠 5ms 轮询兜底
    const uint32_t irq = g_transport.chip().readInterrupt();
    if (irq != 0) {
      using namespace BspCan::mcp2518fd;
      if ((irq & kIntRxOvIf) != 0) {
        ++g_stats.rxLost;  // RX FIFO 溢出，说明搬得不够快或队列太小
      }
      g_transport.chip().clearInterrupt(irq & (kIntRxIf | kIntTxIf |
                                               kIntCerIf | kIntSerIf |
                                               kIntRxOvIf | kIntModIf));
    }

    // ---------------- 总线状态 ----------------
    const BspCan::Mcp2518fd::Status st = g_transport.chip().status();
    g_stats.tec = st.tec;
    g_stats.rec = st.rec;
    if (st.busOff) {
      // bus-off 只能重新初始化才能恢复，下一轮走重试分支
      g_stats.phase = Phase::ConfigFail;
      continue;
    }

    // ---------------- 自测心跳 ----------------
    if constexpr (kHeartbeat) {
      const uint32_t now = osKernelGetTickCount();
      if ((now - hbTick) >= kHeartbeatPeriodMs) {
        hbTick = now;
        // 走和真实设备同一条队列，顺带验证 addCanMsg -> txService 通路
        CanBus::ClassicPack pack{};
        pack.id = kHeartbeatId;
        pack.type = CanBus::Type::STANDARD;
        pack.DLC = 8;
        for (uint8_t i = 0; i < 8; ++i) {
          pack.data[i] = seq++;
        }
        (void)can4_bus.addCanMsg(pack);
      }
    }
  }
}

extern "C" void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
  if (GPIO_Pin == INT_CAN_Pin) {
    SpiCanApp::onExti();
  }
}
