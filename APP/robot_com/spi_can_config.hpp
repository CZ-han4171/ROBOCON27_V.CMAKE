/**
 * @file spi_can_config.hpp
 * @brief 第 4 路 CAN FD：STM32H723 SPI4 + MCP2518FD + TCAN1044V
 * @version 0.1
 * @date 2026-11-07
 *
 * @copyright Copyright (c) 2026
 *
 * @attention : 1. 芯片初始化放在任务里做，不要放 Robot_Init()。那里是关中断状态，
 *              HAL 阻塞式 SPI 靠 SysTick 计时超时，关中断时 HAL_GetTick() 不前进，
 *              芯片不应答就会死等
 *              2. 所有 SPI 事务只允许在 can4SendTask 里发起
 * @note : 1. 帧长上限 64 字节，发送是"写 RAM + 置 TXREQ"，没有 Mailbox
 *         2. 接收靠 INT# 下降沿唤醒 + 任务里主动读 SPI，中断里不搬数据
 *         3. 和三路片上 FDCAN 完全隔离：Canbus.hpp / Canbus.cpp 一行不动，
 *            CAN4 有自己的 Can4Bus（设备表 + 发送队列）和 can4SendTask。
 *            但设备仍然复用 CanDevice，matchRx / onRx 的语义和三路 FDCAN 一致
 *         4. 只做经典帧 8 字节的上层语义，和 CanDevice::onRx 保持一致
 * @versioninfo : 0.1 独立总线（Can4Bus），不依赖 CanBus 的后端抽象
 */
#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include "cmsis_os.h"

// 第 4 路 CAN 任务，5ms 周期，同时被 INT# 唤醒
void can4SendTask(void *argument);

#ifdef __cplusplus
}
#endif

#ifdef __cplusplus

#include "Canbus.hpp"
#include "SpiCanTransport.hpp"
#include "lockfree_queue.hpp"

namespace SpiCanApp {

// 初始化进度，失败时用来区分"SPI 不通"和"SPI 通了但配置失败"
enum class Phase : uint8_t {
  Init = 0,        // 还没尝试
  SpiDead = 1,     // 读不到 DEVID：接线、片选、供电、晶振
  ConfigFail = 2,  // SPI 通但配置失败：波特率组合、ISO 位、模式切换
  Running = 3,
};

// 调试快照，调试器里直接看变量，不占 CPU
struct Stats {
  Phase phase = Phase::Init;
  uint32_t rxFrames = 0;  // 累计收到帧数
  uint32_t txFrames = 0;  // 累计成功推给芯片的帧数
  uint32_t rxLost = 0;    // RX FIFO 溢出次数
  uint32_t irqCount = 0;  // INT# 次数
  uint32_t lastRxId = 0;  // 最后一帧的 ID
  uint8_t lastRxLen = 0;  // 最后一帧的字节数
  uint16_t devId = 0;     // DEVID 原始 12 位：[11:4]=ID [3:0]=REV
  uint8_t tec = 0;        // 发送错误计数
  uint8_t rec = 0;        // 接收错误计数
  uint32_t rxNoMatch = 0;  // 收到但没有任何设备认领的帧
  uint32_t txLost = 0;     // 出队后推给芯片失败（FIFO 满 / SPI 出错）
  uint32_t spiFail = 0;    // SPI 事务失败累计，非 0 会触发重新初始化
};

extern Stats g_stats;

/**
 * @brief CAN4 总线管理：设备表 + 发送队列
 *
 * 生产者（各任务的 addCanMsg）只入队，消费者只有 can4SendTask 一个，
 * 所以用 MPSC 无锁队列。这里只认 CanDevice，不认 CanBus：
 * 设备类（C610Motor / C620Motor / ...）一行都不用改，只是挂到 CAN4 上时
 * 构造参数里的 manager 传 nullptr（原因见下面"注意事项"第 1 条）。
 */
class Can4Bus {
public:
  static constexpr uint8_t kMaxDevices = 12;
  static constexpr uint8_t kTxDepth = 8;

  struct Pack {
    uint32_t id = 0;
    uint8_t len = 0;
    bool ext = false;
    bool fd = false;   // true = CAN FD 帧
    bool brs = false;  // 仅 FD 帧有效
    uint8_t data[8] = {0};
  };

  // 返回 0 = 成功，1 = 参数非法，2 = 设备表已满
  uint8_t registerDevice(CanDevice *device);
  uint8_t deviceCount() const;

  // 入队一帧，返回 false = 队列满。入队成功后会唤醒 can4SendTask
  bool addCanMsg(const Pack &pack);
  bool addCanMsg(uint32_t id, const uint8_t data[8], uint8_t len,
                 bool ext = false, bool fd = false, bool brs = false);

  // 入队成功后的回调，can4SendTask 里注册一次即可
  void setTxNotify(void (*fn)(void)) { tx_notify_ = fn; }

  // 把队列里的帧推给芯片，只能在 can4SendTask 里调用
  void txService();
  // 收到一帧后分发给设备，只能在 can4SendTask 里调用
  void dispatch(const BspCan::FdFrame &frame);

private:
  CanDevice *device_[kMaxDevices] = {};
  Algorithm::MpscQueue<Pack, kTxDepth> txQueue_;
  void (*tx_notify_)(void) = nullptr;
};

// 全局实例，和 CanBus 的三路总线并列
extern Can4Bus g_can4Bus;

/*
 * CAN4 用法，只有三件事：
 *
 *   // 1) 定义设备：manager 传 nullptr（CAN4 上没有 CanBus）
 *   //    DJI 合法 ID 组合只有 0x200<->0x201~0x204、0x1FF<->0x205~0x208
 *   C610Motor can4_motor1(nullptr, 0x205, 0, 0x1FF, 0);
 *
 *   // 2) 挂总线：写在 comServiceInit() 里
 *   can4_motor1.init();
 *   can4Attach(&can4_motor1);
 *
 *   // 3) 发帧：任何任务都能调，内部自动选直发还是入队
 *   SpiCanApp::Can4Bus::Pack pack;
 *   pack.id = 0x1FF;  pack.ext = false;
 *   const uint32_t ids[4] = {0x205, 0, 0, 0};          // 0 = 该槽位不下发
 *   int16_t cmd[4] = {(int16_t)can4_motor1.cmdTrans(), 0, 0, 0};
 *   packDJIMotorCanMsg(pack.id, ids, cmd, 4, pack.data, pack.len);
 *   can4Send(pack.id, pack.data, pack.len);
 *
 *   收帧不用管：can4SendTask 的 dispatch() 按 matchRx 自动调 onRx()。
 *
 * ⚠ 别调 can4_motor1.send() / mitControl() / dmMotorEnable()——它们走
 *   manager_->addCanMsg()，而 CAN4 的 manager 是 nullptr。发送只能在任务里组帧。
 */

// 唯一实例，中断回调和任务共用
BspCan::SpiCanTransport &transport();

/**
 * @brief 绕过队列直接发一帧（自测/回环用，业务代码用 can4Send）
 * @warning 只能在 can4SendTask 里调，其它任务调用直接返回 false。
 * @param len 字节数 0 ~ 64
 * @return false = 芯片发送队列满或参数非法
 */
bool sendFrame(uint32_t id, const uint8_t *data, uint8_t len, bool ext = false,
               bool fd = false);

/**
 * @brief 发一帧，任何任务都能直接调（业务代码只用这个）
 *
 * 在 can4SendTask 里 -> 直接写芯片；别的任务 -> 入队 + 唤醒，最多晚 5ms。
 *
 * @param data 跨任务时最多 8 字节（FD 长帧只能本任务内发）
 * @param len  0 ~ 8
 * @param fd   true = CAN FD 帧
 * @return false = 队列满 / 芯片发送队列满 / SPI 出错
 * @note  true 只代表已入队或已交给芯片，实际发出多少看 g_stats.txFrames
 */
bool can4Send(uint32_t id, const uint8_t *data, uint8_t len, bool ext = false,
              bool fd = false);

/**
 * @brief 把设备挂到 CAN4 上，等价于 g_can4Bus.registerDevice()
 * @return 0 = 成功，1 = 参数非法，2 = 设备表已满
 */
uint8_t can4Attach(CanDevice *dev);

Stats snapshot();

} // namespace SpiCanApp
#endif
