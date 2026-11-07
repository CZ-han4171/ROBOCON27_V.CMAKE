/**
 * @file Canbus.cpp
 * @author Keten (2863861004@qq.com)
 * @brief
 * @version 0.1
 * @date 2026-04-18
 *
 * @copyright Copyright (c) 2026
 *
 * @attention :
 * @note :
 * @versioninfo :
 */
#include "Canbus.hpp"
#include "main.h"
#include "stm32h7xx_hal_def.h"
#include "stm32h7xx_hal_fdcan.h"
#include <cstdint>
#include <cstring>

namespace {

int8_t busIndexFromHandle(FDCAN_HandleTypeDef *hfdcan) {
  if (hfdcan == &hfdcan1) {
    return 0;
  }
  if (hfdcan == &hfdcan2) {
    return 1;
  }
  if (hfdcan == &hfdcan3) {
    return 2;
  }
  return -1;
}

} // namespace

CanBus *CanBus::map[3] = {nullptr, nullptr, nullptr};

/**
 * @brief 配置过滤器
 *
 * @param hcan
 * @param idType 接收扩展帧or标准帧
 * @param bank
 * @param fifo
 * @param id
 * @param maskId
 */
void canFilterInit(FDCAN_HandleTypeDef *hcan, uint32_t idType, uint32_t fifo,
                   uint32_t id, uint32_t maskId) {
  FDCAN_FilterTypeDef sFilterConfig = {0};

  sFilterConfig.FilterConfig = fifo; // 消息过滤器配置为接收FIFO0或FIFO1
  sFilterConfig.FilterID1 = id;      // 过滤器ID1
  sFilterConfig.FilterID2 = maskId;  // 过滤器ID2
  sFilterConfig.FilterIndex = 0;     // 过滤器编号
  sFilterConfig.FilterType = FDCAN_FILTER_MASK; // 掩码模式
  sFilterConfig.IdType = idType;                // 标准帧或拓展帧
  sFilterConfig.IsCalibrationMsg = 0;           // 不是标定消息
  sFilterConfig.RxBufferIndex = 0;

  if (HAL_FDCAN_ConfigFilter(hcan, &sFilterConfig) != HAL_OK) {
    Error_Handler();
  }

  // 配置全局过滤器
  HAL_FDCAN_ConfigGlobalFilter(hcan, FDCAN_REJECT, FDCAN_REJECT,
                               FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE);
}

// can外设初始化
void bspCanInit(FDCAN_HandleTypeDef *hcan) {
  // 使能can外设中断
  if (HAL_FDCAN_ActivateNotification(
          hcan,
          FDCAN_IT_RX_FIFO0_NEW_MESSAGE | FDCAN_IT_RX_FIFO1_NEW_MESSAGE |
              FDCAN_IT_BUS_OFF | FDCAN_IT_ERROR_PASSIVE,
          0) != HAL_OK) {
    Error_Handler();
  }

  // 使能can发送完成中断
  if (HAL_FDCAN_ActivateNotification(hcan, FDCAN_IT_TX_FIFO_EMPTY, 0) !=
      HAL_OK) {
    Error_Handler();
  }

  // 使能can外设
  if (HAL_FDCAN_Start(hcan) != HAL_OK) {
    Error_Handler();
  }
}

extern "C" void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan,
                                          uint32_t RxFifo0ITs) {
  if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) == 0) {
    return;
  }
  // 拿到canbus
  const int8_t idx = busIndexFromHandle(hfdcan);
  CanBus *bus =
      (idx >= 0) ? CanBus::instanceByIndex(static_cast<uint8_t>(idx)) : nullptr;
  // 分发对应的can帧
  if (bus != nullptr) {
    bus->processRxInterrupt(FDCAN_RX_FIFO0);
  }
}

extern "C" void HAL_FDCAN_RxFifo1Callback(FDCAN_HandleTypeDef *hfdcan,
                                          uint32_t RxFifo1ITs) {
  if ((RxFifo1ITs & FDCAN_IT_RX_FIFO1_NEW_MESSAGE) == 0) {
    return;
  }

  const int8_t idx = busIndexFromHandle(hfdcan);
  CanBus *bus =
      (idx >= 0) ? CanBus::instanceByIndex(static_cast<uint8_t>(idx)) : nullptr;
  if (bus != nullptr) {
    bus->processRxInterrupt(FDCAN_RX_FIFO1);
  }
}

extern "C" void HAL_FDCAN_TxBufferCompleteCallback(FDCAN_HandleTypeDef *hfdcan,
                                                   uint32_t BufferIndexes) {
  const int8_t idx = busIndexFromHandle(hfdcan);
  CanBus *bus =
      (idx >= 0) ? CanBus::instanceByIndex(static_cast<uint8_t>(idx)) : nullptr;
  if (bus != nullptr) {
    bus->txService();
  }
}

extern "C" void HAL_FDCAN_TxFifoEmptyCallback(FDCAN_HandleTypeDef *hfdcan) {
  const int8_t idx = busIndexFromHandle(hfdcan);
  CanBus *bus =
      (idx >= 0) ? CanBus::instanceByIndex(static_cast<uint8_t>(idx)) : nullptr;
  if (bus != nullptr) {
    bus->txService();
  }
}

extern "C" void HAL_FDCAN_ErrorCallback(FDCAN_HandleTypeDef *hfdcan) {
  // 清除错误标志位
  // hfdcan->ErrorCode = HAL_FDCAN_ERROR_NONE;
  while(1){}
}

uint32_t error_state = 0;

extern "C" void HAL_FDCAN_ErrorStatusCallback(FDCAN_HandleTypeDef *hfdcan,
                                              uint32_t ErrorStatusITs) {
                                                error_state = ErrorStatusITs;
}

CanBus *CanBus::instanceByIndex(uint8_t idx) {
  if (idx >= 3U) {
    return nullptr;
  }
  return map[idx];
}

uint8_t CanBus::init(void) {
  // 只有片上 FDCAN 需要登记到 map，供 HAL 回调按句柄找回总线；
  // SPI 外挂控制器没有 HAL 句柄，不参与中断路由
  if (impl_ != nullptr) {
    const int8_t idx = busIndexFromHandle(impl_);
    if (idx < 0) {
      return 1;
    }
    map[idx] = this;
  }

  for (uint8_t i = 0; i < kMaxDevices; ++i) {
    device_[i] = nullptr;
  }
  txMailbox = 0;

  return 0;
}

uint8_t CanBus::registerDevice(CanDevice *device) {
  if (device == nullptr) {
    return 1;
  }

  for (uint8_t i = 0; i < kMaxDevices; ++i) {
    if (device_[i] == device) {
      return 0;
    }
  }

  for (uint8_t i = 0; i < kMaxDevices; ++i) {
    if (device_[i] == nullptr) {
      device_[i] = device;
      return 0;
    }
  }

  return 2;
}

uint8_t CanBus::addCanMsg(const ClassicPack &pack) {
  if (tx_queue_.TryPush(pack) != Algorithm::QueueError::OK) {
    return 1;
  }

  if (tx_notify_ != nullptr) {
    // 后端只能在它自己的任务里发帧（SPI 外挂控制器），交给任务去推
    tx_notify_();
  } else {
    // 片上 FDCAN：入队后顺手试发一次
    txService();
  }
  return 0;
}

// 处理接收中断：只取一帧，剩下的靠中断再来或 pollRx() 兜底
void CanBus::processRxInterrupt(uint32_t fifo) {
  BspCan::FdFrame frame;
  // FDCAN_RX_FIFO0 = 0，FDCAN_RX_FIFO1 = 1
  if (!backend_->recvFrom((fifo == FDCAN_RX_FIFO1) ? 1U : 0U, frame)) {
    return;
  }
  dispatch(frame);
}

// 主动搬一轮：SPI 后端没有中断里搬数据的条件，只能在任务里轮询
void CanBus::pollRx() {
  BspCan::FdFrame frame;
  for (uint8_t i = 0; i < kMaxRxPerPoll; ++i) {
    if (!backend_->recv(frame)) {
      break;
    }
    dispatch(frame);
  }
}

// 一帧收到后按 ID 分发给设备，FDCAN 和 SPI 走的是同一段代码
void CanBus::dispatch(const BspCan::FdFrame &frame) {
  // 设备回调只有 8 字节，FD 长帧先截断
  const uint8_t len = (frame.len > 8U) ? 8U : frame.len;

  last_rx_id_ = frame.id;
  last_rx_len_ = len;
  ++rx_frames_;

  // 造一个 FDCAN 报文头，CanDevice::matchRx 的签名就不用动
  FDCAN_RxHeaderTypeDef rx_header = {0};
  rx_header.Identifier = frame.id;
  rx_header.IdType = frame.ext ? FDCAN_EXTENDED_ID : FDCAN_STANDARD_ID;
  rx_header.RxFrameType = frame.rtr ? FDCAN_REMOTE_FRAME : FDCAN_DATA_FRAME;
  rx_header.DataLength = BspCan::lenToDlc(len);
  rx_header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
  rx_header.BitRateSwitch = FDCAN_BRS_OFF;
  rx_header.FDFormat = FDCAN_CLASSIC_CAN;

  bool matched = false;
  for (uint8_t i = 0; i < kMaxDevices; ++i) {
    if (device_[i] == nullptr) {
      continue;
    }
    // 设备轮询
    if (!device_[i]->matchRx(rx_header)) {
      continue;
    }
    // 查到设备则调用其回调
    device_[i]->onRx(frame.data, len);
    matched = true;
  }

  if (!matched) {
    ++rx_no_match_;
  }
}

// 处理发送中断
void CanBus::processTxInterrupt() { txService(); }

// can发送硬件的串行化
void CanBus::txService() {

  // 进来尝试拿到锁
  if (tx_lock_.exchange(1, std::memory_order_acquire) != 0) {
    // 拿不到则设置忙碌
    tx_pend_.store(1, std::memory_order_release);
    return;
  }

  do {
    // 清除一次busy pend
    tx_pend_.store(0, std::memory_order_release);

    // 然后硬件发送
    // 如果有可用fifo
    while (backend_->txFreeLevel() > 0U) {
      ClassicPack pack = {0};
      if (tx_queue_.TryPop(pack) != Algorithm::QueueError::OK) {
        break;
      }

      // 上层只给 8 字节，DLC 超 8 的按 8 字节发
      BspCan::FdFrame frame;
      frame.id = pack.id;
      frame.ext = (pack.type == Type::EXTENDED);
      frame.len = (pack.DLC > 8U) ? 8U : static_cast<uint8_t>(pack.DLC);
      std::memcpy(frame.data, pack.data, 8);

      // 调用发送,如果失败,标志busy pend,自动进行重试
      if (!backend_->send(frame)) {
        ++tx_fail_;
        tx_pend_.store(1, std::memory_order_release);
        break;
      }
      ++tx_frames_;
    }
  } while (tx_pend_.exchange(0, std::memory_order_acq_rel) != 0);

  // 处理完释放锁
  tx_lock_.store(0, std::memory_order_release);
}
