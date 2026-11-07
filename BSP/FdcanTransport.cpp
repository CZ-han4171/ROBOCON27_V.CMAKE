/**
 * @file FdcanTransport.cpp
 * @brief ICanTransport 的片上 FDCAN 实现
 */
#include "FdcanTransport.hpp"

#include <cstring>

namespace BspCan {

namespace {
// CanBus 的通道号 -> HAL 的 FIFO 常量
uint32_t fifoFromChannel(uint8_t channel) {
  return (channel == 1U) ? FDCAN_RX_FIFO1 : FDCAN_RX_FIFO0;
}
} // namespace

bool FdcanTransport::send(const FdFrame &frame) {
  if (h_ == nullptr) {
    return false;
  }

  // 上层设备类的 onRx 只有 8 字节，FD 帧暂时不走这条路径
  const uint8_t len = (frame.len > 8U) ? 8U : frame.len;

  FDCAN_TxHeaderTypeDef tx_header = {0};
  tx_header.Identifier = frame.id;
  tx_header.IdType =
      frame.ext ? FDCAN_EXTENDED_ID : FDCAN_STANDARD_ID;
  tx_header.TxFrameType = frame.rtr ? FDCAN_REMOTE_FRAME : FDCAN_DATA_FRAME;
  tx_header.DataLength = lenToDlc(len);
  tx_header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
  tx_header.BitRateSwitch = FDCAN_BRS_OFF;
  tx_header.FDFormat = FDCAN_CLASSIC_CAN;
  tx_header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
  tx_header.MessageMarker = 0;

  uint8_t data[8] = {0};
  std::memcpy(data, frame.data, len);

  return HAL_FDCAN_AddMessageToTxFifoQ(h_, &tx_header, data) == HAL_OK;
}

bool FdcanTransport::readFifo(uint32_t fifo, FdFrame &frame) {
  if (h_ == nullptr) {
    return false;
  }

  FDCAN_RxHeaderTypeDef rx_header = {0};
  uint8_t data[8] = {0};
  if (HAL_FDCAN_GetRxMessage(h_, fifo, &rx_header, data) != HAL_OK) {
    return false;
  }

  frame.id = rx_header.Identifier;
  frame.ext = (rx_header.IdType == FDCAN_EXTENDED_ID);
  frame.len = dlcToLen(static_cast<uint8_t>(rx_header.DataLength));
  frame.fdf = (rx_header.FDFormat == FDCAN_FD_CAN);
  frame.brs = (rx_header.BitRateSwitch == FDCAN_BRS_ON);
  frame.rtr = (rx_header.RxFrameType == FDCAN_REMOTE_FRAME);
  std::memcpy(frame.data, data, 8);
  return true;
}

bool FdcanTransport::recvFrom(uint8_t channel, FdFrame &frame) {
  return readFifo(fifoFromChannel(channel), frame);
}

uint32_t FdcanTransport::txFreeLevel() {
  if (h_ == nullptr) {
    return 0;
  }
  return HAL_FDCAN_GetTxFifoFreeLevel(h_);
}

bool FdcanTransport::busHealthy() const {
  if (h_ == nullptr || h_->Instance == nullptr) {
    return false;
  }
  // BO = bus-off，EP = error passive
  const uint32_t psr = h_->Instance->PSR;
  return (psr & (FDCAN_PSR_BO | FDCAN_PSR_EP)) == 0U;
}

void FdcanTransport::errorCounters(uint8_t &tec, uint8_t &rec) const {
  tec = 0;
  rec = 0;
  if (h_ == nullptr || h_->Instance == nullptr) {
    return;
  }
  const uint32_t ecr = h_->Instance->ECR;
  tec = static_cast<uint8_t>((ecr >> 16) & 0xFFU);  // TEC[23:16]
  rec = static_cast<uint8_t>((ecr >> 8) & 0x7FU);   // REC[14:8]
}

} // namespace BspCan
